#include "net_owner.h"
#include "shared_state.h"
#include "settings_apply.h"
#include "hal_rand.h"
#include "hal_time.h"
#include "hal_udp_transport.h"
#include "umb_network_identity.h"
#include <string.h>

static bool due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

void net_owner_init(net_owner_t *o, const ros_backend_t *ros) {
    memset(o, 0, sizeof *o);
    o->ros = ros;
    ros_session_init(&o->session);
    net_wifi_init(&o->wifi);
    net_endpoint_init(&o->endpoint);
    o->provision = umb_ap_provision_init();
    shared_get_settings(&o->active);
}

void net_owner_set_radio_ready(net_owner_t *o, bool ready, uint32_t init_ms) {
    o->radio_ready = ready; o->radio_init_ms = init_ms;
    uint8_t mac[6];
    if (ready && !o->ap_ssid[0] && hal_wifi_mac(mac)) {
        (void)umb_network_ssid(mac, o->ap_ssid);
        o->client_key = umb_network_client_key(mac);
    }
}

void net_owner_record_loop(net_owner_t *o, uint32_t duration_ms) {
    o->loop_last_ms = duration_ms;
    if (duration_ms > o->loop_max_ms) o->loop_max_ms = duration_ms;
}

static bool wifi_fields_equal(const umb_settings_t *a, const umb_settings_t *b) {
    return a->wifi_mode == b->wifi_mode && a->sta_timeout_s == b->sta_timeout_s &&
           strncmp(a->wifi_ssid, b->wifi_ssid, sizeof a->wifi_ssid) == 0 &&
           strncmp(a->wifi_pass, b->wifi_pass, sizeof a->wifi_pass) == 0 &&
           strncmp(a->ap_pass, b->ap_pass, sizeof a->ap_pass) == 0;
}

static void copy_wifi_fields(umb_settings_t *dst, const umb_settings_t *src) {
    dst->wifi_mode = src->wifi_mode; dst->sta_timeout_s = src->sta_timeout_s;
    memcpy(dst->wifi_ssid, src->wifi_ssid, sizeof dst->wifi_ssid);
    memcpy(dst->wifi_pass, src->wifi_pass, sizeof dst->wifi_pass);
    memcpy(dst->ap_pass, src->ap_pass, sizeof dst->ap_pass);
}

/* Latch loss into shared state first: control sees it on its next tick even if
 * the link comes back before then, and no command of the old link survives.
 * agent_up only with WiFi and a ready session (full graph + executor). */
static void set_link(net_owner_t *o, bool up) {
    if (o->wifi_up && !up) o->link_losses++;
    shared_set_link(up, up && o->session.ready);
    o->wifi_up = up;
}

static bool session_held(const net_owner_t *o) {
    return o->session.ready || ros_session_busy(&o->session);
}

/* Tear down everything bound to the current link: session entities and its
 * socket first, then the probe socket. Callers latch the loss before. False
 * keeps ownership for a retry. */
static bool release_resources(net_owner_t *o) {
    bool session_released = !session_held(o) || ros_session_teardown(&o->session, o->ros);
    bool closed = net_endpoint_close(&o->endpoint);
    o->probe_owned = !closed;
    o->round_failures = 0;
    return closed && session_released;
}

/* Radio start/stop SDK calls block for hundreds of ms; never while armed. */
static bool radio_ops_allowed(void) {
    umb_outputs_t out;
    shared_get_outputs(&out);
    return !out.armed;
}

static void reconfigure(net_owner_t *o, const umb_settings_t *latest) {
    if (!radio_ops_allowed()) return; /* staged until disarmed */
    set_link(o, false);
    if (!release_resources(o)) return; /* staged: retried next iteration */
    hal_wifi_stop_sta();
    hal_wifi_stop_ap();
    copy_wifi_fields(&o->active, latest);
    net_wifi_init(&o->wifi);
    o->endpoint_mode = UMB_ENDPOINT_OFF;
    o->reconfigures++;
}

static umb_prov_result_t provision_once(net_owner_t *o) {
    uint8_t mac[6], entropy[16];
    umb_network_identity_t id;
    umb_prov_result_t r = UMB_PROV_RESULT_NO_ENTROPY;
    if (hal_wifi_mac(mac) && hal_rand_fill16(entropy) && umb_network_identity(mac, entropy, "", &id)) {
        switch (settings_provision_ap_pass(id.password)) {
        case SETTINGS_SAVE_OK:
            memcpy(o->active.ap_pass, id.password, sizeof o->active.ap_pass);
            r = UMB_PROV_RESULT_OK; break;
        case SETTINGS_SAVE_ALREADY_SET: {
            umb_settings_t current; shared_get_settings(&current);
            memcpy(o->active.ap_pass, current.ap_pass, sizeof o->active.ap_pass);
            memset(&current, 0, sizeof current);
            r = UMB_PROV_RESULT_ALREADY; break;
        }
        case SETTINGS_SAVE_BUSY:
        case SETTINGS_SAVE_NEUTRAL_TIMEOUT: r = UMB_PROV_RESULT_BUSY; break;
        default: r = UMB_PROV_RESULT_FLASH_FAILED; break;
        }
    }
    memset(&id, 0, sizeof id); memset(entropy, 0, sizeof entropy);
    return r;
}

/* Provisioned this boot but RAM lost the password: restore it, never regenerate. */
static void restore_pass(net_owner_t *o) {
    uint32_t now = hal_now_ms();
    if (!due(now, o->next_restore_ms)) return;
    o->next_restore_ms = now + NET_OWNER_RESTORE_RETRY_MS;
    settings_save_result_t r = settings_restore_ap_pass();
    if (r == SETTINGS_SAVE_OK || r == SETTINGS_SAVE_ALREADY_SET) {
        umb_settings_t current; shared_get_settings(&current);
        memcpy(o->active.ap_pass, current.ap_pass, sizeof o->active.ap_pass);
        memset(&current, 0, sizeof current);
    }
}

/* Only when the FSM is about to start the AP and no password exists. A failure
 * leaves ap_pass empty, so net_wifi refuses the AP start (never an open AP).
 * Returns true when it attempted blocking work: the caller ends the iteration
 * so the AP start happens on the next one, after a heartbeat. */
static bool maybe_provision(net_owner_t *o) {
    if (!(o->wifi.deferred_actions & UMB_NET_START_AP) || o->active.ap_pass[0]) return false;
    if (o->provision.stage == UMB_PROV_DONE) { restore_pass(o); return true; }
    if (!umb_ap_provision_due(o->provision, false, hal_now_ms())) return false;
    umb_prov_result_t r = provision_once(o);
    o->provision = umb_ap_provision_result(o->provision, r, hal_now_ms());
    return true;
}

static bool link_up(const net_owner_t *o) {
    return (o->wifi.policy.stage == UMB_WIFI_ON_STA && o->wifi.status.sta_up) ||
           (o->wifi.policy.stage == UMB_WIFI_ON_AP && o->wifi.status.ap_up);
}

static void run_endpoint(net_owner_t *o, bool up, bool may_block) {
    umb_endpoint_mode_t mode = !up ? UMB_ENDPOINT_OFF :
        o->wifi.policy.stage == UMB_WIFI_ON_STA ? UMB_ENDPOINT_STA : UMB_ENDPOINT_AP;
    uint32_t now = hal_now_ms();
    bool config_changed = o->endpoint.configured_ip != o->active.agent_ip ||
                          o->endpoint.port != o->active.agent_port;
    if (mode != o->endpoint_mode || config_changed || due(now, o->next_refresh_ms)) {
        if (net_endpoint_refresh(&o->endpoint, mode, o->active.agent_ip, o->active.agent_port))
            o->endpoint_losses++; /* run_session drops the session (loss first) */
        if (mode != o->endpoint_mode || config_changed) { o->round_failures = 0; o->next_probe_ms = now; }
        o->endpoint_mode = mode;
        o->next_refresh_ms = now + NET_OWNER_REFRESH_MS;
    }
    if (o->probe_owned) { o->probe_owned = !hal_udp_probe_close(); return; }
    if (!may_block || mode == UMB_ENDPOINT_OFF || o->endpoint.policy.selected || !due(now, o->next_probe_ms)) return;
    uint32_t before = o->endpoint.probes;
    bool selected = net_endpoint_probe_once(&o->endpoint);
    if (o->endpoint.probes == before) return;
    o->probe_owned = !hal_udp_probe_close(); /* never carry a socket across iterations */
    if (selected) { o->round_failures = 0; return; }
    size_t round = mode == UMB_ENDPOINT_AP && o->endpoint.input.lease_count ? o->endpoint.input.lease_count : 1u;
    if (++o->round_failures >= round) {
        /* A full bounded round failed: stay offline and retry later. */
        o->round_failures = 0;
        o->next_probe_ms = hal_now_ms() + NET_OWNER_ROUND_RETRY_MS;
    }
}

/* Session after endpoint refresh: a lost/changed endpoint drops it this
 * iteration. Skipped in radio start/stop iterations (one blocking op each). */
static void run_session(net_owner_t *o, bool up, bool may_block) {
    if (!may_block) return;
    ros_session_in_t in = {
        .now_ms = hal_now_ms(), .link_up = up,
        .endpoint_selected = o->endpoint.policy.selected && o->client_key != 0,
        .endpoint = {o->endpoint.policy.selected_ip, o->active.agent_port,
                     o->active.ros_domain_id, o->client_key},
        .generation = shared_session_generation(),
        .open_allowed = radio_ops_allowed()}; /* never open a session while armed */
    ros_session_out_t out = ros_session_step(&o->session, &in, o->ros);
    set_link(o, up); /* publishes agent_up = ready; a dropped session latches loss here */
    if (out.teardown) (void)ros_session_teardown(&o->session, o->ros);
}

static void publish_session(const ros_session_t *s, shared_network_t *n) {
    n->session_stage = (uint8_t)s->fsm.stage;
    n->session_ready = s->ready; n->session_busy = ros_session_busy(s);
    n->time_synced = s->sync.synced;
    n->sync_age_ms = s->sync.synced ? hal_now_ms() - s->last_sync_ms : 0;
    n->session_generation = s->generation; n->sessions = s->sessions;
    n->session_losses = s->losses; n->session_create_failures = s->create_failures;
    n->session_transport_failures = s->transport_failures;
    n->ping_failures = s->ping_failures; n->sync_failures = s->sync_failures;
}

static void publish(const net_owner_t *o) {
    shared_network_t n = {0};
    const hal_wifi_status_t *st = &o->wifi.status;
    n.wifi_mode = o->active.wifi_mode; n.wifi_stage = (uint8_t)o->wifi.policy.stage;
    n.radio_ready = o->radio_ready; n.wifi_up = o->wifi_up;
    n.sta_up = st->sta_up; n.ap_up = st->ap_up; n.rssi_valid = st->rssi_valid; n.rssi = st->rssi;
    n.local_ip = st->sta_up ? st->sta_ip : st->ap_up ? st->ap_ip : 0;
    n.endpoint_selected = o->endpoint.policy.selected;
    n.endpoint_method = (uint8_t)(n.endpoint_selected ? o->endpoint.policy.method : UMB_ENDPOINT_NONE);
    n.endpoint_ip = n.endpoint_selected ? o->endpoint.policy.selected_ip : 0;
    n.endpoint_port = o->active.agent_port;
    n.probe_owned = o->probe_owned;
    memcpy(n.ap_ssid, o->ap_ssid, sizeof n.ap_ssid);
    n.ap_pass_stage = (uint8_t)o->provision.stage; n.ap_pass_last = (uint8_t)o->provision.last;
    n.ap_pass_failures = o->provision.failures;
    n.probes = o->endpoint.probes; n.probe_failures = o->endpoint.failures;
    n.wifi_errors = o->wifi_errors; n.reconfigures = o->reconfigures;
    n.link_losses = o->link_losses; n.endpoint_losses = o->endpoint_losses;
    n.loop_last_ms = o->loop_last_ms; n.loop_max_ms = o->loop_max_ms; n.radio_init_ms = o->radio_init_ms;
    publish_session(&o->session, &n);
    shared_publish_network(&n);
}

void net_owner_step(net_owner_t *o) {
    umb_settings_t latest;
    shared_get_settings(&latest);
    /* Endpoint fields apply in place: reconcile cancels probes/selection. */
    o->active.agent_ip = latest.agent_ip; o->active.agent_port = latest.agent_port;
    o->active.ros_domain_id = latest.ros_domain_id; /* a change reconnects the session */
    if (!o->radio_ready) set_link(o, false);
    else if (!wifi_fields_equal(&o->active, &latest)) reconfigure(o, &latest);
    else if (!maybe_provision(o)) {
        uint8_t pending = o->wifi.deferred_actions;
        /* Armed: busy defers radio I/O and the autonomous AP->STA retry; loss is
         * still reported (and disarms), so the wait is about one control tick. */
        bool busy = o->probe_owned || session_held(o) || !radio_ops_allowed();
        net_wifi_result_t r = net_wifi_step(&o->wifi, &o->active, busy);
        if (r.action_failed) o->wifi_errors++;
        bool radio_io = pending && !o->wifi.deferred_actions;
        bool up = link_up(o) && !r.loss && !r.needs_teardown;
        set_link(o, up);
        if (r.loss || r.needs_teardown) (void)release_resources(o);
        run_endpoint(o, up, !radio_io);
        run_session(o, up, !radio_io);
    }
    memset(&latest, 0, sizeof latest);
    publish(o);
}
