/* Fake-time end-to-end tests of the real network owner, WiFi/endpoint bridges,
 * provisioning coordinator and shared state. No radio, socket or flash access;
 * all passwords are synthetic. */
#include "test_runner.h"
#include "net_owner.h"
#include "shared_state.h"
#include "hal_dhcp_server.h"
#include "hal_flash_store.h"
#include "hal_rand.h"
#include "hal_udp_transport.h"
#include "umb_network_identity.h"
#include "fake_ros_backend.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define STA_AGENT 0x0a000005u
#define PHONE_IP 0xc0a80402u
#define PC_IP 0xc0a80403u
#define SYNTH_STA_PASS "synthetic-sta-pass"
#define SYNTH_AP_PASS "synthetic-ap-pass"

static uint32_t now;
static bool sta_started, ap_started, sta_reachable, ap_ok, rand_ok, flash_ok, close_ok;
static unsigned sta_starts, ap_starts, stops_with_link_up, stops, flash_calls, probe_calls;
static unsigned blocking_calls_in_step, max_blocking_in_step;
static char ap_pass_used[64];
static umb_settings_t flashed, persisted;
static hal_dhcp_client_t leases[UMB_DHCP_LEASES];
static size_t lease_count;
static uint8_t assoc[HAL_WIFI_AP_CLIENTS][6];
static int assoc_count;
static uint32_t agent_ip, broadcast_reply, last_probe_ip;
static uint16_t agent_port, last_probe_port;
static bool last_probe_broadcast;
static fake_ros_t ros;
static ros_backend_t ros_backend;

uint32_t hal_now_ms(void) { return now; }
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelay(TickType_t ticks) {
    now += ticks; /* simulated control tick acknowledges neutral */
    shared_control_mailbox_t box; shared_take_control_mailbox(&box);
    if (box.maintenance) shared_ack_maintenance(box.maintenance_id);
}
static void note_stop(void) {
    bool wifi, agent; shared_get_link(&wifi, &agent);
    stops++; if (wifi) stops_with_link_up++;
}
bool hal_wifi_start_sta(const char *ssid, const char *password) {
    sta_starts++; blocking_calls_in_step++; sta_started = ssid && password && ssid[0]; return sta_started;
}
void hal_wifi_stop_sta(void) { note_stop(); sta_started = false; }
bool hal_wifi_start_ap(const char *ssid, const char *password) {
    ap_starts++; blocking_calls_in_step++;
    if (!ssid || strncmp(ssid, "umbreon-", 8) || !password || strlen(password) < 8) return false;
    strcpy(ap_pass_used, password); ap_started = ap_ok; return ap_ok;
}
void hal_wifi_stop_ap(void) { note_stop(); ap_started = false; }
void hal_wifi_status(hal_wifi_status_t *out) {
    memset(out, 0, sizeof *out);
    out->sta_up = sta_started && sta_reachable; out->ap_up = ap_started;
    if (out->sta_up) { out->sta_ip = 0x0a000002u; out->sta_mask = 0xffffff00u; }
    if (out->ap_up) out->ap_ip = 0xc0a80401u;
}
bool hal_wifi_mac(uint8_t mac[6]) {
    const uint8_t value[6] = {2, 1, 2, 3, 0xab, 0xcd}; memcpy(mac, value, 6); return true;
}
int hal_wifi_ap_clients(uint8_t macs[HAL_WIFI_AP_CLIENTS][6]) {
    if (!ap_started) return -1;
    memcpy(macs, assoc, sizeof assoc); return assoc_count;
}
size_t hal_dhcp_server_clients(hal_dhcp_client_t out[UMB_DHCP_LEASES]) {
    memcpy(out, leases, sizeof leases); return lease_count;
}
bool hal_udp_probe(uint32_t ip, uint16_t port, unsigned timeout, bool broadcast, uint32_t *found) {
    probe_calls++; blocking_calls_in_step++;
    last_probe_ip = ip; last_probe_port = port; last_probe_broadcast = broadcast;
    now += timeout / 5u; /* a bounded socket wait */
    if (broadcast && broadcast_reply && port == agent_port) { *found = broadcast_reply; return true; }
    if (!broadcast && ip == agent_ip && port == agent_port) { *found = ip; return true; }
    return false;
}
bool hal_udp_probe_close(void) { return close_ok; }
bool hal_rand_fill16(uint8_t out[16]) {
    for (int i = 0; i < 16; i++) out[i] = (uint8_t)(0x35 + 17 * i);
    return rand_ok;
}
bool hal_settings_save(const umb_settings_t *s) {
    flash_calls++; blocking_calls_in_step++; flashed = *s;
    if (flash_ok) persisted = *s;
    return flash_ok;
}
bool hal_settings_persisted(umb_settings_t *out) { *out = persisted; return true; }

static umb_settings_t base(int mode) {
    now = 1000; sta_started = ap_started = false; sta_reachable = true;
    ap_ok = rand_ok = flash_ok = close_ok = true;
    sta_starts = ap_starts = stops_with_link_up = stops = flash_calls = probe_calls = 0;
    max_blocking_in_step = 0;
    memset(ap_pass_used, 0, sizeof ap_pass_used); memset(&flashed, 0, sizeof flashed);
    memset(leases, 0, sizeof leases); lease_count = 0;
    memset(assoc, 0, sizeof assoc); assoc_count = 0;
    agent_ip = STA_AGENT; agent_port = 8888; broadcast_reply = 0;
    umb_settings_t s = umb_settings_defaults();
    s.wifi_mode = (uint8_t)mode; s.sta_timeout_s = 5;
    strcpy(s.wifi_ssid, "synthetic-net"); strcpy(s.wifi_pass, SYNTH_STA_PASS);
    strcpy(s.ap_pass, SYNTH_AP_PASS);
    s.agent_ip = STA_AGENT; s.agent_port = 8888;
    return s;
}
static void count_blocking(void *user) { (void)user; blocking_calls_in_step++; }
static void boot(net_owner_t *o, const umb_settings_t *s) {
    fake_ros_reset(&ros); ros.on_blocking = count_blocking; ros_backend = fake_ros_backend(&ros);
    shared_init(s); persisted = *s; net_owner_init(o, &ros_backend); net_owner_set_radio_ready(o, true, 0);
}
static void step(net_owner_t *o) {
    blocking_calls_in_step = 0;
    net_owner_step(o);
    if (blocking_calls_in_step > max_blocking_in_step) max_blocking_in_step = blocking_calls_in_step;
    now += 10;
}
static void run(net_owner_t *o, uint32_t ms) {
    uint32_t end = now + ms;
    while ((int32_t)(now - end) < 0) step(o);
}
/* Simulated control tick: consumes the latched loss/disarm like control_task. */
static void control_tick(void) {
    shared_arm_request_t r[SHARED_ARM_QUEUE_LEN]; umb_arm_result_t res[SHARED_ARM_QUEUE_LEN];
    unsigned n = 0;
    while (n < SHARED_ARM_QUEUE_LEN && shared_take_arm_record(&r[n], now)) res[n++] = UMB_ARM_OK;
    umb_outputs_t idle = {.esc_us = 1500}; umb_arming_t arming = {0};
    (void)shared_commit_arm_batch(r, res, n, &idle, &arming, now);
    shared_publish_outputs(&idle);
    shared_control_mailbox_t box; shared_take_control_mailbox(&box);
}
static bool wifi_link(void) { bool w, a; shared_get_link(&w, &a); return w; }
static bool agent_link(void) { bool w, a; shared_get_link(&w, &a); return a; }
static shared_network_t net(void) { shared_network_t n; shared_get_network(&n); return n; }
static void add_client(size_t i, uint32_t ip, uint8_t last) {
    leases[i] = (hal_dhcp_client_t){.ip = ip, .mac = {2, 1, 2, 3, 4, last}};
    memcpy(assoc[i], leases[i].mac, 6);
    if (lease_count < i + 1) lease_count = i + 1;
    if (assoc_count < (int)(i + 1)) assoc_count = (int)(i + 1);
}

TEST(test_offline_radio_never_touches_adapters) {
    umb_settings_t s = base(UMB_WIFI_AUTO); net_owner_t o;
    fake_ros_reset(&ros); ros_backend = fake_ros_backend(&ros);
    shared_init(&s); net_owner_init(&o, &ros_backend);
    for (int i = 0; i < 100; i++) step(&o);
    ASSERT_EQ(sta_starts + ap_starts + probe_calls + flash_calls, 0);
    ASSERT_FALSE(wifi_link()); ASSERT_FALSE(net().radio_ready);
    ASSERT_EQ(ros.creates, 0);
}
TEST(test_sta_join_is_async_and_configured_ip_selected) {
    umb_settings_t s = base(UMB_WIFI_AUTO); net_owner_t o; boot(&o, &s);
    sta_reachable = false;
    step(&o); step(&o); ASSERT_EQ(sta_starts, 1);
    /* Joining: the async join start was the only blocking call; no probe, no flash. */
    for (int i = 0; i < 50; i++) step(&o);
    ASSERT_EQ(max_blocking_in_step, 1); ASSERT_FALSE(wifi_link());
    sta_reachable = true; step(&o);
    ASSERT_TRUE(wifi_link()); ASSERT_EQ(net().wifi_stage, UMB_WIFI_ON_STA);
    run(&o, 100);
    ASSERT_EQ(last_probe_ip, STA_AGENT); ASSERT_FALSE(last_probe_broadcast);
    shared_network_t n = net();
    ASSERT_TRUE(n.endpoint_selected); ASSERT_EQ(n.endpoint_ip, STA_AGENT);
    ASSERT_EQ(n.endpoint_method, UMB_ENDPOINT_CONFIGURED); ASSERT_EQ(n.local_ip, 0x0a000002u);
    /* Selection alone never marks the agent session up: entities come first. */
    ASSERT_FALSE(agent_link()); ASSERT_TRUE(ros.live != (1u << ROS_ENT_COUNT) - 1u);
    unsigned probes = probe_calls; run(&o, 2000); ASSERT_EQ(probe_calls, probes);
    ASSERT_EQ(flash_calls, 0); /* provisioned device: no boot write */
    ASSERT_TRUE(max_blocking_in_step <= 1);
}
TEST(test_sta_broadcast_rounds_are_bounded_and_retry) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); s.agent_ip = 0; agent_ip = 0;
    net_owner_t o; boot(&o, &s);
    run(&o, 100); ASSERT_TRUE(wifi_link());
    unsigned first = probe_calls; ASSERT_TRUE(first >= 1);
    ASSERT_EQ(last_probe_ip, 0x0a0000ffu); ASSERT_TRUE(last_probe_broadcast);
    run(&o, 5000);
    /* One broadcast round per second at most, the task never blocks longer. */
    ASSERT_TRUE(probe_calls - first <= 5); ASSERT_TRUE(probe_calls - first >= 4);
    ASSERT_FALSE(net().endpoint_selected); ASSERT_TRUE(max_blocking_in_step <= 1);
    broadcast_reply = 0x0a000009u; run(&o, 1100);
    ASSERT_TRUE(net().endpoint_selected); ASSERT_EQ(net().endpoint_ip, 0x0a000009u);
    ASSERT_EQ(net().endpoint_method, UMB_ENDPOINT_BROADCAST);
}
TEST(test_sta_timeout_falls_back_to_ap_phone_first_pc_second) {
    umb_settings_t s = base(UMB_WIFI_AUTO); net_owner_t o; boot(&o, &s);
    sta_reachable = false; agent_ip = PC_IP;
    run(&o, 4990); ASSERT_EQ(ap_starts, 0); ASSERT_EQ(net().wifi_stage, UMB_WIFI_JOINING);
    run(&o, 100); ASSERT_EQ(ap_starts, 1); ASSERT_TRUE(wifi_link());
    ASSERT_TRUE(strcmp(ap_pass_used, SYNTH_AP_PASS) == 0);
    ASSERT_TRUE(strcmp(net().ap_ssid, "umbreon-ABCD") == 0);
    /* Phone leases first and never answers XRCE; PC leases second. */
    add_client(0, PHONE_IP, 5); run(&o, 300);
    ASSERT_EQ(last_probe_ip, PHONE_IP); ASSERT_FALSE(net().endpoint_selected);
    add_client(1, PC_IP, 6); run(&o, 1500);
    shared_network_t n = net();
    ASSERT_TRUE(n.endpoint_selected); ASSERT_EQ(n.endpoint_ip, PC_IP);
    ASSERT_EQ(n.endpoint_method, UMB_ENDPOINT_DHCP);
    /* The STA fixed IP is never probed in AP mode. */
    ASSERT_TRUE(last_probe_ip != STA_AGENT);
    /* PC disconnects: selection invalidated, probing resumes. */
    assoc_count = 1; run(&o, 100);
    ASSERT_FALSE(net().endpoint_selected); ASSERT_EQ(net().endpoint_losses, 1);
    assoc_count = 2; run(&o, 1500); ASSERT_TRUE(net().endpoint_selected);
    /* Lease expiry also invalidates. */
    lease_count = 1; run(&o, 100); ASSERT_FALSE(net().endpoint_selected);
    /* No agent session: station retried after 60 s, loss latched before radio stop. */
    sta_reachable = true; uint32_t gen = shared_session_generation();
    run(&o, 60000);
    ASSERT_EQ(sta_starts, 2); ASSERT_EQ(stops_with_link_up, 0);
    ASSERT_TRUE(shared_session_generation() != gen);
    run(&o, 50); ASSERT_EQ(net().wifi_stage, UMB_WIFI_ON_STA); ASSERT_FALSE(ap_started);
}
TEST(test_missing_ap_pass_provisioned_once_then_ap_starts) {
    umb_settings_t s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; net_owner_t o; boot(&o, &s);
    run(&o, 50);
    ASSERT_EQ(flash_calls, 1); ASSERT_EQ(ap_starts, 1); ASSERT_TRUE(ap_started);
    ASSERT_EQ(max_blocking_in_step, 1); /* flash write and AP start in separate iterations */
    ASSERT_EQ(strlen(ap_pass_used), UMB_AP_PASSWORD_LEN);
    ASSERT_TRUE(strcmp(flashed.ap_pass, ap_pass_used) == 0);
    ASSERT_TRUE(strcmp(flashed.wifi_pass, SYNTH_STA_PASS) == 0); /* complete settings */
    umb_settings_t now_s; shared_get_settings(&now_s);
    ASSERT_TRUE(strcmp(now_s.ap_pass, ap_pass_used) == 0);
    ASSERT_FALSE(shared_maintenance_active());
    ASSERT_EQ(net().ap_pass_stage, UMB_PROV_DONE);
    /* Provisioning commit is not mistaken for an owner edit: no radio restart. */
    run(&o, 1000); ASSERT_EQ(ap_starts, 1); ASSERT_EQ(o.reconfigures, 0);
    /* Reboot with the persisted settings: no second write. */
    umb_settings_t persisted = flashed; ap_started = false;
    boot(&o, &persisted); run(&o, 1000);
    ASSERT_EQ(flash_calls, 1); ASSERT_EQ(ap_starts, 2);
}
TEST(test_provision_failure_keeps_ap_closed_without_write_loop) {
    umb_settings_t s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; flash_ok = false;
    net_owner_t o; boot(&o, &s);
    run(&o, 600000);
    ASSERT_EQ(flash_calls, UMB_PROV_MAX_WRITES);
    ASSERT_FALSE(ap_started); ASSERT_EQ(ap_starts, 0); /* never an open/unknown AP */
    ASSERT_FALSE(wifi_link()); ASSERT_FALSE(shared_maintenance_active());
    shared_network_t n = net();
    ASSERT_EQ(n.ap_pass_stage, UMB_PROV_GAVE_UP); ASSERT_EQ(n.ap_pass_failures, UMB_PROV_MAX_WRITES);
    ASSERT_EQ(n.ap_pass_last, UMB_PROV_RESULT_FLASH_FAILED);
    ASSERT_TRUE(max_blocking_in_step <= 1);
    /* Armed car: reservation refused, nothing written, retried later. */
    s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; boot(&o, &s);
    umb_outputs_t armed = {.armed = true}; shared_publish_outputs(&armed);
    run(&o, 5000); ASSERT_EQ(flash_calls, 0); ASSERT_EQ(net().ap_pass_last, UMB_PROV_RESULT_BUSY);
    umb_outputs_t idle = {.esc_us = 1500}; shared_publish_outputs(&idle);
    run(&o, 10000); ASSERT_EQ(flash_calls, 1); ASSERT_TRUE(ap_started);
    /* Entropy failure: nothing written, AP stays closed. */
    s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; rand_ok = false; boot(&o, &s);
    run(&o, 5000); ASSERT_EQ(flash_calls, 0); ASSERT_FALSE(ap_started);
    ASSERT_EQ(net().ap_pass_last, UMB_PROV_RESULT_NO_ENTROPY);
}
TEST(test_auto_without_ap_pass_writes_only_when_ap_needed) {
    umb_settings_t s = base(UMB_WIFI_AUTO); s.ap_pass[0] = 0; net_owner_t o; boot(&o, &s);
    run(&o, 5000); ASSERT_EQ(flash_calls, 0); ASSERT_TRUE(wifi_link()); /* STA joined */
    s = base(UMB_WIFI_STA_ONLY); s.ap_pass[0] = 0; sta_reachable = false; boot(&o, &s);
    run(&o, 20000); ASSERT_EQ(flash_calls, 0); ASSERT_EQ(ap_starts, 0);
}
TEST(test_owner_setting_change_staged_until_socket_closed) {
    umb_settings_t s = base(UMB_WIFI_AUTO); net_owner_t o; boot(&o, &s);
    run(&o, 200); ASSERT_TRUE(wifi_link()); ASSERT_TRUE(net().endpoint_selected);
    /* Endpoint-only edit: reprobed on the new port without touching the radio. */
    umb_settings_t edit; uint32_t rev = shared_settings_snapshot(&edit);
    edit.agent_port = 7777; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    run(&o, 30); ASSERT_FALSE(net().endpoint_selected); ASSERT_EQ(stops, 0);
    agent_port = 7777; run(&o, 1200);
    ASSERT_TRUE(net().endpoint_selected); ASSERT_EQ(last_probe_port, 7777);
    /* WiFi edit while a probe socket could not be closed: link drops at once,
     * radio is reconfigured only after the socket is released. */
    close_ok = false; rev = shared_settings_snapshot(&edit);
    edit.agent_port = 6666; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    run(&o, 200); ASSERT_TRUE(net().probe_owned);
    unsigned probes = probe_calls; run(&o, 2000); ASSERT_EQ(probe_calls, probes); /* no reuse */
    uint32_t gen = shared_session_generation();
    rev = shared_settings_snapshot(&edit);
    edit.wifi_mode = UMB_WIFI_AP_ONLY; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    run(&o, 500);
    ASSERT_FALSE(wifi_link()); ASSERT_TRUE(shared_session_generation() != gen);
    ASSERT_EQ(stops, 0); ASSERT_TRUE(sta_started); ASSERT_EQ(ap_starts, 0);
    close_ok = true; run(&o, 100);
    ASSERT_EQ(stops_with_link_up, 0); ASSERT_FALSE(sta_started);
    ASSERT_EQ(ap_starts, 1); ASSERT_TRUE(ap_started); ASSERT_EQ(o.reconfigures, 1);
    ASSERT_EQ(net().wifi_mode, UMB_WIFI_AP_ONLY);
}
TEST(test_sta_link_loss_latched_before_rejoin) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    run(&o, 200); ASSERT_TRUE(wifi_link());
    uint32_t gen = shared_session_generation();
    sta_reachable = false; step(&o);
    ASSERT_FALSE(wifi_link()); ASSERT_TRUE(shared_session_generation() != gen);
    ASSERT_EQ(net().link_losses, 1); ASSERT_FALSE(net().endpoint_selected);
    run(&o, 100); ASSERT_EQ(stops_with_link_up, 0); ASSERT_EQ(sta_starts, 2);
    sta_reachable = true; run(&o, 300); ASSERT_TRUE(wifi_link()); ASSERT_TRUE(net().endpoint_selected);
    net_owner_record_loop(&o, 7); net_owner_record_loop(&o, 3); step(&o);
    ASSERT_EQ(net().loop_last_ms, 3); ASSERT_EQ(net().loop_max_ms, 7);
}

TEST(test_defaults_keep_password_and_never_reprovision) {
    umb_settings_t s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; net_owner_t o; boot(&o, &s);
    run(&o, 100); ASSERT_EQ(flash_calls, 1);
    char first[64]; strcpy(first, ap_pass_used);
    /* Console `defaults` (unsaved): AP restarts with the same password, no write. */
    umb_settings_t cur; uint32_t rev = shared_settings_snapshot(&cur);
    umb_settings_t d = umb_settings_defaults_keeping_ap_pass(&cur);
    d.wifi_mode = UMB_WIFI_AP_ONLY;
    ASSERT_TRUE(shared_commit_settings(&d, rev, false));
    run(&o, 2000);
    ASSERT_EQ(flash_calls, 1); ASSERT_EQ(o.reconfigures, 1); ASSERT_EQ(ap_starts, 2);
    ASSERT_TRUE(strcmp(ap_pass_used, first) == 0);
    /* Even if RAM loses ap_pass by another path, the persisted one is restored. */
    control_tick(); rev = shared_settings_snapshot(&cur); cur.ap_pass[0] = 0;
    ASSERT_TRUE(shared_commit_settings(&cur, rev, false));
    for (int i = 0; i < 300; i++) { step(&o); control_tick(); } /* control consumes the loss */
    ASSERT_EQ(flash_calls, 1); ASSERT_TRUE(ap_started); ASSERT_EQ(o.reconfigures, 2);
    ASSERT_TRUE(strcmp(ap_pass_used, first) == 0);
    /* Fresh boot of a provisioned device whose RAM copy is empty: no write either. */
    umb_settings_t ram = persisted; ram.ap_pass[0] = 0;
    shared_init(&ram); net_owner_init(&o, &ros_backend); net_owner_set_radio_ready(&o, true, 0);
    ap_started = false; run(&o, 20000);
    ASSERT_EQ(flash_calls, 1); ASSERT_TRUE(ap_started); ASSERT_TRUE(strcmp(ap_pass_used, first) == 0);
}
TEST(test_unsaved_edit_not_flashed_by_provisioning) {
    umb_settings_t s = base(UMB_WIFI_AP_ONLY); s.ap_pass[0] = 0; net_owner_t o; boot(&o, &s);
    umb_settings_t edit; uint32_t rev = shared_settings_snapshot(&edit);
    edit.agent_port = 4242; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    run(&o, 100);
    ASSERT_EQ(flash_calls, 1); ASSERT_EQ(flashed.agent_port, 8888);
    ASSERT_TRUE(strcmp(flashed.ap_pass, ap_pass_used) == 0);
    umb_settings_t now_s; shared_get_settings(&now_s); ASSERT_EQ(now_s.agent_port, 4242);
}
TEST(test_radio_starts_deferred_while_armed) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    umb_outputs_t armed = {.armed = true}; shared_publish_outputs(&armed);
    run(&o, 3000); ASSERT_EQ(sta_starts, 0); ASSERT_FALSE(wifi_link());
    umb_outputs_t idle = {.esc_us = 1500}; shared_publish_outputs(&idle);
    run(&o, 100); ASSERT_EQ(sta_starts, 1); ASSERT_TRUE(wifi_link());
    /* Link loss while armed: loss latched at once, the rejoin waits for disarm. */
    shared_publish_outputs(&armed); sta_reachable = false; run(&o, 3000);
    ASSERT_FALSE(wifi_link()); ASSERT_EQ(sta_starts, 1);
    shared_publish_outputs(&idle); sta_reachable = true; run(&o, 200);
    ASSERT_EQ(sta_starts, 2); ASSERT_TRUE(wifi_link()); ASSERT_TRUE(max_blocking_in_step <= 1);
}

/* ---- ROS session integration -------------------------------------------- */
static unsigned destroys_with_agent_up, destroys_before_loss;
static uint32_t gen_at_ready;
static void check_destroy(void *user, ros_entity_id_t id) {
    (void)id; (void)user;
    bool w, a; shared_get_link(&w, &a);
    if (a) destroys_with_agent_up++;
    if (shared_session_generation() == gen_at_ready) destroys_before_loss++;
}
static bool run_until_agent(net_owner_t *o, unsigned max_steps) {
    for (unsigned i = 0; i < max_steps; i++) { step(o); if (agent_link()) return true; }
    return false;
}
TEST(test_agent_up_only_after_full_graph_with_current_generation) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); s.ros_domain_id = 17; net_owner_t o; boot(&o, &s);
    ASSERT_TRUE(run_until_agent(&o, 200));
    ASSERT_EQ(ros.live, (1u << ROS_ENT_COUNT) - 1u);
    uint32_t gen = shared_session_generation();
    ASSERT_EQ(ros.generation, gen); ASSERT_TRUE(shared_session_ready(gen));
    ASSERT_EQ(ros.last_cfg.agent_ip, STA_AGENT); ASSERT_EQ(ros.last_cfg.agent_port, 8888);
    ASSERT_EQ(ros.last_cfg.domain_id, 17);
    uint8_t mac[6] = {2, 1, 2, 3, 0xab, 0xcd};
    ASSERT_EQ(ros.last_cfg.client_key, umb_network_client_key(mac));
    shared_network_t n = net();
    ASSERT_TRUE(n.session_ready); ASSERT_EQ(n.session_generation, gen); ASSERT_EQ(n.sessions, 1);
    run(&o, 1000); ASSERT_TRUE(agent_link()); ASSERT_TRUE(net().time_synced);
    ASSERT_TRUE(max_blocking_in_step <= 1);
}
TEST(test_agent_loss_latched_before_entities_finalized_then_fresh_session) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    ASSERT_TRUE(run_until_agent(&o, 200));
    gen_at_ready = shared_session_generation();
    destroys_with_agent_up = destroys_before_loss = 0; ros.on_destroy = check_destroy;
    ros.agent_alive = false;
    unsigned guard = 0;
    while (agent_link() && guard++ < 60) step(&o);
    ASSERT_FALSE(agent_link()); ASSERT_TRUE(guard * 10 <= 400); /* loss declared within 400 ms */
    ASSERT_EQ(ros.live, 0); ASSERT_FALSE(ros.transport_open);
    ASSERT_EQ(destroys_with_agent_up, 0); ASSERT_EQ(destroys_before_loss, 0);
    shared_control_mailbox_t box; shared_take_control_mailbox(&box);
    ASSERT_TRUE(box.loss_mask & UMB_STOP_AGENT); ASSERT_TRUE(wifi_link());
    ASSERT_FALSE(shared_session_ready(gen_at_ready));
    ros.agent_alive = true; unsigned creates = ros.creates;
    ASSERT_TRUE(run_until_agent(&o, 400));
    ASSERT_EQ(ros.creates - creates, ROS_ENT_COUNT); /* every entity created afresh */
    ASSERT_TRUE(ros.generation != gen_at_ready); ASSERT_TRUE(shared_session_ready(ros.generation));
    ASSERT_EQ(net().session_losses, 1); ASSERT_EQ(net().sessions, 2);
    ros.on_destroy = NULL;
}
TEST(test_wifi_loss_and_reconfigure_release_session_first) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    ASSERT_TRUE(run_until_agent(&o, 200));
    gen_at_ready = shared_session_generation();
    destroys_with_agent_up = destroys_before_loss = 0; ros.on_destroy = check_destroy;
    sta_reachable = false; step(&o);
    ASSERT_FALSE(wifi_link()); ASSERT_FALSE(agent_link()); ASSERT_EQ(ros.live, 0);
    ASSERT_EQ(destroys_with_agent_up, 0); ASSERT_EQ(destroys_before_loss, 0);
    sta_reachable = true; ASSERT_TRUE(run_until_agent(&o, 400));
    /* WiFi edit with a socket the stack refuses to close: radio waits for it. */
    control_tick(); /* control consumes the earlier loss (disarm) before the edit */
    ros.release_ok = false; gen_at_ready = shared_session_generation(); stops = 0;
    umb_settings_t edit; uint32_t rev = shared_settings_snapshot(&edit);
    edit.wifi_mode = UMB_WIFI_AP_ONLY; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    run(&o, 300);
    ASSERT_FALSE(agent_link()); ASSERT_FALSE(wifi_link()); ASSERT_EQ(ros.live, 0);
    ASSERT_EQ(stops, 0); ASSERT_EQ(o.reconfigures, 0); ASSERT_EQ(destroys_with_agent_up, 0);
    ros.release_ok = true; run(&o, 100);
    ASSERT_EQ(o.reconfigures, 1); ASSERT_EQ(stops_with_link_up, 0); ASSERT_FALSE(ros.transport_open);
    ros.on_destroy = NULL;
}
TEST(test_endpoint_edit_reconnects_session_on_new_port) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    ASSERT_TRUE(run_until_agent(&o, 200));
    umb_settings_t edit; uint32_t rev = shared_settings_snapshot(&edit);
    edit.agent_port = 7777; ASSERT_TRUE(shared_commit_settings(&edit, rev, false));
    step(&o); ASSERT_FALSE(agent_link()); ASSERT_EQ(ros.live, 0);
    agent_port = 7777; ASSERT_TRUE(run_until_agent(&o, 400));
    ASSERT_EQ(ros.last_cfg.agent_port, 7777);
}
TEST(test_session_reopen_waits_while_console_armed) {
    umb_settings_t s = base(UMB_WIFI_STA_ONLY); net_owner_t o; boot(&o, &s);
    ASSERT_TRUE(run_until_agent(&o, 200));
    umb_outputs_t armed = {.armed = true}; shared_publish_outputs(&armed);
    ros.agent_alive = false;
    unsigned guard = 0;
    while (agent_link() && guard++ < 60) step(&o);
    ASSERT_FALSE(agent_link()); ASSERT_EQ(ros.live, 0);
    ros.agent_alive = true; ros.len = 0;
    run(&o, 2000); /* bench-armed: no blocking session open */
    for (unsigned i = 0; i < ros.len; i++) ASSERT_TRUE(!(ros.op[i] == 'c' && ros.id[i] == ROS_ENT_CONTEXT));
    ASSERT_FALSE(agent_link()); ASSERT_TRUE(wifi_link());
    umb_outputs_t idle = {.esc_us = 1500}; shared_publish_outputs(&idle);
    ASSERT_TRUE(run_until_agent(&o, 400));
}
int main(void) {
    RUN_TEST(test_offline_radio_never_touches_adapters);
    RUN_TEST(test_sta_join_is_async_and_configured_ip_selected);
    RUN_TEST(test_sta_broadcast_rounds_are_bounded_and_retry);
    RUN_TEST(test_sta_timeout_falls_back_to_ap_phone_first_pc_second);
    RUN_TEST(test_missing_ap_pass_provisioned_once_then_ap_starts);
    RUN_TEST(test_provision_failure_keeps_ap_closed_without_write_loop);
    RUN_TEST(test_auto_without_ap_pass_writes_only_when_ap_needed);
    RUN_TEST(test_owner_setting_change_staged_until_socket_closed);
    RUN_TEST(test_sta_link_loss_latched_before_rejoin);
    RUN_TEST(test_defaults_keep_password_and_never_reprovision);
    RUN_TEST(test_unsaved_edit_not_flashed_by_provisioning);
    RUN_TEST(test_radio_starts_deferred_while_armed);
    RUN_TEST(test_agent_up_only_after_full_graph_with_current_generation);
    RUN_TEST(test_agent_loss_latched_before_entities_finalized_then_fresh_session);
    RUN_TEST(test_wifi_loss_and_reconfigure_release_session_first);
    RUN_TEST(test_endpoint_edit_reconnects_session_on_new_port);
    RUN_TEST(test_session_reopen_waits_while_console_armed);
    TEST_SUMMARY();
}
