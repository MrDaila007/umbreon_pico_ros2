/* ros_session.c - see ros_session.h. Single core-0 owner; no shared state. */
#include "ros_session.h"
#include <string.h>

static bool due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

static bool same_config(const ros_session_config_t *a, const ros_session_config_t *b) {
    return a->agent_ip == b->agent_ip && a->agent_port == b->agent_port &&
           a->domain_id == b->domain_id && a->client_key == b->client_key;
}

void ros_session_init(ros_session_t *s) {
    memset(s, 0, sizeof *s);
    s->fsm = umb_session_init();
    ros_entities_init(&s->entities);
    s->sync = umb_time_sync_init();
}

bool ros_session_busy(const ros_session_t *s) { return ros_entities_busy(&s->entities); }

static void drop_ready(ros_session_t *s, const ros_backend_t *b) {
    if (s->ready) s->losses++;
    s->ready = false; s->generation = 0;
    b->set_generation(b->ctx, 0); /* in-flight callbacks can no longer match */
}

bool ros_session_teardown(ros_session_t *s, const ros_backend_t *b) {
    drop_ready(s, b);
    umb_time_sync_invalidate(&s->sync);
    bool released = ros_entities_destroy(&s->entities, b);
    s->teardowns++;
    s->pulse.destroy_done = released;
    if (s->fsm.stage != UMB_SESSION_DESTROYING) {
        /* Owner-forced teardown (link loss, reconfiguration): the policy must
         * not keep a ready view of entities that no longer exist. */
        s->fsm.stage = UMB_SESSION_DESTROYING; s->fsm.have_entities = true;
        s->fsm.deadline_ms = s->pulse.now_ms + 1000u;
    }
    return released;
}

static void create_one(ros_session_t *s, const ros_backend_t *b) {
    ros_create_result_t r = ros_entities_create_next(&s->entities, b, &s->cfg);
    if (r == ROS_CREATE_PROGRESS) return;
    s->pulse.create_done = true;
    s->pulse.create_ok = r == ROS_CREATE_DONE;
    if (!s->pulse.create_ok) s->create_failures++;
}

static void become_ready(ros_session_t *s, const ros_session_in_t *in, const ros_backend_t *b) {
    s->ready = true; s->sessions++; s->errors_in_row = 0;
    s->generation = in->generation;
    b->set_generation(b->ctx, in->generation);
    b->session_started(b->ctx);
    s->next_sync_ms = in->now_ms;
}

static void try_sync(ros_session_t *s, uint32_t now, const ros_backend_t *b) {
    int64_t epoch_ns = 0;
    uint64_t mono_ms = 0;
    if (b->sync_time(b->ctx, ROS_SYNC_TIMEOUT_MS, &epoch_ns, &mono_ms) &&
        umb_time_sync_set(&s->sync, epoch_ns, mono_ms)) {
        s->last_sync_ms = now; s->next_sync_ms = now + ROS_SYNC_PERIOD_MS;
        return;
    }
    umb_time_sync_failed(&s->sync); /* a previous valid offset is kept */
    s->sync_failures++;
    s->next_sync_ms = now + ROS_SYNC_RETRY_MS;
}

static void run_ready(ros_session_t *s, uint32_t now, bool pinged, const ros_backend_t *b) {
    if (b->spin(b->ctx) == ROS_SPIN_ERROR) {
        if (++s->errors_in_row >= ROS_SESSION_ERROR_LIMIT) {
            s->pulse.transport_failed = true; s->transport_failures++; s->errors_in_row = 0;
        }
    } else s->errors_in_row = 0;
    if (!pinged && due(now, s->next_sync_ms)) try_sync(s, now, b);
}

ros_session_out_t ros_session_step(ros_session_t *s, const ros_session_in_t *in, const ros_backend_t *b) {
    umb_session_in_t fin = s->pulse;
    fin.now_ms = in->now_ms; fin.wifi_up = in->link_up;
    fin.endpoint_valid = fin.find_done = in->endpoint_selected;
    fin.endpoint_changed = ros_session_busy(s) && !same_config(&s->cfg, &in->endpoint);
    memset(&s->pulse, 0, sizeof s->pulse);
    s->pulse.now_ms = in->now_ms;
    umb_session_out_t o;
    s->fsm = umb_session_step(s->fsm, &fin, &o);
    ros_session_out_t out = {0};
    if (o.loss) { drop_ready(s, b); out.loss = true; }
    bool pinged = false;
    switch (o.action) {
    case UMB_SESSION_ACT_PROBE: /* the endpoint was just XRCE-validated; the open re-pings */
        s->pulse.probe_done = s->pulse.probe_ok = true; break;
    case UMB_SESSION_ACT_CREATE:
        s->cfg = in->endpoint;
        umb_time_sync_invalidate(&s->sync);
        b->set_generation(b->ctx, 0);
        break;
    case UMB_SESSION_ACT_PING:
        pinged = true;
        s->pulse.ping_done = true;
        s->pulse.ping_ok = b->ping(b->ctx, UMB_NET_PING_TIMEOUT_MS);
        if (!s->pulse.ping_ok) s->ping_failures++;
        break;
    case UMB_SESSION_ACT_DESTROY: out.teardown = true; break;
    default: break;
    }
    if (s->fsm.stage == UMB_SESSION_CREATING) create_one(s, b);
    if (o.ready && !s->ready && !o.loss) become_ready(s, in, b);
    if (s->ready) run_ready(s, in->now_ms, pinged, b);
    out.ready = s->ready;
    return out;
}
