#include "umb_net_fsm.h"
#include <string.h>
#define STA_RETRY_MS 60000u
#define AP_START_MS 3000u
#define MAX_BACKOFF_MS 8000u
#define SESSION_RETRY_MAX_MS 2000u

static bool due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
umb_wifi_state_t umb_wifi_init(void) {
    umb_wifi_state_t s = {.stage = UMB_WIFI_IDLE, .backoff_ms = 1000}; return s;
}
static umb_wifi_state_t wifi_start(umb_wifi_state_t s, const umb_wifi_config_t *c,
                                  uint32_t now, umb_wifi_out_t *out) {
    s.mode = c->mode;
    if (c->mode != UMB_WIFI_AP_ONLY && c->credentials) {
        s.stage = UMB_WIFI_JOINING; s.deadline_ms = now + c->sta_timeout_ms;
        out->actions |= UMB_NET_START_STA;
    } else if (c->mode != UMB_WIFI_STA_ONLY) {
        s.stage = UMB_WIFI_STARTING_AP; s.deadline_ms = now + AP_START_MS;
        out->actions |= UMB_NET_START_AP;
    } else {
        s.stage = UMB_WIFI_BACKOFF; s.retry_ap = false; s.deadline_ms = now + s.backoff_ms;
    }
    return s;
}
static umb_wifi_state_t wifi_backoff(umb_wifi_state_t s, uint32_t now, bool ap) {
    s.stage = UMB_WIFI_BACKOFF; s.retry_ap = ap; s.deadline_ms = now + s.backoff_ms;
    if (s.backoff_ms < MAX_BACKOFF_MS) s.backoff_ms *= 2;
    return s;
}
umb_wifi_state_t umb_wifi_step(umb_wifi_state_t s, const umb_wifi_config_t *c,
                               const umb_wifi_in_t *in, umb_wifi_out_t *out) {
    memset(out, 0, sizeof *out);
    if ((unsigned)c->mode > UMB_WIFI_AP_ONLY) return s;
    if (s.mode != c->mode && !in->session_busy && s.stage != UMB_WIFI_IDLE) {
        if (s.stage == UMB_WIFI_ON_AP || s.stage == UMB_WIFI_STARTING_AP) out->actions |= UMB_NET_STOP_AP;
        if (s.stage == UMB_WIFI_ON_STA || s.stage == UMB_WIFI_JOINING) out->actions |= UMB_NET_STOP_STA;
        out->loss = true;
        return wifi_start(umb_wifi_init(), c, in->now_ms, out);
    }
    switch (s.stage) {
    case UMB_WIFI_IDLE: return wifi_start(s, c, in->now_ms, out);
    case UMB_WIFI_JOINING:
        if (in->sta_up) { s.stage = UMB_WIFI_ON_STA; s.backoff_ms = 1000; }
        else if (in->error || due(in->now_ms, s.deadline_ms)) {
            out->actions = UMB_NET_STOP_STA;
            if (c->mode == UMB_WIFI_AUTO) {
                s.stage = UMB_WIFI_STARTING_AP; s.deadline_ms = in->now_ms + AP_START_MS;
                out->actions |= UMB_NET_START_AP;
            } else s = wifi_backoff(s, in->now_ms, false);
        }
        break;
    case UMB_WIFI_STARTING_AP:
        if (in->ap_up) {
            s.stage = UMB_WIFI_ON_AP; s.ap_retry_ms = in->now_ms + STA_RETRY_MS; s.backoff_ms = 1000;
        } else if (in->error || due(in->now_ms, s.deadline_ms)) {
            out->actions = UMB_NET_STOP_AP;
            /* AUTO with credentials rejoins the station first on the next attempt. */
            s = wifi_backoff(s, in->now_ms, c->mode != UMB_WIFI_AUTO || !c->credentials);
        }
        break;
    case UMB_WIFI_ON_STA:
        if (!in->sta_up) {
            out->loss = true; out->actions = UMB_NET_STOP_STA;
            s = wifi_start(s, c, in->now_ms, out);
        }
        break;
    case UMB_WIFI_ON_AP:
        if (!in->ap_up) {
            out->loss = true; out->actions = UMB_NET_STOP_AP;
            s.stage = UMB_WIFI_STARTING_AP; s.deadline_ms = in->now_ms + AP_START_MS;
            out->actions |= UMB_NET_START_AP;
        } else if (c->mode == UMB_WIFI_AUTO && c->credentials && !in->session_busy && due(in->now_ms, s.ap_retry_ms)) {
            out->loss = true; out->actions = UMB_NET_STOP_AP;
            s = wifi_start(s, c, in->now_ms, out);
        }
        break;
    case UMB_WIFI_BACKOFF:
        if (due(in->now_ms, s.deadline_ms)) {
            if (s.retry_ap && c->mode != UMB_WIFI_STA_ONLY) {
                s.stage = UMB_WIFI_STARTING_AP; s.deadline_ms = in->now_ms + AP_START_MS;
                out->actions = UMB_NET_START_AP;
            } else s = wifi_start(s, c, in->now_ms, out);
        }
        break;
    }
    return s;
}

umb_session_t umb_session_init(void) {
    umb_session_t s = {.stage = UMB_SESSION_OFF, .retry_ms = 100}; return s;
}
static umb_session_t session_backoff(umb_session_t s, uint32_t now) {
    s.stage = UMB_SESSION_BACKOFF; s.deadline_ms = now + s.retry_ms;
    if (s.retry_ms < SESSION_RETRY_MAX_MS) {
        s.retry_ms *= 2;
        if (s.retry_ms > SESSION_RETRY_MAX_MS) s.retry_ms = SESSION_RETRY_MAX_MS;
    }
    return s;
}
static umb_session_t session_begin(umb_session_t s, const umb_session_in_t *in, umb_session_out_t *out) {
    if (!in->wifi_up) { s.stage = UMB_SESSION_OFF; return s; }
    s.stage = in->endpoint_valid ? UMB_SESSION_PROBING : UMB_SESSION_FINDING;
    s.deadline_ms = in->now_ms + (in->endpoint_valid ? 100u : 1000u);
    out->action = in->endpoint_valid ? UMB_SESSION_ACT_PROBE : UMB_SESSION_ACT_FIND;
    return s;
}
static umb_session_t session_drop(umb_session_t s, uint32_t now, umb_session_out_t *out) {
    out->loss = true; out->ready = false;
    if (s.have_entities) {
        s.stage = UMB_SESSION_DESTROYING; s.deadline_ms = now + 1000u;
        out->action = UMB_SESSION_ACT_DESTROY;
    } else s = session_backoff(s, now);
    return s;
}
umb_session_t umb_session_step(umb_session_t s, const umb_session_in_t *in, umb_session_out_t *out) {
    *out = (umb_session_out_t){.ready = s.stage == UMB_SESSION_READY || s.stage == UMB_SESSION_PINGING};
    if (s.stage != UMB_SESSION_OFF && s.stage != UMB_SESSION_BACKOFF && s.stage != UMB_SESSION_DESTROYING &&
        (!in->wifi_up || in->endpoint_changed ||
         (s.stage != UMB_SESSION_FINDING && !in->endpoint_valid))) return session_drop(s, in->now_ms, out);
    switch (s.stage) {
    case UMB_SESSION_OFF: return session_begin(s, in, out);
    case UMB_SESSION_FINDING:
        if (in->find_done || due(in->now_ms, s.deadline_ms)) {
            if (in->find_done && in->endpoint_valid) return session_begin(s, in, out);
            s = session_backoff(s, in->now_ms);
        }
        break;
    case UMB_SESSION_PROBING:
        if (in->probe_done && in->probe_ok) {
            s.have_entities = true; s.stage = UMB_SESSION_CREATING;
            s.deadline_ms = in->now_ms + 5000u; out->action = UMB_SESSION_ACT_CREATE;
        } else if (in->probe_done || due(in->now_ms, s.deadline_ms)) s = session_backoff(s, in->now_ms);
        break;
    case UMB_SESSION_CREATING:
        if (in->create_done && in->create_ok) {
            s.stage = UMB_SESSION_READY; s.ping_failures = 0; s.retry_ms = 100;
            s.next_ping_ms = in->now_ms + UMB_NET_PING_PERIOD_MS; out->ready = true;
        } else if (in->create_done || due(in->now_ms, s.deadline_ms)) return session_drop(s, in->now_ms, out);
        break;
    case UMB_SESSION_READY:
        if (due(in->now_ms, s.next_ping_ms)) {
            s.stage = UMB_SESSION_PINGING; s.deadline_ms = in->now_ms + UMB_NET_PING_TIMEOUT_MS;
            out->action = UMB_SESSION_ACT_PING;
        }
        break;
    case UMB_SESSION_PINGING:
        if (in->ping_done || due(in->now_ms, s.deadline_ms)) {
            if (in->ping_done && in->ping_ok) s.ping_failures = 0;
            else s.ping_failures++;
            if (s.ping_failures >= UMB_NET_PING_FAIL_LIMIT) return session_drop(s, in->now_ms, out);
            s.stage = UMB_SESSION_READY; s.next_ping_ms = in->now_ms + UMB_NET_PING_PERIOD_MS;
        }
        break;
    case UMB_SESSION_DESTROYING:
        if (in->destroy_done) { s.have_entities = false; s = session_backoff(s, in->now_ms); }
        else if (due(in->now_ms, s.deadline_ms)) {
            /* Adapter teardown must be idempotent. Do not create over unreleased resources. */
            out->action = UMB_SESSION_ACT_DESTROY; s.deadline_ms = in->now_ms + 1000u;
        }
        break;
    case UMB_SESSION_BACKOFF:
        if (due(in->now_ms, s.deadline_ms)) return session_begin(s, in, out);
        break;
    }
    return s;
}
