#include "net_wifi.h"
#include "hal_time.h"
#include "umb_network_identity.h"
#include <string.h>
#define AP_START_MS 3000u
void net_wifi_init(net_wifi_t *s) {
    memset(s, 0, sizeof *s); s->policy = umb_wifi_init();
}
static bool apply_actions(uint8_t actions, const umb_settings_t *settings) {
    bool ok = true;
    if (actions & UMB_NET_STOP_STA) hal_wifi_stop_sta();
    if (actions & UMB_NET_STOP_AP) hal_wifi_stop_ap();
    if (actions & UMB_NET_START_STA)
        ok = hal_wifi_start_sta(settings->wifi_ssid, settings->wifi_pass);
    if (actions & UMB_NET_START_AP) {
        uint8_t mac[6]; umb_network_identity_t id;
        /* A missing password must be provisioned/persisted by the maintenance
         * coordinator first. Never silently start an open AP or write here. */
        bool identity_ok = settings->ap_pass[0] && hal_wifi_mac(mac) &&
            umb_network_identity(mac, NULL, settings->ap_pass, &id);
        ok = identity_ok && hal_wifi_start_ap(id.ssid, id.password) && ok;
    }
    return ok;
}
net_wifi_result_t net_wifi_step(net_wifi_t *s, const umb_settings_t *settings, bool resources_busy) {
    net_wifi_result_t result = {0};
    if (s->deferred_actions) {
        if (resources_busy) { result.needs_teardown = true; return result; }
        uint8_t actions = s->deferred_actions; s->deferred_actions = 0;
        s->action_error = !apply_actions(actions, settings);
        result.action_failed = s->action_error;
        /* The pure FSM decision may have waited for teardown. Its join/AP start
         * deadline begins at actual adapter invocation, not the earlier decision. */
        if (actions & UMB_NET_START_STA) s->policy.deadline_ms = hal_now_ms() + (uint32_t)settings->sta_timeout_s * 1000u;
        if (actions & UMB_NET_START_AP) s->policy.deadline_ms = hal_now_ms() + AP_START_MS;
        hal_wifi_status(&s->status);
        return result;
    }
    hal_wifi_status(&s->status);
    umb_wifi_config_t config = {.mode=(umb_wifi_mode_t)settings->wifi_mode,
        .credentials=settings->wifi_ssid[0] != 0, .sta_timeout_ms=(uint32_t)settings->sta_timeout_s * 1000u};
    umb_wifi_in_t in = {.now_ms=hal_now_ms(),.sta_up=s->status.sta_up,.ap_up=s->status.ap_up,
        .error=s->action_error,.session_busy=resources_busy};
    s->action_error = false;
    umb_wifi_out_t out;
    s->policy = umb_wifi_step(s->policy, &config, &in, &out);
    s->deferred_actions = out.actions;
    /* Always split the decision from I/O by an owner iteration. Even if a link
     * falls during creation, loss reaches control before disabling the radio. */
    result.loss = out.loss || (out.actions != 0 && resources_busy);
    result.needs_teardown = out.actions != 0 && resources_busy;
    return result;
}
