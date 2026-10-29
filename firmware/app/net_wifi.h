/* Core-0 WiFi orchestration. Owner initializes CYW43 first, then calls every
 * bounded iteration. It must process loss/clear readiness before the next call.
 * Production loop wiring and persistent AP provisioning are separate stages. */
#pragma once
#include "umb_net_fsm.h"
#include "umb_settings.h"
#include "hal_wifi.h"
typedef struct {
    umb_wifi_state_t policy;
    hal_wifi_status_t status;
    uint8_t deferred_actions;
    bool action_error;
} net_wifi_t;
typedef struct { bool loss, needs_teardown, action_failed; } net_wifi_result_t;
void net_wifi_init(net_wifi_t *state);
/* resources_busy includes partial entities, live session and any owned socket.
 * Reconfiguration is deferred until they have all been torn down/closed.
 * Network settings edits must be staged by the owner until that teardown. */
net_wifi_result_t net_wifi_step(net_wifi_t *state, const umb_settings_t *settings,
                                bool resources_busy);
