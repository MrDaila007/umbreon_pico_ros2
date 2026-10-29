/* Single core-0 owner only. No ROS entities, PWM or flash operations here. */
#pragma once
#include "umb_endpoint.h"
#include "hal_wifi.h"
typedef struct {
    umb_endpoint_t policy;
    umb_endpoint_input_t input;
    hal_wifi_status_t wifi;
    umb_endpoint_mode_t requested_mode;
    uint32_t configured_ip;
    uint16_t port;
    uint32_t probes, failures;
} net_endpoint_t;
void net_endpoint_init(net_endpoint_t *state);
/* Returns selected-endpoint loss. Owner must clear readiness/notify control
 * before teardown/reconfiguration. Call before handling session events. */
bool net_endpoint_refresh(net_endpoint_t *state, umb_endpoint_mode_t active_mode,
                           uint32_t configured_ip, uint16_t port);
/* At most one dedicated XRCE probe, <=100ms requested timeout. Rechecks link,
 * lease and association after the probe before accepting its result. */
bool net_endpoint_probe_once(net_endpoint_t *state);
/* Caller notifies loss first, then tears down session, then closes probe before
 * radio reconfiguration. Failed close retains ownership; caller must retry. */
bool net_endpoint_close(net_endpoint_t *state);
