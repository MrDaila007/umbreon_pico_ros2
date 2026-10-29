/* Core-0 network orchestration: one bounded iteration per call. Sole caller of
 * the WiFi/endpoint adapters and of first-boot AP password provisioning.
 * Every step performs at most one blocking operation: one <=100 ms XRCE probe,
 * one maintenance-reserved flash write, or one radio start/stop. Radio
 * reconfiguration (SDK calls measured up to ~250 ms) only happens while
 * disarmed; STA joins are asynchronous.
 * Loss is latched into shared state before any teardown or radio change. */
#pragma once
#include "net_wifi.h"
#include "net_endpoint.h"
#include "umb_ap_provision.h"
#define NET_OWNER_REFRESH_MS 50u      /* lease/association snapshot period */
#define NET_OWNER_ROUND_RETRY_MS 1000u /* pause after a failed discovery round */
#define NET_OWNER_RESTORE_RETRY_MS 1000u
typedef struct {
    net_wifi_t wifi;
    net_endpoint_t endpoint;
    umb_ap_provision_t provision;
    umb_settings_t active; /* WiFi fields in force; owner edits staged until teardown */
    umb_endpoint_mode_t endpoint_mode;
    char ap_ssid[13];
    uint32_t next_probe_ms, next_refresh_ms, next_restore_ms;
    uint8_t round_failures;
    bool radio_ready, wifi_up, probe_owned;
    uint32_t wifi_errors, reconfigures, link_losses, endpoint_losses;
    uint32_t loop_last_ms, loop_max_ms, radio_init_ms;
} net_owner_t;
void net_owner_init(net_owner_t *o);
/* Radio initialized (hal_wifi_init succeeded); until then the owner stays offline. */
void net_owner_set_radio_ready(net_owner_t *o, bool ready, uint32_t init_ms);
void net_owner_step(net_owner_t *o);
void net_owner_record_loop(net_owner_t *o, uint32_t duration_ms);
