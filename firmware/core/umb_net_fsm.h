/* Bounded network policy. Inputs are observations/results; outputs request I/O. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"

#define UMB_NET_PING_PERIOD_MS 100u
#define UMB_NET_PING_TIMEOUT_MS 20u
#define UMB_NET_PING_FAIL_LIMIT 3u
/* *_done inputs are one-iteration result pulses for the outstanding action. */
enum { UMB_NET_START_STA = 1u, UMB_NET_STOP_STA = 2u,
       UMB_NET_START_AP = 4u, UMB_NET_STOP_AP = 8u };
typedef enum { UMB_WIFI_IDLE, UMB_WIFI_JOINING, UMB_WIFI_STARTING_AP,
               UMB_WIFI_ON_STA, UMB_WIFI_ON_AP, UMB_WIFI_BACKOFF } umb_wifi_stage_t;
typedef struct {
    umb_wifi_stage_t stage;
    umb_wifi_mode_t mode;
    uint32_t deadline_ms, ap_retry_ms, backoff_ms;
    bool retry_ap;
} umb_wifi_state_t;
typedef struct { umb_wifi_mode_t mode; bool credentials; uint32_t sta_timeout_ms; } umb_wifi_config_t;
typedef struct { uint32_t now_ms; bool sta_up, ap_up, error, session_busy; } umb_wifi_in_t;
typedef struct { uint8_t actions; bool loss; } umb_wifi_out_t;
umb_wifi_state_t umb_wifi_init(void);
umb_wifi_state_t umb_wifi_step(umb_wifi_state_t state, const umb_wifi_config_t *config,
                               const umb_wifi_in_t *in, umb_wifi_out_t *out);

typedef enum { UMB_SESSION_OFF, UMB_SESSION_FINDING, UMB_SESSION_PROBING,
               UMB_SESSION_CREATING, UMB_SESSION_READY, UMB_SESSION_PINGING,
               UMB_SESSION_DESTROYING, UMB_SESSION_BACKOFF } umb_session_stage_t;
typedef enum { UMB_SESSION_ACT_NONE, UMB_SESSION_ACT_FIND, UMB_SESSION_ACT_PROBE,
               UMB_SESSION_ACT_CREATE, UMB_SESSION_ACT_PING, UMB_SESSION_ACT_DESTROY } umb_session_action_t;
typedef struct {
    umb_session_stage_t stage;
    uint32_t deadline_ms, next_ping_ms, retry_ms;
    uint8_t ping_failures;
    bool have_entities;
} umb_session_t;
typedef struct {
    uint32_t now_ms;
    bool wifi_up, endpoint_valid, endpoint_changed;
    bool find_done, probe_done, probe_ok, create_done, create_ok;
    bool ping_done, ping_ok, destroy_done;
    bool transport_failed; /* repeated actual rcl/transport errors (not executor timeouts) */
} umb_session_in_t;
typedef struct { umb_session_action_t action; bool loss, ready; } umb_session_out_t;
umb_session_t umb_session_init(void);
umb_session_t umb_session_step(umb_session_t state, const umb_session_in_t *in, umb_session_out_t *out);
