/* umb_safety.h - always-on link protections as a stop bitmask (pure C11).
 * The control step acts on the mask. Battery is a reflex, not evaluated here. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"
#include "umb_settings.h"

enum {
    UMB_STOP_CMD_TIMEOUT = 1u << 0,
    UMB_STOP_AGENT       = 1u << 1,
    UMB_STOP_WIFI        = 1u << 2,
    UMB_STOP_REFLEX      = 1u << 3, /* set by umb_control from umb_reflex, never here */
    UMB_STOP_CORE0       = 1u << 4,
    UMB_STOP_MAINTENANCE = 1u << 6, /* shared maintenance reservation */
    UMB_STOP_NO_CMD      = 1u << 5
};

/* Conditions that also disarm (a command timeout only stops). */
#define UMB_STOP_DISARM_MASK (UMB_STOP_AGENT | UMB_STOP_WIFI | UMB_STOP_CORE0)
#define UMB_CORE0_STALE_MS 500u

typedef struct {
    uint32_t now_ms;
    bool have_cmd;
    uint32_t last_cmd_ms;
    umb_cmd_source_t source;
    bool wifi_up;
    bool agent_up;
    uint32_t core0_beat_ms;
} umb_safety_in_t;

uint32_t umb_safety_evaluate(const umb_safety_in_t *in, const umb_settings_t *c);
