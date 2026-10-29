#include "umb_safety.h"

/* Signed age of a stamp, clamped at 0: core 0 can stamp a command or beat a
 * moment after core 1 read its clock, which must count as fresh, not as
 * ~49 days old. A genuinely old stamp across the 32-bit wrap stays old. */
static uint32_t age_ms(uint32_t now, uint32_t then) {
    int32_t age = (int32_t)(now - then);
    if (age < 0) age = 0;
    return (uint32_t)age;
}

uint32_t umb_safety_evaluate(const umb_safety_in_t *in, const umb_settings_t *c) {
    uint32_t mask = 0u;

    if (!in->have_cmd) {
        mask |= UMB_STOP_NO_CMD;
    } else if (age_ms(in->now_ms, in->last_cmd_ms) >= c->cmd_timeout_ms) {
        mask |= UMB_STOP_CMD_TIMEOUT;
    }

    if (in->source == UMB_SRC_ROS) {
        if (!in->wifi_up) mask |= UMB_STOP_WIFI;
        if (!in->agent_up) mask |= UMB_STOP_AGENT;
    }

    if (age_ms(in->now_ms, in->core0_beat_ms) > UMB_CORE0_STALE_MS) {
        mask |= UMB_STOP_CORE0;
    }
    return mask;
}
