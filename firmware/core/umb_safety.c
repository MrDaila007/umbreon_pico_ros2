#include "umb_safety.h"

uint32_t umb_safety_evaluate(const umb_safety_in_t *in, const umb_settings_t *c) {
    uint32_t mask = 0u;

    if (!in->have_cmd) {
        mask |= UMB_STOP_NO_CMD;
    } else if ((uint32_t)(in->now_ms - in->last_cmd_ms) >= c->cmd_timeout_ms) {
        mask |= UMB_STOP_CMD_TIMEOUT;
    }

    if (in->source == UMB_SRC_ROS) {
        if (!in->wifi_up) mask |= UMB_STOP_WIFI;
        if (!in->agent_up) mask |= UMB_STOP_AGENT;
    }

    if ((uint32_t)(in->now_ms - in->core0_beat_ms) > UMB_CORE0_STALE_MS) {
        mask |= UMB_STOP_CORE0;
    }
    return mask;
}
