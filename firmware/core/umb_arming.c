/* umb_arming.c - see umb_arming.h */
#include "umb_arming.h"
#include <math.h>
#include <stddef.h>
#include "umb_gear.h"   /* UMB_DIR_DEADBAND */

static void report(umb_arm_result_t *res, umb_arm_result_t v) {
    if (res != NULL) *res = v;
}

/* Result after an accepted key press: OK once the mode's keys are all set. */
static umb_arm_result_t key_result(umb_arming_t a) {
    if (a.mode == UMB_ARM_BOTH) {
        if (!a.car_key) return UMB_ARM_WAITING_CAR;
        if (!a.ros_key) return UMB_ARM_WAITING_ROS;
    }
    return UMB_ARM_OK;
}

umb_arming_t umb_arming_init(umb_arm_mode_t mode) {
    umb_arming_t a = { mode, false, false, false };
    return a;
}

umb_arming_t umb_arming_car_key(umb_arming_t a, umb_arm_result_t *res) {
    if (a.mode != UMB_ARM_ENCODER && a.mode != UMB_ARM_BOTH) {
        report(res, UMB_ARM_REJECTED_MODE);
        return a;
    }
    a.car_key = true;
    a.neutral_seen = false;
    report(res, key_result(a));
    return a;
}

umb_arming_t umb_arming_ros(umb_arming_t a, bool arm, umb_arm_result_t *res) {
    if (!arm) {
        report(res, UMB_ARM_OK);
        return umb_arming_disarm(a);
    }
    if (a.mode != UMB_ARM_ROS && a.mode != UMB_ARM_BOTH) {
        report(res, UMB_ARM_REJECTED_MODE);
        return a;
    }
    a.ros_key = true;
    a.neutral_seen = false;
    report(res, key_result(a));
    return a;
}

umb_arming_t umb_arming_disarm(umb_arming_t a) {
    return umb_arming_init(a.mode);
}

umb_arming_t umb_arming_set_mode(umb_arming_t a, umb_arm_mode_t m) {
    (void)a;
    return umb_arming_init(m);
}

umb_arming_t umb_arming_on_command(umb_arming_t a, const umb_cmd_t *cmd) {
    if (cmd != NULL && cmd->kind != UMB_CMD_NONE && isfinite(cmd->value) &&
        fabsf(cmd->value) < UMB_DIR_DEADBAND) {
        a.neutral_seen = true;
    }
    return a;
}

bool umb_arming_is_armed(umb_arming_t a) {
    switch (a.mode) {
    case UMB_ARM_NONE:    return true;
    case UMB_ARM_ENCODER: return a.car_key;
    case UMB_ARM_ROS:     return a.ros_key;
    case UMB_ARM_BOTH:    return a.car_key && a.ros_key;
    }
    return false;
}

bool umb_arming_motion_allowed(umb_arming_t a) {
    return umb_arming_is_armed(a) && a.neutral_seen;
}
