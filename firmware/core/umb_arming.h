/* umb_arming.h - configurable arming state machine with re-enable latch (pure C11).
 * Motion needs the mode's keys AND a neutral command received after the last
 * key press / boot / disarm. Value semantics: every call returns the new state. */
#pragma once
#include <stdbool.h>
#include "umb_types.h"

typedef struct {
    umb_arm_mode_t mode;
    bool car_key;
    bool ros_key;
    bool neutral_seen;
} umb_arming_t;

typedef enum {
    UMB_ARM_OK,
    UMB_ARM_WAITING_CAR,
    UMB_ARM_WAITING_ROS,
    UMB_ARM_REJECTED_MODE
} umb_arm_result_t;

umb_arming_t umb_arming_init(umb_arm_mode_t mode);                  /* all keys false */
/* res may be NULL. An accepted key press clears neutral_seen. */
umb_arming_t umb_arming_car_key(umb_arming_t a, umb_arm_result_t *res);
umb_arming_t umb_arming_ros(umb_arming_t a, bool arm, umb_arm_result_t *res);
umb_arming_t umb_arming_disarm(umb_arming_t a);                     /* clears keys and neutral_seen */
umb_arming_t umb_arming_set_mode(umb_arming_t a, umb_arm_mode_t m); /* disarm + new mode */
umb_arming_t umb_arming_on_command(umb_arming_t a, const umb_cmd_t *cmd); /* neutral cmd sets neutral_seen */
bool umb_arming_is_armed(umb_arming_t a);        /* keys satisfy mode; NONE -> true */
bool umb_arming_motion_allowed(umb_arming_t a);  /* is_armed && neutral_seen */
