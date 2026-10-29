/* Pure Twist mapping. Timestamp is monotonic local receipt time, never ROS time. */
#pragma once
#include <stdbool.h>
#include "umb_types.h"
typedef enum { UMB_ROS_CMD_VEL, UMB_ROS_CMD_TELEOP, UMB_ROS_CMD_RAW } umb_ros_command_topic_t;
typedef struct { double linear[3], angular[3]; } umb_twist_fields_t;
/* Rejected input leaves *out untouched. All fields must be finite; unused axes
 * must be zero. Steering clamps to [-1,1]. Huge finite values saturate to the
 * float representation; actual speed/throttle limits remain in control. */
bool umb_ros_command(umb_ros_command_topic_t topic, const umb_twist_fields_t *twist,
                       uint32_t receive_ms, umb_cmd_t *out);
