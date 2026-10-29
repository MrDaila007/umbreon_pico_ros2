#include "umb_ros_command.h"
#include <float.h>
#include <math.h>
bool umb_ros_command(umb_ros_command_topic_t topic, const umb_twist_fields_t *t,
                       uint32_t now, umb_cmd_t *out) {
    if (!t || !out || (unsigned)topic > UMB_ROS_CMD_RAW) return false;
    for (unsigned i = 0; i < 3; i++)
        if (!isfinite(t->linear[i]) || !isfinite(t->angular[i])) return false;
    if (t->linear[1] != 0 || t->linear[2] != 0 || t->angular[0] != 0 || t->angular[1] != 0) return false;
    double steer = t->angular[2], value = t->linear[0];
    if (steer < -1) steer = -1;
    if (steer > 1) steer = 1;
    if (value < -FLT_MAX) value = -FLT_MAX;
    if (value > FLT_MAX) value = FLT_MAX;
    *out = (umb_cmd_t){.kind=topic == UMB_ROS_CMD_RAW ? UMB_CMD_RAW : UMB_CMD_SPEED,
        .source=UMB_SRC_ROS,.mode=topic == UMB_ROS_CMD_VEL ? UMB_DRIVE_AUTO : UMB_DRIVE_MANUAL,
        .steer=(float)steer,.value=(float)value,.rx_ms=now};
    return true;
}
