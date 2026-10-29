#include "ros_commands.h"
#include "shared_state.h"
#include "hal_time.h"
bool ros_commands_post(umb_ros_command_topic_t topic, const umb_twist_fields_t *twist,
                        uint32_t generation) {
    umb_cmd_t command;
    if (!umb_ros_command(topic, twist, hal_now_ms(), &command)) return false;
    return shared_post_ros_cmd(&command, generation);
}
