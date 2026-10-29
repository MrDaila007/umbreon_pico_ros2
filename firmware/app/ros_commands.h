/* Network owner adapter; callbacks must carry their ready session generation. */
#pragma once
#include "umb_ros_command.h"
bool ros_commands_post(umb_ros_command_topic_t topic, const umb_twist_fields_t *twist,
                        uint32_t generation);
