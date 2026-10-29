/* Network owner adapter; callbacks must carry their ready session generation. */
#pragma once
#include "umb_ros_command.h"
/* Subscription contract for Task9 binding: best-effort, keep-last depth 1. */
#define ROS_COMMANDS_HISTORY_DEPTH 1u
/* Generation carried by callbacks until the session is declared ready. Never
 * matches shared state, so samples cached during lifecycle setup are dropped. */
#define ROS_COMMANDS_SETUP_GENERATION 0u
bool ros_commands_post(umb_ros_command_topic_t topic, const umb_twist_fields_t *twist,
                        uint32_t generation);
