/* ros_save.h - /umbreon/save_settings (std_srvs/Trigger) handler for the core-0
 * network owner. Runs the same reservation + control neutral-ACK + flash path as
 * console `save` (settings_apply). success is true only after encode, write and
 * readback verified; the message carries the non-secret slot and sequence. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define ROS_SAVE_MESSAGE_MAX 64u
typedef struct { bool success; char message[ROS_SAVE_MESSAGE_MAX]; } ros_save_reply_t;

/* generation: the ready session the request belongs to (0 = none). `progress`
 * (may be NULL) runs after the neutral ACK and before flash work; the rcl binding
 * uses it for the core-0 heartbeat. Blocks at most the 60 ms neutral-ACK wait plus
 * one flash write (see docs/ros-interface.md for the measured budget). */
ros_save_reply_t ros_services_save(uint32_t generation, void (*progress)(void));
