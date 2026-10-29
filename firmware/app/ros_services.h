/* ros_services.h - /umbreon/arm (std_srvs/SetBool) handler logic for the core-0
 * network owner. Host-testable: no rcl or RTOS dependency; Task9 binds it to the
 * rcl service and supplies an hal_now_ms/vTaskDelay clock. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_arm_reply.h"

#define ROS_ARM_WAIT_MS UMB_ARM_WAIT_MS

typedef struct {
    uint32_t (*now_ms)(void *ctx);           /* local monotonic ms */
    void (*sleep_ms)(void *ctx, uint32_t ms);
    void *ctx;
} ros_arm_clock_t;
typedef struct { bool success; const char *message; } ros_arm_reply_t; /* static string */

/* generation: the ready session the service belongs to (0 = none/setup).
 * Blocks at most ROS_ARM_WAIT_MS polling the actual control ACK. */
ros_arm_reply_t ros_services_arm(bool arm, uint32_t generation, const ros_arm_clock_t *clock);
