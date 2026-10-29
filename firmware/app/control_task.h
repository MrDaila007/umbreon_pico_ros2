/* control_task.h - the 50 Hz control loop, pinned to core 1 at the highest priority.
 * The only caller of hal_pwm_write; feeds the hardware watchdog. */
#pragma once
#include "FreeRTOS.h"

#define CONTROL_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define CONTROL_TASK_STACK_WORDS 2048u
#define CONTROL_TASK_CORE_MASK (1u << 1)
#define CONTROL_PERIOD_MS 20u
#define CONTROL_WATCHDOG_MS 250u

void control_task(void *param);
