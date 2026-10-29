/* ui_task.h - encoder keys and buzzer cues, pinned to core 1 at the lowest app priority.
 * Owns the encoder HAL (its 1 ms timer must run on core 1) and the buzzer PWM. */
#pragma once
#include "FreeRTOS.h"

#define UI_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define UI_TASK_STACK_WORDS 1024u
#define UI_TASK_CORE_MASK (1u << 1)
#define UI_PERIOD_MS 10u

void ui_task(void *param);
