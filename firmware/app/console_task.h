/* console_task.h - line console on USB CDC and UART0 (GP16/17, 115200), pinned to core 0.
 * Network task owns the core-0 heartbeat; console responsiveness cannot mask it. */
#pragma once
#include "FreeRTOS.h"

#define CONSOLE_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define CONSOLE_TASK_STACK_WORDS 2048u
#define CONSOLE_TASK_CORE_MASK (1u << 0)

/* param: reset-reason string literal, shown by `status`. */
void console_task(void *param);
