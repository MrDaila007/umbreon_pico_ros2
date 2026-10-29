/* Staged network owner. Task6 wires network policy; this task owns platform initialization. */
#pragma once
#include "FreeRTOS.h"
#define NET_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define NET_TASK_STACK_WORDS 8192u
#define NET_TASK_CORE_MASK (1u << 0)
void net_task(void *param);
