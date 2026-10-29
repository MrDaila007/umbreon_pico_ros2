/* Staged network owner. Task5 replaces the heartbeat-only body with network work. */
#pragma once
#include "FreeRTOS.h"
#define NET_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define NET_TASK_STACK_WORDS 1024u
#define NET_TASK_CORE_MASK (1u << 0)
void net_task(void *param);
