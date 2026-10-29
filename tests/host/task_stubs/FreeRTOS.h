#pragma once
#define INC_FREERTOS_H
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#define portBYTE_ALIGNMENT 16u
#define configTICK_RATE_HZ 1000u
void *pvPortMalloc(size_t size);
void vPortFree(void *ptr);
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMAX_DELAY UINT32_MAX
#define configASSERT(x) assert(x)
