#pragma once
#ifndef INC_FREERTOS_H
#error "include FreeRTOS.h before task.h, as required by the actual RTOS"
#endif
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks);

#define taskSCHEDULER_RUNNING 2
int xTaskGetSchedulerState(void);
