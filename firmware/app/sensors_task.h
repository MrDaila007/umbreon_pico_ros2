/* sensors_task.h - the core-1 sensor tasks: ToF (6 x VL53L0X on I2C1) and IMU + battery
 * (MPU-6050 on I2C0, ADC0). Both publish into one shared_sensors_t snapshot; health,
 * restart and filter decisions are core/umb_tof_array, umb_imu_filter and umb_battery. */
#pragma once
#include "FreeRTOS.h"

/* Below the control task (configMAX_PRIORITIES - 1) and the timer task (- 2), above UI. */
#define IMU_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#define TOF_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define IMU_TASK_STACK_WORDS 1024u
#define TOF_TASK_STACK_WORDS 1024u
#define SENSORS_TASK_CORE_MASK (1u << 1)

#define TOF_PERIOD_MS 10u
#define IMU_PERIOD_MS 10u
#define IMU_CAL_PERIOD_MS 5u
#define BAT_PERIOD_MS 50u

/* Once, before vTaskStartScheduler (after hal_i2c_init): creates the snapshot mutex. */
void sensors_init(void);

void tof_task(void *param);
void imu_task(void *param);
