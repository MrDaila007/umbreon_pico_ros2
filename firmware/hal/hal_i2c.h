/* hal_i2c.h - the two I2C buses (pico-sdk i2c0/i2c1) behind umb_i2c_t, one FreeRTOS mutex per bus.
 * I2C0 GP0/GP1 @ 400 kHz, I2C1 GP2/GP3 @ 100 kHz, internal pull-ups on. 5 ms timeout per transfer.
 * Each umb_i2c_t write/read takes and gives the bus mutex around its single transfer. A register
 * read is write(nostop)+read, so callers (the sensor/IMU tasks) MUST wrap every driver call in
 * hal_i2c_lock()/hal_i2c_unlock() to keep the pair atomic against other users of the bus
 * (the mutex is recursive, so nesting with the per-transfer locking is fine). */
#pragma once

#include <stdbool.h>
#include "umb_i2c.h"

typedef enum { HAL_I2C0 = 0, HAL_I2C1 = 1 } hal_i2c_bus_t;

/* Call once before vTaskStartScheduler (creates the mutexes). */
void hal_i2c_init(void);
const umb_i2c_t *hal_i2c_bus(hal_i2c_bus_t b);
/* Returns false if the bus is invalid or the mutex could not be taken. */
bool hal_i2c_lock(hal_i2c_bus_t b);
void hal_i2c_unlock(hal_i2c_bus_t b);
