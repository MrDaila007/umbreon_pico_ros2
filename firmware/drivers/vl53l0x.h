/*
 * vl53l0x.h — Register-level VL53L0X time-of-flight driver over umb_i2c_t.
 * C port of Pololu's vl53l0x-arduino (itself based on ST's VL53L0X API);
 * see vl53l0x.c and LICENSE-vl53l0x.txt.
 *
 * Return codes: 0 ok, VL53L0X_ERR_BUS (-1) I2C error/NACK, VL53L0X_ERR_MODEL (-2)
 * model ID mismatch, VL53L0X_ERR_TIMEOUT (-3) a bounded wait (100 ms) expired,
 * VL53L0X_ERR_PARAM (-4) bad argument or timing budget out of range.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "umb_i2c.h"

#define VL53L0X_DEFAULT_ADDR 0x29
#define VL53L0X_MAX_VALID_MM 8190      /* >= this = out of range */

#define VL53L0X_MIN_BUDGET_US        20000u  /* Pololu/ST minimum */
#define VL53L0X_HIGH_SPEED_BUDGET_US 20000u  /* project "high speed" profile */

#define VL53L0X_ERR_BUS     (-1)
#define VL53L0X_ERR_MODEL   (-2)
#define VL53L0X_ERR_TIMEOUT (-3)
#define VL53L0X_ERR_PARAM   (-4)

typedef struct {
    uint8_t addr;
    uint8_t stop_variable;
    uint32_t timing_budget_us;
    bool ok;
} vl53l0x_t;

/* Full init (data init, static init, ref calibration) of the sensor at addr. */
int vl53l0x_init(vl53l0x_t *d, const umb_i2c_t *b, uint8_t addr);

/* Move the sensor to new_addr (reg 0x8A); later calls use the new address. */
int vl53l0x_set_address(vl53l0x_t *d, const umb_i2c_t *b, uint8_t new_addr);

/* Set the measurement timing budget; us < VL53L0X_MIN_BUDGET_US is rejected. */
int vl53l0x_set_timing_budget(vl53l0x_t *d, const umb_i2c_t *b, uint32_t us);

/* Start continuous back-to-back ranging. */
int vl53l0x_start_continuous(vl53l0x_t *d, const umb_i2c_t *b);

/* Non-blocking: 1 = new reading (interrupt cleared), 0 = not ready, < 0 = error.
 * status (optional) receives the range status, (reg 0x14 >> 3) & 0x0F. */
int vl53l0x_poll_range(vl53l0x_t *d, const umb_i2c_t *b, uint16_t *mm, uint8_t *status);
