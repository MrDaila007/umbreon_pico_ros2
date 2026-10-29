/*
 * fake_vl53l0x.h — Device behaviour for a VL53L0X on the fake I2C bus.
 *
 * Installs an on_write hook on the fake bus that emulates just enough of the
 * sensor for the driver's init and ranging sequence to complete:
 *   - writing 0x83 = 0x00 on page 7 makes 0x83 read non-zero (SPAD info ready)
 *     when spad_ready is set;
 *   - writing SYSRANGE_START (0x00) with bit 0 set on page 0 raises
 *     RESULT_INTERRUPT_STATUS (0x13) when calib_ready is set;
 *   - writing SYSTEM_INTERRUPT_CLEAR (0x0B) clears 0x13;
 *   - writing I2C_SLAVE_DEVICE_ADDRESS (0x8A) moves the device to the new address.
 * The hook works by address, so one state can drive several devices.
 */
#pragma once

#include "fake_i2c.h"

#define FAKE_VL53L0X_STOP_VARIABLE 0x3C
#define FAKE_VL53L0X_SPAD_INFO     0x85   /* count 5, aperture type */

typedef struct {
    bool spad_ready;
    bool calib_ready;
} fake_vl53l0x_t;

/* Preload power-on registers for a device at addr and install the hook. */
void fake_vl53l0x_install(fake_i2c_t *f, uint8_t addr, fake_vl53l0x_t *dev);
