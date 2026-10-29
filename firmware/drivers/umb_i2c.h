/*
 * umb_i2c.h — Bus abstraction for chip drivers.
 * 7-bit addresses; every call returns 0 on success, negative on error/NACK/timeout.
 * The HAL implements this over pico-sdk; host tests implement it with a fake bus.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct umb_i2c {
    int (*write)(void *ctx, uint8_t addr, const uint8_t *buf, size_t len, bool nostop);
    int (*read)(void *ctx, uint8_t addr, uint8_t *buf, size_t len);
    void (*delay_us)(void *ctx, uint32_t us);
    void *ctx;
} umb_i2c_t;

/* Write one register. */
int umb_i2c_write_reg8(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val);

/* Read n >= 1 consecutive registers starting at reg (repeated start). */
int umb_i2c_read_regs(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t *out, size_t n);
