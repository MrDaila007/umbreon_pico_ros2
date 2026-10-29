/*
 * fake_i2c.h — Register-map fake I2C bus for host driver tests.
 *
 * Model: 256-byte register file per 7-bit address. A 1-byte write sets the
 * register pointer; an N>1-byte write sets the pointer then writes regs[ptr..]
 * with auto-increment; a read returns regs from the pointer, auto-incrementing.
 * Optional hooks override the register file per byte (on_read returns the byte;
 * on_write is called in addition to storing the byte).
 */
#pragma once

#include "umb_i2c.h"

#define FAKE_I2C_LOG_MAX 64

typedef struct fake_i2c fake_i2c_t;

typedef uint8_t (*fake_i2c_read_hook)(fake_i2c_t *f, uint8_t addr, uint8_t reg, void *user);
typedef void (*fake_i2c_write_hook)(fake_i2c_t *f, uint8_t addr, uint8_t reg, uint8_t val,
                                    void *user);

typedef struct { uint8_t addr, reg, val; } fake_i2c_write_t;

struct fake_i2c {
    uint8_t regs[128][256];
    uint8_t ptr[128];
    bool present[128];
    bool nack_all;
    int nack_writes_after;       /* -1 = never; else NACK write calls after this many succeed */
    int writes;                  /* register-data bytes written (pointer-set writes excluded) */
    int write_calls;             /* every write transaction */
    fake_i2c_write_t write_log[FAKE_I2C_LOG_MAX];
    int write_log_len;
    uint32_t delay_total_us;
    fake_i2c_read_hook on_read;
    fake_i2c_write_hook on_write;
    void *hook_user;
};

void fake_i2c_reset(fake_i2c_t *f);
umb_i2c_t fake_i2c_bus(fake_i2c_t *f);
