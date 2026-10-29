#include "umb_i2c.h"

#define UMB_I2C_ERR (-1)

int umb_i2c_write_reg8(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val) {
    if (!b || !b->write) return UMB_I2C_ERR;
    const uint8_t buf[2] = { reg, val };
    return b->write(b->ctx, addr, buf, sizeof buf, false);
}

int umb_i2c_read_regs(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t *out, size_t n) {
    if (!b || !b->write || !b->read || !out || n == 0) return UMB_I2C_ERR;
    int rc = b->write(b->ctx, addr, &reg, 1, true);
    if (rc < 0) return rc;
    return b->read(b->ctx, addr, out, n);
}
