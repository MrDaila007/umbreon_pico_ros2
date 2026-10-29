#include "fake_i2c.h"
#include <string.h>

void fake_i2c_reset(fake_i2c_t *f) {
    memset(f, 0, sizeof *f);
    f->nack_writes_after = -1;
}

static bool reachable(const fake_i2c_t *f, uint8_t addr) {
    return !f->nack_all && addr < 128 && f->present[addr];
}

static int fake_write(void *ctx, uint8_t addr, const uint8_t *buf, size_t len, bool nostop) {
    (void)nostop;
    fake_i2c_t *f = ctx;
    if (!reachable(f, addr)) return -1;
    if (f->nack_writes_after >= 0 && f->write_calls >= f->nack_writes_after) return -1;
    f->write_calls++;
    if (len == 0) return 0;
    f->ptr[addr] = buf[0];
    for (size_t i = 1; i < len; i++) {
        uint8_t reg = f->ptr[addr]++;
        f->regs[addr][reg] = buf[i];
        f->writes++;
        if (f->write_log_len < FAKE_I2C_LOG_MAX)
            f->write_log[f->write_log_len++] = (fake_i2c_write_t){ addr, reg, buf[i] };
        if (f->on_write) f->on_write(f, addr, reg, buf[i], f->hook_user);
    }
    return 0;
}

static int fake_read(void *ctx, uint8_t addr, uint8_t *buf, size_t len) {
    fake_i2c_t *f = ctx;
    if (!reachable(f, addr)) return -1;
    for (size_t i = 0; i < len; i++) {
        uint8_t reg = f->ptr[addr]++;
        buf[i] = f->on_read ? f->on_read(f, addr, reg, f->hook_user) : f->regs[addr][reg];
    }
    return 0;
}

static void fake_delay(void *ctx, uint32_t us) {
    ((fake_i2c_t *)ctx)->delay_total_us += us;
}

umb_i2c_t fake_i2c_bus(fake_i2c_t *f) {
    return (umb_i2c_t){ fake_write, fake_read, fake_delay, f };
}
