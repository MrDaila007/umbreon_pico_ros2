#include "fake_vl53l0x.h"
#include <string.h>

#define REG_SYSRANGE_START 0x00
#define REG_INTR_CLEAR     0x0B
#define REG_INTR_STATUS    0x13
#define REG_SPAD_WAIT      0x83
#define REG_I2C_ADDR       0x8A
#define REG_PAGE           0xFF

static void on_write(fake_i2c_t *f, uint8_t addr, uint8_t reg, uint8_t val, void *user) {
    const fake_vl53l0x_t *dev = user;
    uint8_t *regs = f->regs[addr];
    uint8_t page = regs[REG_PAGE];

    if (reg == REG_SPAD_WAIT && val == 0x00 && page == 0x07 && dev->spad_ready) {
        regs[REG_SPAD_WAIT] = 0x01;
    } else if (reg == REG_SYSRANGE_START && page == 0x00 && (val & 0x01) && dev->calib_ready) {
        regs[REG_INTR_STATUS] |= 0x07;
    } else if (reg == REG_INTR_CLEAR && (val & 0x01)) {
        regs[REG_INTR_STATUS] &= (uint8_t)~0x07;
    } else if (reg == REG_I2C_ADDR) {
        uint8_t to = val & 0x7F;
        if (to != addr) {
            memcpy(f->regs[to], f->regs[addr], sizeof f->regs[addr]);
            f->ptr[to] = f->ptr[addr];
            f->present[to] = true;
            f->present[addr] = false;
        }
    }
}

void fake_vl53l0x_install(fake_i2c_t *f, uint8_t addr, fake_vl53l0x_t *dev) {
    uint8_t *regs = f->regs[addr];
    f->present[addr] = true;
    regs[0xC0] = 0xEE;                         /* IDENTIFICATION_MODEL_ID */
    regs[0x91] = FAKE_VL53L0X_STOP_VARIABLE;
    regs[0x92] = FAKE_VL53L0X_SPAD_INFO;
    memset(&regs[0xB0], 0xFF, 6);              /* GLOBAL_CONFIG_SPAD_ENABLES_REF_0..5 */
    regs[0x84] = 0x11;                         /* GPIO_HV_MUX_ACTIVE_HIGH */
    f->on_write = on_write;
    f->hook_user = dev;
}
