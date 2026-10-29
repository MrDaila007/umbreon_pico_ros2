#include "mpu6050.h"

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define PWR_PLL_GYRO_X   0x01
#define SMPLRT_100HZ     9      /* 1 kHz / (1 + 9) */
#define DLPF_42HZ        0x03
#define GYRO_FS_500DPS   0x08
#define ACCEL_FS_8G      0x10

#define ACCEL_LSB_PER_G   4096.0f
#define GYRO_LSB_PER_DPS  65.5f
#define TEMP_LSB_PER_C    340.0f
#define TEMP_OFFSET_C     36.53f

#define FRAME_LEN        14
#define MPU_ERR_BUS      (-1)
#define MPU_ERR_FRAME    (-2)

static bool whoami_ok(uint8_t id) {
    return id == 0x68 || id == 0x70 || id == 0x72 || id == 0x98;
}

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

int mpu6050_init(const umb_i2c_t *b, uint8_t addr) {
    uint8_t id = 0;
    int rc = umb_i2c_read_regs(b, addr, REG_WHO_AM_I, &id, 1);
    if (rc < 0) return rc;
    if (!whoami_ok(id)) return MPU_ERR_BUS;

    static const struct { uint8_t reg, val; } cfg[] = {
        { REG_PWR_MGMT_1,   PWR_PLL_GYRO_X },
        { REG_SMPLRT_DIV,   SMPLRT_100HZ },
        { REG_CONFIG,       DLPF_42HZ },
        { REG_GYRO_CONFIG,  GYRO_FS_500DPS },
        { REG_ACCEL_CONFIG, ACCEL_FS_8G },
    };
    for (size_t i = 0; i < sizeof cfg / sizeof cfg[0]; i++) {
        rc = umb_i2c_write_reg8(b, addr, cfg[i].reg, cfg[i].val);
        if (rc < 0) return rc;
    }
    return 0;
}

int mpu6050_read(const umb_i2c_t *b, uint8_t addr, mpu6050_sample_t *out) {
    if (!out) return MPU_ERR_BUS;
    uint8_t f[FRAME_LEN];
    int rc = umb_i2c_read_regs(b, addr, REG_ACCEL_XOUT_H, f, sizeof f);
    if (rc < 0) return rc;

    bool all0 = true, allff = true;
    for (size_t i = 0; i < sizeof f; i++) {
        if (f[i] != 0x00) all0 = false;
        if (f[i] != 0xFF) allff = false;
    }
    if (all0 || allff) return MPU_ERR_FRAME;

    mpu6050_sample_t s;
    for (int i = 0; i < 3; i++) {
        s.accel_g[i]  = (float)be16(&f[2 * i]) / ACCEL_LSB_PER_G;
        s.gyro_dps[i] = (float)be16(&f[8 + 2 * i]) / GYRO_LSB_PER_DPS;
    }
    s.temp_c = (float)be16(&f[6]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    *out = s;
    return 0;
}
