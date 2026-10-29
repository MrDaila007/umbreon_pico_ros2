/* mpu6050.h — MPU-6050 (and common clones) IMU driver, bus-agnostic. */
#pragma once

#include "umb_i2c.h"

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temp_c;
} mpu6050_sample_t;

/* 0 ok, <0 error (-1 bus error/NACK/unknown WHO_AM_I). */
int mpu6050_init(const umb_i2c_t *b, uint8_t addr);

/* 0 ok, -1 bus error, -2 invalid frame (all 0x00 or all 0xFF). */
int mpu6050_read(const umb_i2c_t *b, uint8_t addr, mpu6050_sample_t *out);
