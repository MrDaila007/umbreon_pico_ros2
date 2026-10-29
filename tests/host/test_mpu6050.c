#include "test_runner.h"
#include "fake_i2c.h"
#include "mpu6050.h"
#include <string.h>

#define ADDR 0x68

static fake_i2c_t f;

static umb_i2c_t setup(uint8_t whoami) {
    fake_i2c_reset(&f);
    f.present[ADDR] = true;
    f.regs[ADDR][0x75] = whoami;
    return fake_i2c_bus(&f);
}

static void put16(int reg, int16_t v) {
    f.regs[ADDR][reg] = (uint8_t)((uint16_t)v >> 8);
    f.regs[ADDR][reg + 1] = (uint8_t)((uint16_t)v & 0xFF);
}

static void fill_frame(uint8_t v) { memset(&f.regs[ADDR][0x3B], v, 14); }

TEST(test_mpu6050_init_writes_config) {
    umb_i2c_t b = setup(0x68);
    ASSERT_EQ(mpu6050_init(&b, ADDR), 0);
    ASSERT_EQ(f.regs[ADDR][0x6B], 0x01);
    ASSERT_EQ(f.regs[ADDR][0x19], 9);
    ASSERT_EQ(f.regs[ADDR][0x1A], 3);
    ASSERT_EQ(f.regs[ADDR][0x1B], 0x08);
    ASSERT_EQ(f.regs[ADDR][0x1C], 0x10);
}

TEST(test_mpu6050_clone_whoami_accepted) {
    const uint8_t ids[] = {0x68, 0x70, 0x72, 0x98};
    for (size_t i = 0; i < sizeof ids; i++) {
        umb_i2c_t b = setup(ids[i]);
        ASSERT_EQ(mpu6050_init(&b, ADDR), 0);
    }
}

TEST(test_mpu6050_bad_whoami_not_ok) {
    umb_i2c_t b = setup(0x00);
    ASSERT_TRUE(mpu6050_init(&b, ADDR) < 0);
    ASSERT_EQ(f.regs[ADDR][0x6B], 0x00);
}

TEST(test_mpu6050_absent_nack) {
    umb_i2c_t b = setup(0x68);
    f.present[ADDR] = false;
    ASSERT_TRUE(mpu6050_init(&b, ADDR) < 0);
    mpu6050_sample_t s;
    ASSERT_EQ(mpu6050_read(&b, ADDR, &s), -1);
}

TEST(test_mpu6050_config_write_failure) {
    umb_i2c_t b = setup(0x68);
    f.nack_writes_after = 2;  /* WHO_AM_I pointer write + first config write ok */
    ASSERT_TRUE(mpu6050_init(&b, ADDR) < 0);
}

TEST(test_mpu6050_scaling) {
    umb_i2c_t b = setup(0x68);
    fill_frame(0x00);
    put16(0x3B, 8192);   /* ax = 2 g */
    put16(0x3D, -4096);  /* ay = -1 g */
    put16(0x3F, 4096);   /* az = 1 g */
    put16(0x41, 0);      /* temp */
    put16(0x43, 655);    /* gx = 10 dps */
    put16(0x45, 65);
    put16(0x47, -131);   /* gz = -2 dps */
    mpu6050_sample_t s;
    ASSERT_EQ(mpu6050_read(&b, ADDR, &s), 0);
    ASSERT_FLOAT_EQ(s.accel_g[0], 2.0f, 0.01f);
    ASSERT_FLOAT_EQ(s.accel_g[1], -1.0f, 0.01f);
    ASSERT_FLOAT_EQ(s.accel_g[2], 1.0f, 0.01f);
    ASSERT_FLOAT_EQ(s.temp_c, 36.53f, 0.01f);
    ASSERT_FLOAT_EQ(s.gyro_dps[0], 10.0f, 0.01f);
    ASSERT_FLOAT_EQ(s.gyro_dps[1], 65.0f / 65.5f, 0.01f);
    ASSERT_FLOAT_EQ(s.gyro_dps[2], -2.0f, 0.01f);
}

TEST(test_mpu6050_temperature_scaling) {
    umb_i2c_t b = setup(0x68);
    fill_frame(0x00);
    put16(0x3F, 4096);
    put16(0x41, 3400);
    mpu6050_sample_t s;
    ASSERT_EQ(mpu6050_read(&b, ADDR, &s), 0);
    ASSERT_FLOAT_EQ(s.temp_c, 46.53f, 0.01f);
}

TEST(test_mpu6050_all_ff_frame_error) {
    umb_i2c_t b = setup(0x68);
    fill_frame(0xFF);
    mpu6050_sample_t s;
    ASSERT_EQ(mpu6050_read(&b, ADDR, &s), -2);
}

TEST(test_mpu6050_all_zero_frame_error) {
    umb_i2c_t b = setup(0x68);
    fill_frame(0x00);
    mpu6050_sample_t s;
    ASSERT_EQ(mpu6050_read(&b, ADDR, &s), -2);
}

TEST(test_mpu6050_null_args) {
    umb_i2c_t b = setup(0x68);
    ASSERT_TRUE(mpu6050_init(NULL, ADDR) < 0);
    ASSERT_TRUE(mpu6050_read(&b, ADDR, NULL) < 0);
    ASSERT_TRUE(mpu6050_read(NULL, ADDR, NULL) < 0);
}

/* fake bus behaviour that Task 4 relies on */
static uint8_t hook_read(fake_i2c_t *fk, uint8_t addr, uint8_t reg, void *user) {
    (void)fk; (void)addr;
    (*(int *)user)++;
    return (uint8_t)(reg ^ 0x5A);
}
static void hook_write(fake_i2c_t *fk, uint8_t addr, uint8_t reg, uint8_t val, void *user) {
    (void)fk; (void)addr; (void)reg;
    *(int *)user += val;
}

TEST(test_fake_i2c_auto_increment_and_pointer) {
    umb_i2c_t b = setup(0x68);
    const uint8_t w[] = {0x10, 1, 2, 3};
    ASSERT_EQ(b.write(b.ctx, ADDR, w, sizeof w, false), 0);
    ASSERT_EQ(f.regs[ADDR][0x11], 2);
    uint8_t out[3];
    ASSERT_EQ(umb_i2c_read_regs(&b, ADDR, 0x10, out, 3), 0);
    ASSERT_EQ(out[0], 1); ASSERT_EQ(out[2], 3);
    ASSERT_EQ(f.writes, 3);
    ASSERT_TRUE(f.write_log_len >= 1);
    ASSERT_EQ(f.write_log[0].reg, 0x10);
}

TEST(test_fake_i2c_hooks_delay_nack_all) {
    umb_i2c_t b = setup(0x68);
    int reads = 0, wsum = 0;
    f.on_read = hook_read; f.on_write = hook_write; f.hook_user = &reads;
    uint8_t o[2];
    ASSERT_EQ(umb_i2c_read_regs(&b, ADDR, 0x20, o, 2), 0);
    ASSERT_EQ(o[0], 0x20 ^ 0x5A);
    ASSERT_EQ(o[1], 0x21 ^ 0x5A);
    ASSERT_EQ(reads, 2);
    f.hook_user = &wsum;
    ASSERT_EQ(umb_i2c_write_reg8(&b, ADDR, 0x30, 7), 0);
    ASSERT_EQ(wsum, 7);
    b.delay_us(b.ctx, 100); b.delay_us(b.ctx, 250);
    ASSERT_EQ(f.delay_total_us, 350);
    f.nack_all = true;
    ASSERT_EQ(umb_i2c_write_reg8(&b, ADDR, 0x30, 7), -1);
    ASSERT_EQ(umb_i2c_read_regs(&b, ADDR, 0x20, o, 2), -1);
}

TEST(test_i2c_helpers_reject_bad_args) {
    umb_i2c_t b = setup(0x68);
    uint8_t o[1];
    ASSERT_TRUE(umb_i2c_write_reg8(NULL, ADDR, 0, 0) < 0);
    ASSERT_TRUE(umb_i2c_read_regs(NULL, ADDR, 0, o, 1) < 0);
    ASSERT_TRUE(umb_i2c_read_regs(&b, ADDR, 0, NULL, 1) < 0);
    ASSERT_TRUE(umb_i2c_read_regs(&b, ADDR, 0, o, 0) < 0);
}

int main(void) {
    RUN_TEST(test_mpu6050_init_writes_config);
    RUN_TEST(test_mpu6050_clone_whoami_accepted);
    RUN_TEST(test_mpu6050_bad_whoami_not_ok);
    RUN_TEST(test_mpu6050_absent_nack);
    RUN_TEST(test_mpu6050_config_write_failure);
    RUN_TEST(test_mpu6050_scaling);
    RUN_TEST(test_mpu6050_temperature_scaling);
    RUN_TEST(test_mpu6050_all_ff_frame_error);
    RUN_TEST(test_mpu6050_all_zero_frame_error);
    RUN_TEST(test_mpu6050_null_args);
    RUN_TEST(test_fake_i2c_auto_increment_and_pointer);
    RUN_TEST(test_fake_i2c_hooks_delay_nack_all);
    RUN_TEST(test_i2c_helpers_reject_bad_args);
    TEST_SUMMARY();
}
