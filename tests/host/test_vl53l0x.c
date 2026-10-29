#include "test_runner.h"
#include "fake_i2c.h"
#include "fake_vl53l0x.h"
#include "vl53l0x.h"

#define ADDR VL53L0X_DEFAULT_ADDR

static fake_i2c_t f;
static fake_vl53l0x_t dev;

static umb_i2c_t setup(void) {
    fake_i2c_reset(&f);
    dev = (fake_vl53l0x_t){ .spad_ready = true, .calib_ready = true };
    fake_vl53l0x_install(&f, ADDR, &dev);
    return fake_i2c_bus(&f);
}

static bool logged(uint8_t addr, uint8_t reg, uint8_t val) {
    for (int i = 0; i < f.write_log_len; i++)
        if (f.write_log[i].addr == addr && f.write_log[i].reg == reg && f.write_log[i].val == val)
            return true;
    return false;
}

static void clear_log(void) { f.write_log_len = 0; f.writes = 0; }

TEST(test_vl53l0x_init_ok_and_model_check) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    ASSERT_TRUE(d.ok);
    ASSERT_EQ(d.addr, ADDR);
    ASSERT_EQ(d.stop_variable, FAKE_VL53L0X_STOP_VARIABLE);
    /* budget recomputed from the tuned registers (seq 0xF8), then re-applied */
    ASSERT_EQ(d.timing_budget_us, 28722);
    const uint8_t *r = f.regs[ADDR];
    ASSERT_EQ(r[0x89] & 0x01, 0x01);          /* 2V8 I/O */
    ASSERT_EQ(r[0x01], 0xE8);                 /* sequence restored after calibration */
    ASSERT_EQ(r[0x0A], 0x04);                 /* interrupt on new sample ready */
    ASSERT_EQ(r[0x84], 0x01);                 /* GPIO active low */
    ASSERT_EQ(r[0x13] & 0x07, 0);             /* interrupt cleared */
    /* 5 aperture SPADs enabled starting at SPAD 12 */
    ASSERT_EQ(r[0xB0], 0x00);
    ASSERT_EQ(r[0xB1], 0xF0);
    ASSERT_EQ(r[0xB2], 0x01);
    ASSERT_EQ(r[0xB3], 0x00);
    ASSERT_EQ(r[0xB5], 0x00);
    ASSERT_EQ(r[0xB6], 0xB4);
}

TEST(test_vl53l0x_wrong_model_fails) {
    umb_i2c_t b = setup();
    f.regs[ADDR][0xC0] = 0x00;
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), VL53L0X_ERR_MODEL);
    ASSERT_FALSE(d.ok);
    ASSERT_EQ(f.writes, 0);
}

TEST(test_vl53l0x_set_address_writes_8a) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    ASSERT_EQ(vl53l0x_set_address(&d, &b, 0x30), 0);
    ASSERT_EQ(f.regs[0x30][0x8A], 0x30 & 0x7F);
    ASSERT_EQ(d.addr, 0x30);
    ASSERT_FALSE(f.present[ADDR]);
    f.regs[0x30][0x13] = 0x07;
    uint16_t mm = 0;
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, NULL), 1);
    ASSERT_EQ(f.regs[0x30][0x13] & 0x07, 0);
}

TEST(test_vl53l0x_set_address_masks_to_7_bits) {
    umb_i2c_t b = setup();
    vl53l0x_t d = { .addr = ADDR };
    ASSERT_EQ(vl53l0x_set_address(&d, &b, 0xB1), 0);
    ASSERT_EQ(d.addr, 0x31);
    ASSERT_TRUE(f.present[0x31]);
}

TEST(test_vl53l0x_poll_not_ready_returns_0) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    clear_log();
    uint32_t delay_before = f.delay_total_us;
    uint16_t mm = 77;
    uint8_t st = 77;
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, &st), 0);
    ASSERT_EQ(f.writes, 0);
    ASSERT_EQ(f.delay_total_us, delay_before);
}

TEST(test_vl53l0x_poll_reads_range_and_clears) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    f.regs[ADDR][0x13] = 0x07;
    f.regs[ADDR][0x14] = (11 << 3) | 0x06;   /* range status 11 */
    f.regs[ADDR][0x14 + 10] = 523 >> 8;
    f.regs[ADDR][0x14 + 11] = 523 & 0xFF;
    clear_log();
    uint32_t delay_before = f.delay_total_us;
    uint16_t mm = 0;
    uint8_t st = 0;
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, &st), 1);
    ASSERT_EQ(mm, 523);
    ASSERT_EQ(st, 11);
    ASSERT_TRUE(logged(ADDR, 0x0B, 0x01));
    ASSERT_EQ(f.regs[ADDR][0x13] & 0x07, 0);
    ASSERT_EQ(f.delay_total_us, delay_before);
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, &st), 0);
}

TEST(test_vl53l0x_init_timeout_bounded) {
    umb_i2c_t b = setup();
    dev.calib_ready = false;
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), VL53L0X_ERR_TIMEOUT);
    ASSERT_FALSE(d.ok);
    ASSERT_TRUE(f.delay_total_us >= 100000);
    ASSERT_TRUE(f.delay_total_us <= 110000);
}

TEST(test_vl53l0x_spad_timeout_bounded) {
    umb_i2c_t b = setup();
    dev.spad_ready = false;
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), VL53L0X_ERR_TIMEOUT);
    ASSERT_TRUE(f.delay_total_us <= 110000);
}

TEST(test_vl53l0x_nack_returns_error) {
    umb_i2c_t b = setup();
    f.present[ADDR] = false;
    vl53l0x_t d = { .addr = ADDR };
    uint16_t mm;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), VL53L0X_ERR_BUS);
    ASSERT_FALSE(d.ok);
    ASSERT_EQ(vl53l0x_set_address(&d, &b, 0x30), VL53L0X_ERR_BUS);
    ASSERT_EQ(d.addr, ADDR);
    ASSERT_EQ(vl53l0x_set_timing_budget(&d, &b, 20000), VL53L0X_ERR_BUS);
    ASSERT_EQ(vl53l0x_start_continuous(&d, &b), VL53L0X_ERR_BUS);
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, NULL), VL53L0X_ERR_BUS);
}

/* A NACK at any point of the init sequence is reported, never ignored. */
TEST(test_vl53l0x_nack_anywhere_in_init) {
    int n;
    for (n = 0; n < 2000; n++) {
        umb_i2c_t b = setup();
        f.nack_writes_after = n;
        vl53l0x_t d;
        int rc = vl53l0x_init(&d, &b, ADDR);
        if (rc == 0) break;
        ASSERT_EQ(rc, VL53L0X_ERR_BUS);
        ASSERT_FALSE(d.ok);
    }
    ASSERT_TRUE(n > 100 && n < 2000);
}

TEST(test_vl53l0x_nack_during_poll_clear) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    f.regs[ADDR][0x13] = 0x07;
    uint16_t mm;
    for (int n = 0; n < 3; n++) {   /* 0x13 ptr, 0x14 ptr, 0x0B clear */
        f.nack_writes_after = f.write_calls + n;
        ASSERT_EQ(vl53l0x_poll_range(&d, &b, &mm, NULL), VL53L0X_ERR_BUS);
    }
}

TEST(test_vl53l0x_timing_budget_20ms_accepted) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    ASSERT_EQ(vl53l0x_set_timing_budget(&d, &b, VL53L0X_HIGH_SPEED_BUDGET_US), 0);
    ASSERT_EQ(d.timing_budget_us, 20000);
    /* final range = 20000 - 14161 us = 153 mclks (10 pclk) + 151 pre-range = 304 */
    ASSERT_EQ(f.regs[ADDR][0x71], 0x01);
    ASSERT_EQ(f.regs[ADDR][0x72], 0x97);
}

TEST(test_vl53l0x_budget_too_small_rejected) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    uint32_t before = d.timing_budget_us;
    clear_log();
    ASSERT_EQ(vl53l0x_set_timing_budget(&d, &b, 19999), VL53L0X_ERR_PARAM);
    ASSERT_EQ(d.timing_budget_us, before);
    ASSERT_EQ(f.writes, 0);
}

TEST(test_vl53l0x_budget_below_step_overheads_rejected) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    uint32_t before = d.timing_budget_us;
    f.regs[ADDR][0x51] = 0x05;   /* pre-range timeout 0x96 << 5 mclks, ~258 ms */
    ASSERT_EQ(vl53l0x_set_timing_budget(&d, &b, 20000), VL53L0X_ERR_PARAM);
    ASSERT_EQ(d.timing_budget_us, before);
}

TEST(test_vl53l0x_start_continuous_back_to_back) {
    umb_i2c_t b = setup();
    vl53l0x_t d;
    ASSERT_EQ(vl53l0x_init(&d, &b, ADDR), 0);
    clear_log();
    ASSERT_EQ(vl53l0x_start_continuous(&d, &b), 0);
    ASSERT_TRUE(logged(ADDR, 0x91, FAKE_VL53L0X_STOP_VARIABLE));
    ASSERT_EQ(f.regs[ADDR][0x00], 0x02);      /* SYSRANGE_MODE_BACKTOBACK */
    ASSERT_EQ(f.regs[ADDR][0xFF], 0x00);
    ASSERT_EQ(f.regs[ADDR][0x80], 0x00);
}

TEST(test_vl53l0x_null_args) {
    umb_i2c_t b = setup();
    vl53l0x_t d = { .addr = ADDR };
    uint16_t mm;
    ASSERT_EQ(vl53l0x_init(NULL, &b, ADDR), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_init(&d, NULL, ADDR), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_set_address(NULL, &b, 0x30), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_set_timing_budget(&d, NULL, 20000), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_start_continuous(NULL, &b), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_poll_range(&d, &b, NULL, NULL), VL53L0X_ERR_PARAM);
    ASSERT_EQ(vl53l0x_poll_range(&d, NULL, &mm, NULL), VL53L0X_ERR_PARAM);
}

int main(void) {
    RUN_TEST(test_vl53l0x_init_ok_and_model_check);
    RUN_TEST(test_vl53l0x_wrong_model_fails);
    RUN_TEST(test_vl53l0x_set_address_writes_8a);
    RUN_TEST(test_vl53l0x_set_address_masks_to_7_bits);
    RUN_TEST(test_vl53l0x_poll_not_ready_returns_0);
    RUN_TEST(test_vl53l0x_poll_reads_range_and_clears);
    RUN_TEST(test_vl53l0x_init_timeout_bounded);
    RUN_TEST(test_vl53l0x_spad_timeout_bounded);
    RUN_TEST(test_vl53l0x_nack_returns_error);
    RUN_TEST(test_vl53l0x_nack_anywhere_in_init);
    RUN_TEST(test_vl53l0x_nack_during_poll_clear);
    RUN_TEST(test_vl53l0x_timing_budget_20ms_accepted);
    RUN_TEST(test_vl53l0x_budget_too_small_rejected);
    RUN_TEST(test_vl53l0x_budget_below_step_overheads_rejected);
    RUN_TEST(test_vl53l0x_start_continuous_back_to_back);
    RUN_TEST(test_vl53l0x_null_args);
    TEST_SUMMARY();
}
