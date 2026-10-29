#include "test_runner.h"
#include "umb_battery.h"
#include <math.h>

TEST(test_battery_first_sample_seeds) {
    umb_battery_t b = umb_battery_add(umb_battery_init(), 2048, 4.85f);
    ASSERT_TRUE(b.have);
    ASSERT_FLOAT_EQ(b.v_filtered, 8.00f, 0.02f); /* 2048*3.3/4095*4.85 = 8.004 (brief said 8.10: arithmetic slip) */
    ASSERT_FLOAT_EQ(b.v_raw, b.v_filtered, 1e-6f);
    ASSERT_FLOAT_EQ(b.v_min, b.v_filtered, 1e-6f);
}

TEST(test_battery_ema) {
    umb_battery_t b = umb_battery_add(umb_battery_init(), 2048, 4.85f);
    float seed = b.v_filtered;
    b = umb_battery_add(b, 0, 4.85f);
    ASSERT_FLOAT_EQ(b.v_raw, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(b.v_filtered, seed - UMB_BAT_EMA * seed, 1e-4f);
}

TEST(test_battery_min_tracks) {
    umb_battery_t b = umb_battery_add(umb_battery_init(), 3000, 2.0f);
    b = umb_battery_add(b, 1000, 2.0f);
    float low = b.v_min;
    b = umb_battery_add(b, 3500, 2.0f);
    ASSERT_FLOAT_EQ(b.v_min, low, 1e-6f);
    ASSERT_FLOAT_EQ(low, 1000 * 3.3f / 4095.0f * 2.0f, 1e-4f);
}

TEST(test_battery_rejects_out_of_range_adc) {
    umb_battery_t b0 = umb_battery_add(umb_battery_init(), 2048, 4.85f);
    umb_battery_t b = umb_battery_add(b0, 4096, 4.85f);
    ASSERT_FLOAT_EQ(b.v_filtered, b0.v_filtered, 1e-9f);
    ASSERT_FLOAT_EQ(b.v_raw, b0.v_raw, 1e-9f);
    umb_battery_t e = umb_battery_add(umb_battery_init(), 5000, 4.85f);
    ASSERT_FALSE(e.have);
}

TEST(test_battery_rejects_non_finite_multiplier) {
    umb_battery_t b0 = umb_battery_add(umb_battery_init(), 2048, 4.85f);
    umb_battery_t b = umb_battery_add(b0, 100, NAN);
    ASSERT_FLOAT_EQ(b.v_filtered, b0.v_filtered, 1e-9f);
    b = umb_battery_add(b0, 100, INFINITY);
    ASSERT_FLOAT_EQ(b.v_raw, b0.v_raw, 1e-9f);
    ASSERT_FALSE(umb_battery_add(umb_battery_init(), 100, NAN).have);
}

int main(void) {
    RUN_TEST(test_battery_first_sample_seeds);
    RUN_TEST(test_battery_ema);
    RUN_TEST(test_battery_min_tracks);
    RUN_TEST(test_battery_rejects_out_of_range_adc);
    RUN_TEST(test_battery_rejects_non_finite_multiplier);
    TEST_SUMMARY();
}
