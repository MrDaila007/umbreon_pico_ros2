#include "test_runner.h"
#include "umb_actuation.h"
#include "umb_settings.h"
#include <math.h>

TEST(test_servo_centre) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_servo_us(0.0f, &c), 1500);
}

TEST(test_servo_full_left_right) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_servo_us(-1.0f, &c), 1233);
    ASSERT_EQ(umb_servo_us(1.0f, &c), 1766);
}

TEST(test_servo_reverse_flips) {
    umb_settings_t c = umb_settings_defaults();
    c.servo_reverse = true;
    ASSERT_EQ(umb_servo_us(1.0f, &c), 1233);
    ASSERT_EQ(umb_servo_us(-1.0f, &c), 1766);
}

TEST(test_servo_clamps) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_servo_us(5.0f, &c), 1766);
    ASSERT_EQ(umb_servo_us(-5.0f, &c), 1233);
    ASSERT_EQ(umb_servo_us(NAN, &c), 1500);
    ASSERT_EQ(umb_servo_us(INFINITY, &c), 1500);
}

TEST(test_esc_zero_is_neutral) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_esc_us_from_throttle(0.0f, &c), UMB_ESC_NEUTRAL_US);
}

TEST(test_esc_forward_range) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_esc_us_from_throttle(0.001f, &c), 1540);
    ASSERT_EQ(umb_esc_us_from_throttle(1.0f, &c), 1600);
    ASSERT_EQ(umb_esc_us_from_throttle(9.0f, &c), 1600);
}

TEST(test_esc_reverse_limited) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_esc_us_from_throttle(-1.0f, &c), 1285);
    ASSERT_EQ(umb_esc_reverse_limit_us(&c), 1285);
    ASSERT_EQ(umb_esc_us_from_throttle(-0.001f, &c), 1460);
}

TEST(test_esc_max_throttle_limits_forward) {
    umb_settings_t c = umb_settings_defaults();
    c.max_throttle = 0.5f;
    ASSERT_EQ(umb_esc_us_from_throttle(1.0f, &c), 1569);
}

TEST(test_esc_nan_inf_neutral) {
    umb_settings_t c = umb_settings_defaults();
    ASSERT_EQ(umb_esc_us_from_throttle(NAN, &c), 1500);
    ASSERT_EQ(umb_esc_us_from_throttle(INFINITY, &c), 1500);
    ASSERT_EQ(umb_esc_us_from_throttle(-INFINITY, &c), 1500);
}

TEST(test_esc_output_clamped_to_1000_2000) {
    umb_settings_t c = umb_settings_defaults();
    c.max_speed = 2500;
    c.min_bspeed = 100;
    ASSERT_EQ(umb_esc_us_from_throttle(1.0f, &c), 2000);
    c.max_throttle_rev = 1.0f;
    ASSERT_EQ(umb_esc_us_from_throttle(-1.0f, &c), 1000);
}

int main(void) {
    RUN_TEST(test_servo_centre);
    RUN_TEST(test_servo_full_left_right);
    RUN_TEST(test_servo_reverse_flips);
    RUN_TEST(test_servo_clamps);
    RUN_TEST(test_esc_zero_is_neutral);
    RUN_TEST(test_esc_forward_range);
    RUN_TEST(test_esc_reverse_limited);
    RUN_TEST(test_esc_max_throttle_limits_forward);
    RUN_TEST(test_esc_nan_inf_neutral);
    RUN_TEST(test_esc_output_clamped_to_1000_2000);
    TEST_SUMMARY();
}
