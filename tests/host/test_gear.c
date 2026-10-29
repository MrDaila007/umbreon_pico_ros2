#include "test_runner.h"
#include "umb_gear.h"
#include <stdint.h>
#include <math.h>

static umb_settings_t C;

static umb_gear_t reach_reverse(uint32_t t0) {
    umb_gear_t g = umb_gear_init();
    g = umb_gear_step(g, -0.5f, 0.5f, t0, &C);
    g = umb_gear_step(g, -0.5f, 0.05f, t0 + 10, &C);
    g = umb_gear_step(g, -0.5f, 0.0f, t0 + 10 + C.reverse_brake_ms, &C);
    g = umb_gear_step(g, -0.5f, 0.0f, t0 + 10 + C.reverse_brake_ms + UMB_GEAR_GAP_MS, &C);
    return g;
}

TEST(test_gear_forward_immediate) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), 0.5f, 0.0f, 0, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_FORWARD);
    ASSERT_EQ(umb_gear_drive_dir(g), 1);
}

TEST(test_gear_reverse_full_sequence) {
    umb_gear_t g = umb_gear_init();
    g = umb_gear_step(g, -0.5f, 0.5f, 0, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_EQ(umb_gear_drive_dir(g), 0);
    g = umb_gear_step(g, -0.5f, 0.05f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    ASSERT_TRUE(umb_gear_braking(g));
    g = umb_gear_step(g, -0.5f, 0.0f, 20 + 649, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, -0.5f, 0.0f, 20 + 650, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    ASSERT_FALSE(umb_gear_braking(g));
    ASSERT_EQ(umb_gear_drive_dir(g), 0);
    g = umb_gear_step(g, -0.5f, 0.0f, 20 + 650 + 79, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, -0.5f, 0.0f, 20 + 650 + 80, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_REVERSE);
    ASSERT_EQ(umb_gear_drive_dir(g), -1);
}

TEST(test_gear_wait_stop_times_out) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 1.0f, 0, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    g = umb_gear_step(g, -0.5f, 1.0f, 999, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    g = umb_gear_step(g, -0.5f, 1.0f, 1000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
}

TEST(test_gear_reverse_to_forward_waits_stop) {
    umb_gear_t g = reach_reverse(0);
    ASSERT_EQ(g.phase, UMB_GEAR_REVERSE);
    g = umb_gear_step(g, 0.5f, 0.3f, 2000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_EQ(umb_gear_drive_dir(g), 0);
    g = umb_gear_step(g, 0.5f, 0.05f, 2020, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_FORWARD);
}

TEST(test_gear_reverse_to_forward_timeout) {
    umb_gear_t g = reach_reverse(0);
    g = umb_gear_step(g, 0.5f, 0.3f, 2000, &C);
    g = umb_gear_step(g, 0.5f, 0.3f, 2999, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    g = umb_gear_step(g, 0.5f, 0.3f, 3000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_FORWARD);
}

TEST(test_gear_deadband_jitter_no_brake) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), 0.5f, 0.5f, 0, &C);
    for (int i = 0; i < 50; i++) {
        g = umb_gear_step(g, (i & 1) ? -0.02f : 0.02f, 0.5f, 20u * (uint32_t)(i + 1), &C);
        ASSERT_TRUE(g.phase != UMB_GEAR_BRAKE);
        ASSERT_TRUE(g.phase != UMB_GEAR_WAIT_STOP);
        ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
    }
}

TEST(test_gear_deadband_from_reverse_is_neutral) {
    umb_gear_t g = reach_reverse(0);
    g = umb_gear_step(g, 0.0f, 0.5f, 2000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

TEST(test_gear_cancel_reverse_during_brake) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 0.0f, 0, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    uint32_t t = 0;
    g = umb_gear_step(g, 0.5f, 0.5f, t += 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_EQ(g.target_dir, 1);
    for (int i = 0; i < 100; i++) {
        g = umb_gear_step(g, 0.5f, 0.5f, t += 20, &C);
        ASSERT_TRUE(g.phase != UMB_GEAR_REVERSE);
    }
    ASSERT_EQ(g.phase, UMB_GEAR_FORWARD);
}

TEST(test_gear_cancel_reverse_deadband_in_sequence) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 0.5f, 0, &C);
    g = umb_gear_step(g, 0.0f, 0.5f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
    g = umb_gear_step(g, -0.5f, 0.0f, 40, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, 0.0f, 0.0f, 60, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
    g = umb_gear_step(g, -0.5f, 0.0f, 80, &C);
    g = umb_gear_step(g, -0.5f, 0.0f, 80 + 650, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, 0.0f, 0.0f, 80 + 660, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

TEST(test_gear_redirect_to_reverse_from_wait_stop_forward) {
    umb_gear_t g = reach_reverse(0);
    g = umb_gear_step(g, 0.5f, 0.3f, 2000, &C);
    ASSERT_EQ(g.target_dir, 1);
    g = umb_gear_step(g, -0.5f, 0.3f, 2020, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_EQ(g.target_dir, -1);
}

TEST(test_gear_deadline_across_wrap) {
    uint32_t t0 = UINT32_MAX - 100;
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 0.0f, t0, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, -0.5f, 0.0f, t0 + 649, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, -0.5f, 0.0f, t0 + 650, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, -0.5f, 0.0f, t0 + 650 + 80, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_REVERSE);
}

TEST(test_gear_speed_sign_reverse) {
    umb_gear_t g = reach_reverse(0);
    ASSERT_FLOAT_EQ(umb_gear_speed_sign(g), -1.0f, 0.0001f);
}

TEST(test_gear_speed_sign_wait_stop_after_reverse) {
    umb_gear_t g = reach_reverse(0);
    g = umb_gear_step(g, 0.5f, 0.3f, 2000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_FLOAT_EQ(umb_gear_speed_sign(g), -1.0f, 0.0001f);
    umb_gear_t f = umb_gear_step(umb_gear_init(), -0.5f, 0.5f, 0, &C);
    ASSERT_EQ(f.phase, UMB_GEAR_WAIT_STOP);
    ASSERT_FLOAT_EQ(umb_gear_speed_sign(f), 1.0f, 0.0001f);
    ASSERT_FLOAT_EQ(umb_gear_speed_sign(umb_gear_init()), 1.0f, 0.0001f);
}

TEST(test_gear_reverse_from_stopped_brakes_same_step) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 0.0f, 1000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    ASSERT_EQ(g.deadline_ms, 1000u + 650u);
    g = umb_gear_step(umb_gear_init(), 0.5f, 0.0f, 0, &C);
    g = umb_gear_step(g, -0.5f, 0.0f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
}

TEST(test_gear_reverse_to_forward_stopped_same_step) {
    umb_gear_t g = reach_reverse(0);
    g = umb_gear_step(g, 0.5f, 0.0f, 2000, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_FORWARD);
}

TEST(test_gear_emergency_brake_ends_neutral) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), 0.5f, 0.5f, 0, &C);
    g = umb_gear_emergency_brake(g, 100, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    ASSERT_EQ(g.target_dir, 0);
    ASSERT_TRUE(umb_gear_braking(g));
    g = umb_gear_step(g, 0.5f, 0.5f, 100 + 649, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, 0.5f, 0.5f, 100 + 650, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, 0.5f, 0.5f, 100 + 650 + 79, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, 0.5f, 0.5f, 100 + 650 + 80, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

TEST(test_gear_emergency_brake_ignores_neutral_request) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), 0.5f, 0.5f, 0, &C);
    g = umb_gear_emergency_brake(g, 0, &C);
    g = umb_gear_step(g, 0.0f, 0.5f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, -0.5f, 0.0f, 40, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_BRAKE);
    g = umb_gear_step(g, 0.0f, 0.0f, 650, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, -0.5f, 0.0f, 660, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_GAP);
    g = umb_gear_step(g, -0.5f, 0.0f, 730, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

TEST(test_gear_emergency_brake_other_phases_unchanged) {
    umb_gear_t n = umb_gear_init();
    umb_gear_t r = reach_reverse(0);
    umb_gear_t w = umb_gear_step(umb_gear_init(), -0.5f, 0.5f, 0, &C);
    ASSERT_EQ(umb_gear_emergency_brake(n, 5, &C).phase, UMB_GEAR_NEUTRAL);
    ASSERT_EQ(umb_gear_emergency_brake(r, 5, &C).phase, UMB_GEAR_REVERSE);
    ASSERT_EQ(umb_gear_emergency_brake(w, 5, &C).phase, UMB_GEAR_WAIT_STOP);
}

TEST(test_gear_nan_request_is_neutral) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), 0.5f, 0.5f, 0, &C);
    g = umb_gear_step(g, NAN, 0.5f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
    g = umb_gear_step(g, INFINITY, 0.5f, 40, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
    g = umb_gear_step(g, 0.5f, 0.5f, 60, &C);
    g = umb_gear_step(g, -INFINITY, 0.5f, 80, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

TEST(test_gear_nan_request_is_neutral_cancels_sequence) {
    umb_gear_t g = umb_gear_step(umb_gear_init(), -0.5f, 0.5f, 0, &C);
    g = umb_gear_step(g, NAN, 0.5f, 20, &C);
    ASSERT_EQ(g.phase, UMB_GEAR_NEUTRAL);
}

int main(void) {
    C = umb_settings_defaults();
    RUN_TEST(test_gear_forward_immediate);
    RUN_TEST(test_gear_reverse_full_sequence);
    RUN_TEST(test_gear_wait_stop_times_out);
    RUN_TEST(test_gear_reverse_to_forward_waits_stop);
    RUN_TEST(test_gear_reverse_to_forward_timeout);
    RUN_TEST(test_gear_deadband_jitter_no_brake);
    RUN_TEST(test_gear_deadband_from_reverse_is_neutral);
    RUN_TEST(test_gear_cancel_reverse_during_brake);
    RUN_TEST(test_gear_cancel_reverse_deadband_in_sequence);
    RUN_TEST(test_gear_redirect_to_reverse_from_wait_stop_forward);
    RUN_TEST(test_gear_deadline_across_wrap);
    RUN_TEST(test_gear_speed_sign_reverse);
    RUN_TEST(test_gear_speed_sign_wait_stop_after_reverse);
    RUN_TEST(test_gear_reverse_from_stopped_brakes_same_step);
    RUN_TEST(test_gear_reverse_to_forward_stopped_same_step);
    RUN_TEST(test_gear_emergency_brake_ends_neutral);
    RUN_TEST(test_gear_emergency_brake_ignores_neutral_request);
    RUN_TEST(test_gear_emergency_brake_other_phases_unchanged);
    RUN_TEST(test_gear_nan_request_is_neutral);
    RUN_TEST(test_gear_nan_request_is_neutral_cancels_sequence);
    TEST_SUMMARY();
}
