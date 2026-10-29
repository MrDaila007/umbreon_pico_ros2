#include "test_runner.h"
#include "umb_pid.h"
#include "umb_settings.h"
#include <math.h>

/* Run n steps 20 ms apart starting at t0, adding dcount counts per step. */
static umb_pid_t run(umb_pid_t p, int n, float target, int8_t dir, float steer,
                     uint32_t *count, uint32_t dcount, uint32_t age_us, uint32_t *t,
                     const umb_settings_t *c, float *speed, int *off) {
    for (int i = 0; i < n; i++) {
        *count += dcount;
        p = umb_pid_step(p, target, dir, steer, *count, age_us, *t, c, speed, off);
        *t += 20u;
    }
    return p;
}

TEST(test_pid_zero_target_zero_offset) {
    umb_settings_t c = umb_settings_defaults();
    uint32_t cnt = 0, t = 1000;
    float sp = -1.0f; int off = -1;
    umb_pid_t p = run(umb_pid_reset(0), 5, 0.0f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off);
    (void)p;
    ASSERT_EQ(off, 0);
    ASSERT_FLOAT_EQ(sp, 0.0f, 1e-6f);
}

TEST(test_pid_feedforward_past_deadzone) {
    umb_settings_t c = umb_settings_defaults();
    uint32_t cnt = 0, t = 1000;
    float sp; int off = 0;
    run(umb_pid_reset(0), 2, 0.5f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off);
    ASSERT_TRUE(off >= c.min_speed - 1500);
}

TEST(test_pid_speed_from_count) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off;
    umb_pid_t p = umb_pid_reset(10);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 10, 0, 1000, &c, &sp, &off);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 12, 0, 1100, &c, &sp, &off);
    float raw = (2.0f / 2.0f) * (3.14159265f * 0.060f) / 0.1f;
    ASSERT_FLOAT_EQ(raw, 1.885f, 0.001f);
    ASSERT_FLOAT_EQ(sp, 0.7f * raw, 0.01f);
}

TEST(test_pid_reset_keeps_tach_baseline_across_wrap) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off;
    umb_pid_t p = umb_pid_reset(0xFFFFFFFFu);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 0xFFFFFFFFu, 0, 1000, &c, &sp, &off);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 1u, 0, 1100, &c, &sp, &off);   /* +2 across wrap */
    ASSERT_FLOAT_EQ(sp, 0.7f * 1.885f, 0.01f);
}

TEST(test_pid_stale_tach_zeroes_speed) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off;
    umb_pid_t p = umb_pid_reset(10);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 10, 600000u, 1000, &c, &sp, &off);
    p = umb_pid_step(p, 0.0f, 1, 0.0f, 12, 600000u, 1100, &c, &sp, &off);
    ASSERT_FLOAT_EQ(sp, 0.0f, 1e-6f);
}

TEST(test_pid_kick_forward_only) {
    umb_settings_t kick = umb_settings_defaults();
    umb_settings_t nokick = kick;
    nokick.kick_pct = 0.0f;
    float sp; int off_k = 0, off_n = 0, off_fk = 0, off_fn = 0;
    uint32_t c1 = 0, c2 = 0, c3 = 0, c4 = 0, t1 = 1000, t2 = 1000, t3 = 1000, t4 = 1000;
    run(umb_pid_reset(0), 6, 0.5f, -1, 0.5f, &c1, 0, 0, &t1, &kick, &sp, &off_k);
    run(umb_pid_reset(0), 6, 0.5f, -1, 0.5f, &c2, 0, 0, &t2, &nokick, &sp, &off_n);
    ASSERT_EQ(off_k, off_n);
    /* Sanity: forward does get the kick. */
    run(umb_pid_reset(0), 6, 0.5f, 1, 0.0f, &c3, 0, 0, &t3, &kick, &sp, &off_fk);
    run(umb_pid_reset(0), 6, 0.5f, 1, 0.0f, &c4, 0, 0, &t4, &nokick, &sp, &off_fn);
    ASSERT_TRUE(off_fk > off_fn);
}

TEST(test_pid_corner_kick_forward_when_slow) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int straight = 0, turning = 0;
    uint32_t c1 = 0, c2 = 0, t1 = 1000, t2 = 1000;
    run(umb_pid_reset(0), 6, 0.5f, 1, 0.0f, &c1, 0, 0, &t1, &c, &sp, &straight);
    run(umb_pid_reset(0), 6, 0.5f, 1, 1.0f, &c2, 0, 0, &t2, &c, &sp, &turning);
    ASSERT_EQ(turning - straight, c.corner_kick_us);
}

TEST(test_pid_kick_expires) {
    umb_settings_t kick = umb_settings_defaults();
    umb_settings_t nokick = kick;
    nokick.kick_pct = 0.0f;
    float sp; int off_k = 0, off_n = 0;
    uint32_t c1 = 0, c2 = 0, t1 = 1000, t2 = 1000;
    /* 400 ms > kick_ms (300): the kick is over, outputs match again. */
    run(umb_pid_reset(0), 20, 0.5f, 1, 0.0f, &c1, 0, 0, &t1, &kick, &sp, &off_k);
    run(umb_pid_reset(0), 20, 0.5f, 1, 0.0f, &c2, 0, 0, &t2, &nokick, &sp, &off_n);
    ASSERT_EQ(off_k, off_n);
}

TEST(test_pid_slew_limits_setpoint) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off;
    uint32_t cnt = 0, t = 1000;
    umb_pid_t p = run(umb_pid_reset(0), 11, 0.5f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off);
    /* 10 steps of 20 ms at 0.85 m/s^2 */
    ASSERT_FLOAT_EQ(p.ref_applied, 0.85f * 0.2f, 0.001f);
    c.spd_slew = 0.0f;
    p = run(p, 1, 0.5f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off);
    ASSERT_FLOAT_EQ(p.ref_applied, 0.5f, 1e-6f);
}

TEST(test_pid_integral_clamped) {
    umb_settings_t c = umb_settings_defaults();
    c.spd_slew = 0.0f;
    float sp; int off;
    uint32_t cnt = 0, t = 1000;
    umb_pid_t p = run(umb_pid_reset(0), 3, 1.0f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off);
    /* A 200 s gap: error*dt would be 200, the clamp holds it at 50. */
    p = umb_pid_step(p, 1.0f, 1, 0.0f, cnt, 0, t + 200000u, &c, &sp, &off);
    ASSERT_FLOAT_EQ(p.integral, 50.0f, 1e-3f);
}

TEST(test_pid_short_dt_holds_state) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off1, off2;
    uint32_t cnt = 0, t = 1000;
    umb_pid_t p = run(umb_pid_reset(0), 5, 0.5f, 1, 0.0f, &cnt, 0, 0, &t, &c, &sp, &off1);
    umb_pid_t q = umb_pid_step(p, 0.5f, 1, 0.0f, cnt + 3, 0, t - 20u + 5u, &c, &sp, &off2);
    ASSERT_EQ(q.prev_count, p.prev_count);
    ASSERT_EQ(q.prev_ms, p.prev_ms);
    ASSERT_FLOAT_EQ(q.integral, p.integral, 1e-6f);
    ASSERT_TRUE(off2 > 0);
}

TEST(test_pid_nonfinite_inputs_are_zero) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off = -1;
    uint32_t cnt = 0, t = 1000;
    run(umb_pid_reset(0), 5, NAN, 1, NAN, &cnt, 0, 0, &t, &c, &sp, &off);
    ASSERT_EQ(off, 0);
}

TEST(test_pid_no_holes_no_speed) {
    umb_settings_t c = umb_settings_defaults();
    c.encoder_holes = 0;
    float sp; int off;
    uint32_t cnt = 0, t = 1000;
    run(umb_pid_reset(0), 3, 0.0f, 1, 0.0f, &cnt, 5, 0, &t, &c, &sp, &off);
    ASSERT_FLOAT_EQ(sp, 0.0f, 1e-6f);
}

TEST(test_pid_idle_keeps_measurement) {
    umb_settings_t c = umb_settings_defaults();
    float sp; int off;
    uint32_t cnt = 0, t = 1000;
    umb_pid_t p = run(umb_pid_reset(0), 5, 0.5f, 1, 0.0f, &cnt, 1, 0, &t, &c, &sp, &off);
    umb_pid_t q = umb_pid_idle(p);
    ASSERT_FLOAT_EQ(q.integral, 0.0f, 0.0f);
    ASSERT_FLOAT_EQ(q.ref_applied, 0.0f, 0.0f);
    ASSERT_FLOAT_EQ(q.kick_prev_ref, 0.0f, 0.0f);
    ASSERT_EQ(q.kick_until_ms, 0u);
    ASSERT_EQ(q.prev_count, p.prev_count);
    ASSERT_EQ(q.prev_ms, p.prev_ms);
    ASSERT_FLOAT_EQ(q.filtered, p.filtered, 0.0f);
    ASSERT_TRUE(q.started);
}

int main(void) {
    RUN_TEST(test_pid_zero_target_zero_offset);
    RUN_TEST(test_pid_feedforward_past_deadzone);
    RUN_TEST(test_pid_speed_from_count);
    RUN_TEST(test_pid_reset_keeps_tach_baseline_across_wrap);
    RUN_TEST(test_pid_stale_tach_zeroes_speed);
    RUN_TEST(test_pid_kick_forward_only);
    RUN_TEST(test_pid_corner_kick_forward_when_slow);
    RUN_TEST(test_pid_kick_expires);
    RUN_TEST(test_pid_slew_limits_setpoint);
    RUN_TEST(test_pid_integral_clamped);
    RUN_TEST(test_pid_short_dt_holds_state);
    RUN_TEST(test_pid_nonfinite_inputs_are_zero);
    RUN_TEST(test_pid_no_holes_no_speed);
    RUN_TEST(test_pid_idle_keeps_measurement);
    TEST_SUMMARY();
}
