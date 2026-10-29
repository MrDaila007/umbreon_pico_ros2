#include "test_runner.h"
#include "umb_odometry.h"
#include <math.h>

#define PI_F 3.14159265f

/* Drive the odometry at 20 ms steps for `ms` with constant inputs. */
static uint32_t run(umb_odom_t *o, uint32_t t, uint32_t ms, float v, float wz_dps, bool imu) {
    for (uint32_t e = 0; e < ms; e += 20) { t += 20; umb_odom_update(o, t, v, wz_dps, imu); }
    return t;
}

TEST(odom_init_zero) {
    umb_odom_t o = umb_odom_init();
    ASSERT_FLOAT_EQ(o.x_m, 0, 0); ASSERT_FLOAT_EQ(o.y_m, 0, 0); ASSERT_FLOAT_EQ(o.yaw_rad, 0, 0);
    ASSERT_FALSE(o.seeded);
}
TEST(odom_first_update_seeds_only) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 1000, 1.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, 0, 0);
    ASSERT_TRUE(o.seeded);
}
TEST(odom_forward_straight) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 1000, 1.0f, 0, true);
    run(&o, 1000, 1000, 1.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, 1.0f, 1e-3f);
    ASSERT_FLOAT_EQ(o.y_m, 0.0f, 1e-4f);
    ASSERT_FLOAT_EQ(o.vx_mps, 1.0f, 0);
    ASSERT_TRUE(o.vx_valid && o.wz_valid && o.yaw_valid);
}
TEST(odom_reverse_straight) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, -0.5f, 0, true);
    run(&o, 0, 1000, -0.5f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, -0.5f, 1e-3f);
    ASSERT_FLOAT_EQ(o.vx_mps, -0.5f, 0);
}
TEST(odom_turn_arc) {
    /* v=1 m/s, wz=90 deg/s for 1 s: quarter circle radius 2/pi */
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 90.0f, true);
    run(&o, 0, 1000, 1.0f, 90.0f, true);
    float r = 2.0f / PI_F;
    ASSERT_FLOAT_EQ(o.yaw_rad, PI_F / 2, 1e-3f);
    ASSERT_FLOAT_EQ(o.x_m, r, 5e-3f);
    ASSERT_FLOAT_EQ(o.y_m, r, 5e-3f);
}
TEST(odom_yaw_wraps_past_pi) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 0.0f, 90.0f, true);
    run(&o, 0, 2500, 0.0f, 90.0f, true); /* 225 deg => -135 deg */
    ASSERT_FLOAT_EQ(o.yaw_rad, -0.75f * PI_F, 1e-3f);
    ASSERT_TRUE(o.yaw_rad > -PI_F && o.yaw_rad <= PI_F);
    /* crossing 360 */
    umb_odom_t p = umb_odom_init();
    umb_odom_update(&p, 0, 0.0f, 90.0f, true);
    run(&p, 0, 4400, 0.0f, 90.0f, true); /* 396 deg => 36 deg */
    ASSERT_FLOAT_EQ(p.yaw_rad, 36.0f * PI_F / 180.0f, 2e-3f);
    /* negative direction through -180 */
    umb_odom_t n = umb_odom_init();
    umb_odom_update(&n, 0, 0.0f, -90.0f, true);
    run(&n, 0, 2500, 0.0f, -90.0f, true);
    ASSERT_FLOAT_EQ(n.yaw_rad, 0.75f * PI_F, 1e-3f);
}
TEST(odom_normalize) {
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(0), 0, 0);
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(PI_F), PI_F, 1e-6f);   /* (-pi, pi] keeps +pi */
    ASSERT_TRUE(umb_odom_normalize_yaw(-PI_F) > 0);                /* -pi maps to +pi */
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(3 * PI_F / 2), -PI_F / 2, 1e-5f);
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(-5 * PI_F / 2), -PI_F / 2, 1e-5f);
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(100 * PI_F + 0.1f), 0.1f, 1e-3f);
    ASSERT_FLOAT_EQ(umb_odom_normalize_yaw(NAN), 0, 0);
}
TEST(odom_quaternion_unit) {
    float q[4];
    umb_odom_quat(PI_F / 2, q);
    ASSERT_FLOAT_EQ(q[0], 0, 0); ASSERT_FLOAT_EQ(q[1], 0, 0);
    ASSERT_FLOAT_EQ(q[2], sinf(PI_F / 4), 1e-6f);
    ASSERT_FLOAT_EQ(q[3], cosf(PI_F / 4), 1e-6f);
    umb_odom_quat(2.0f, q);
    ASSERT_FLOAT_EQ(q[2] * q[2] + q[3] * q[3], 1.0f, 1e-6f);
}
TEST(odom_imu_failure_freezes_yaw_keeps_position) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 90.0f, true);
    uint32_t t = run(&o, 0, 500, 1.0f, 90.0f, true);   /* yaw 45 deg */
    float yaw = o.yaw_rad, x = o.x_m, y = o.y_m;
    ASSERT_FLOAT_EQ(yaw, PI_F / 4, 1e-3f);
    t = run(&o, t, 1000, 1.0f, 999.0f, false);         /* IMU down: rate ignored */
    ASSERT_FLOAT_EQ(o.yaw_rad, yaw, 0);
    ASSERT_FALSE(o.yaw_valid); ASSERT_FALSE(o.wz_valid);
    ASSERT_TRUE(o.cov_yaw >= UMB_ODOM_COV_UNKNOWN);
    ASSERT_TRUE(o.cov_xy > UMB_ODOM_COV_XY);
    ASSERT_FLOAT_EQ(o.x_m - x, cosf(yaw) * 1.0f, 2e-3f);
    ASSERT_FLOAT_EQ(o.y_m - y, sinf(yaw) * 1.0f, 2e-3f);
    ASSERT_TRUE(o.vx_valid);
    /* recovery: yaw integrates again from the held value */
    run(&o, t, 500, 0.0f, 90.0f, true);
    ASSERT_TRUE(o.yaw_valid && o.wz_valid);
    ASSERT_FLOAT_EQ(o.yaw_rad, yaw + PI_F / 4, 1e-3f);
    ASSERT_FLOAT_EQ(o.cov_yaw, UMB_ODOM_COV_YAW, 0);
}
TEST(odom_nonfinite_inputs_never_corrupt) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 0, true);
    run(&o, 0, 100, 1.0f, 0, true);
    float x = o.x_m;
    umb_odom_update(&o, 120, NAN, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, x, 0); ASSERT_FALSE(o.vx_valid);
    umb_odom_update(&o, 140, 1.0f, NAN, true);   /* NaN rate == IMU invalid */
    ASSERT_FALSE(o.yaw_valid);
    ASSERT_TRUE(isfinite(o.x_m) && isfinite(o.yaw_rad));
    umb_odom_update(&o, 160, INFINITY, 0, true);
    ASSERT_TRUE(isfinite(o.x_m));
}
TEST(odom_long_dt_ignored_and_reseeded) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 0, true);
    run(&o, 0, 200, 1.0f, 0, true);
    float x = o.x_m;
    umb_odom_update(&o, 200 + 201, 1.0f, 0, true);     /* 201 ms gap: skipped */
    ASSERT_FLOAT_EQ(o.x_m, x, 0);
    ASSERT_EQ(o.skipped_intervals, 1);
    umb_odom_update(&o, 200 + 221, 1.0f, 0, true);     /* clock re-seeded: normal step */
    ASSERT_FLOAT_EQ(o.x_m, x + 0.02f, 1e-4f);
    umb_odom_update(&o, 200 + 221 + 200, 1.0f, 0, true); /* exactly 200 ms is integrated */
    ASSERT_FLOAT_EQ(o.x_m, x + 0.22f, 1e-4f);
}
TEST(odom_zero_or_backward_dt_skipped) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 1000, 1.0f, 0, true);
    umb_odom_update(&o, 1020, 1.0f, 0, true);
    float x = o.x_m;
    umb_odom_update(&o, 1020, 1.0f, 0, true);
    umb_odom_update(&o, 1000, 1.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, x, 0);
    ASSERT_EQ(o.skipped_intervals, 2);
}
TEST(odom_uint32_wrap_dt) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0xFFFFFFF6u, 1.0f, 0, true);
    umb_odom_update(&o, 0x0000000Au, 1.0f, 0, true);   /* dt = 20 across wrap */
    ASSERT_FLOAT_EQ(o.x_m, 0.02f, 1e-5f);
}
TEST(odom_reconnect_retains_pose_without_jump) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 0, true);
    uint32_t t = run(&o, 0, 1000, 1.0f, 0, true);
    float x = o.x_m;
    /* network outage: odometry not driven for 30 s (and speed would have been nonzero) */
    umb_odom_update(&o, t + 30000, 1.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, x, 0);                      /* retained, no giant interval */
    umb_odom_update(&o, t + 30020, 1.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, x + 0.02f, 1e-4f);
}
TEST(odom_boot_reset_is_init) {
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 1.0f, 0, true);
    run(&o, 0, 500, 1.0f, 0, true);
    ASSERT_TRUE(o.x_m > 0.4f);
    o = umb_odom_init(); /* firmware restart */
    ASSERT_FLOAT_EQ(o.x_m, 0, 0);
}
TEST(odom_uses_measured_not_target) {
    /* only the measured speed argument exists in the API: zero measured => no motion */
    umb_odom_t o = umb_odom_init();
    umb_odom_update(&o, 0, 0.0f, 0, true);
    run(&o, 0, 1000, 0.0f, 0, true);
    ASSERT_FLOAT_EQ(o.x_m, 0, 0);
    ASSERT_FLOAT_EQ(o.vx_mps, 0, 0);
}

int main(void) {
    printf("test_odometry\n");
    RUN_TEST(odom_init_zero); RUN_TEST(odom_first_update_seeds_only);
    RUN_TEST(odom_forward_straight); RUN_TEST(odom_reverse_straight);
    RUN_TEST(odom_turn_arc); RUN_TEST(odom_yaw_wraps_past_pi);
    RUN_TEST(odom_normalize); RUN_TEST(odom_quaternion_unit);
    RUN_TEST(odom_imu_failure_freezes_yaw_keeps_position);
    RUN_TEST(odom_nonfinite_inputs_never_corrupt);
    RUN_TEST(odom_long_dt_ignored_and_reseeded); RUN_TEST(odom_zero_or_backward_dt_skipped);
    RUN_TEST(odom_uint32_wrap_dt); RUN_TEST(odom_reconnect_retains_pose_without_jump);
    RUN_TEST(odom_boot_reset_is_init); RUN_TEST(odom_uses_measured_not_target);
    TEST_SUMMARY();
}
