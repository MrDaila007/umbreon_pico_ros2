#include "test_runner.h"
#include "umb_imu_filter.h"
#include <math.h>
#include <string.h>

static umb_imu_raw_t raw_gz(float gz) {
    umb_imu_raw_t r;
    memset(&r, 0, sizeof r);
    r.gyro_dps[2] = gz;
    return r;
}
static umb_imu_cal_t cal_n(int n, float a, float b) {
    umb_imu_cal_t c;
    memset(&c, 0, sizeof c);
    for (int i = 0; i < n; i++) {
        umb_imu_raw_t r = raw_gz((i & 1) ? b : a);
        c = umb_imu_cal_add(c, &r);
    }
    return c;
}
static umb_imu_state_t with_bias(float bias) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(UMB_IMU_CAL_SAMPLES, bias, bias);
    umb_imu_cal_finish(&c, &st);
    return st;
}

TEST(test_imu_init_zeroed) {
    umb_imu_state_t st = umb_imu_init();
    ASSERT_FALSE(st.calibrated);
    ASSERT_FALSE(st.have_sample);
    ASSERT_FLOAT_EQ(st.heading_deg, 0.0f, 1e-6f);
    ASSERT_FALSE(umb_imu_is_ok(&st, 0));
}

TEST(test_imu_cal_bias) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(200, 1.5f, 1.5f);
    ASSERT_TRUE(umb_imu_cal_finish(&c, &st));
    ASSERT_TRUE(st.calibrated);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[2], 1.5f, 1e-4f);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[0], 0.0f, 1e-4f);
}

TEST(test_imu_cal_bias_all_axes) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c;
    memset(&c, 0, sizeof c);
    umb_imu_raw_t r = {{0}, {0.5f, -0.25f, 1.0f}};
    for (int i = 0; i < 200; i++) c = umb_imu_cal_add(c, &r);
    ASSERT_TRUE(umb_imu_cal_finish(&c, &st));
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[0], 0.5f, 1e-4f);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[1], -0.25f, 1e-4f);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[2], 1.0f, 1e-4f);
}

TEST(test_imu_bias_rejects_motion) {
    umb_imu_state_t st = with_bias(1.5f);
    umb_imu_cal_t c = cal_n(200, 0.0f, 10.0f);
    ASSERT_FALSE(umb_imu_cal_finish(&c, &st));
    ASSERT_TRUE(st.calibrated);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[2], 1.5f, 1e-6f);
}

TEST(test_imu_bias_rejects_motion_uncalibrated_stays_so) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(200, 0.0f, 10.0f);
    ASSERT_FALSE(umb_imu_cal_finish(&c, &st));
    ASSERT_FALSE(st.calibrated);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[2], 0.0f, 1e-6f);
}

TEST(test_imu_cal_spread_at_limit_accepted) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(200, 0.0f, UMB_IMU_CAL_MAX_SPREAD_DPS);
    ASSERT_TRUE(umb_imu_cal_finish(&c, &st));
}

TEST(test_imu_cal_too_few_samples) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(199, 1.5f, 1.5f);
    ASSERT_FALSE(umb_imu_cal_finish(&c, &st));
    ASSERT_FALSE(st.calibrated);
    ASSERT_FLOAT_EQ(st.gyro_bias_dps[2], 0.0f, 1e-6f);
}

TEST(test_imu_cal_ignores_nonfinite) {
    umb_imu_cal_t c = cal_n(10, 1.0f, 1.0f);
    umb_imu_raw_t bad = raw_gz(NAN);
    umb_imu_cal_t c2 = umb_imu_cal_add(c, &bad);
    ASSERT_EQ(c2.n, 10);
}

TEST(test_imu_cal_null_args) {
    umb_imu_state_t st = umb_imu_init();
    umb_imu_cal_t c = cal_n(200, 1.0f, 1.0f);
    ASSERT_FALSE(umb_imu_cal_finish(NULL, &st));
    ASSERT_FALSE(umb_imu_cal_finish(&c, NULL));
}

TEST(test_imu_deadzone) {
    umb_imu_state_t st = with_bias(1.0f);
    umb_imu_raw_t r = raw_gz(1.3f);
    st = umb_imu_update(st, &r, true, 100, false);
    ASSERT_FLOAT_EQ(st.yaw_rate_dps, 0.0f, 1e-6f);
}

TEST(test_imu_ema_and_rotate) {
    umb_imu_state_t st = with_bias(1.0f);
    umb_imu_raw_t r = raw_gz(11.0f);
    umb_imu_state_t a = umb_imu_update(st, &r, true, 100, false);
    ASSERT_FLOAT_EQ(a.yaw_rate_dps, 3.0f, 1e-4f);
    a = umb_imu_update(a, &r, true, 110, false);
    ASSERT_FLOAT_EQ(a.yaw_rate_dps, 3.0f + 0.7f * 3.0f + 0.0f, 1e-3f); /* 0.3*10 + 0.7*3 */
    umb_imu_state_t b = umb_imu_update(st, &r, true, 100, true);
    ASSERT_FLOAT_EQ(b.yaw_rate_dps, -3.0f, 1e-4f);
}

TEST(test_imu_heading_integrates) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    uint32_t t = 1000;
    st = umb_imu_update(st, &r, true, t, false);
    for (int i = 0; i < 100; i++) { t += 10; st = umb_imu_update(st, &r, true, t, false); }
    /* EMA settles fast: ~10 deg/s * 1 s, minus a small startup lag */
    ASSERT_FLOAT_EQ(st.heading_deg, 10.0f, 1.0f);
}

TEST(test_imu_heading_wraps) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(300.0f);
    uint32_t t = 0;
    st = umb_imu_update(st, &r, true, t, false);
    for (int i = 0; i < 40; i++) { t += 100; st = umb_imu_update(st, &r, true, t, false); }
    ASSERT_TRUE(fabsf(st.heading_deg) < 360.0f);
    ASSERT_TRUE(st.heading_deg > 0.0f || st.heading_deg < 0.0f);
}

TEST(test_imu_dt_gap_not_integrated) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    st = umb_imu_update(st, &r, true, 1600, false);
    ASSERT_FLOAT_EQ(st.heading_deg, 0.0f, 1e-6f);
    ASSERT_EQ(st.last_ok_ms, 1600u);
}

TEST(test_imu_first_sample_does_not_integrate) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 5000, false);
    ASSERT_FLOAT_EQ(st.heading_deg, 0.0f, 1e-6f);
    ASSERT_TRUE(st.have_sample);
}

TEST(test_imu_same_timestamp_no_integration) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    st = umb_imu_update(st, &r, true, 1000, false);
    ASSERT_FLOAT_EQ(st.heading_deg, 0.0f, 1e-6f);
}

TEST(test_imu_dt_wrap_safe) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 0xFFFFFFFFu - 9u, false);
    st = umb_imu_update(st, &r, true, 10u, false); /* dt = 20 ms across wrap */
    ASSERT_TRUE(st.heading_deg > 0.0f);
    ASSERT_FLOAT_EQ(st.heading_deg, 0.102f, 0.005f); /* 20 ms, EMA 5.1 deg/s */
}

TEST(test_imu_stale_not_ok) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(0.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    ASSERT_TRUE(umb_imu_is_ok(&st, 1000));
    ASSERT_TRUE(umb_imu_is_ok(&st, 1100));
    ASSERT_FALSE(umb_imu_is_ok(&st, 1101));
    ASSERT_FALSE(umb_imu_is_ok(&st, 1150));
}

TEST(test_imu_future_stamp_is_fresh) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(0.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    ASSERT_TRUE(umb_imu_is_ok(&st, 990));
}

TEST(test_imu_is_ok_wrap_and_null) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(0.0f);
    st = umb_imu_update(st, &r, true, 0xFFFFFFF0u, false);
    ASSERT_TRUE(umb_imu_is_ok(&st, 20u));
    ASSERT_FALSE(umb_imu_is_ok(&st, 200u));
    ASSERT_FALSE(umb_imu_is_ok(NULL, 0));
}

TEST(test_imu_all_ones_frame_is_fault) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    umb_imu_state_t f = umb_imu_update(st, &r, false, 1010, false);
    ASSERT_EQ(f.last_ok_ms, 1000u);
    ASSERT_FLOAT_EQ(f.yaw_rate_dps, st.yaw_rate_dps, 1e-9f);
    ASSERT_FLOAT_EQ(f.heading_deg, st.heading_deg, 1e-9f);
}

TEST(test_imu_nan_sample_is_failed_read) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    umb_imu_raw_t bad = raw_gz(10.0f);
    bad.accel_g[1] = INFINITY;
    umb_imu_state_t f = umb_imu_update(st, &bad, true, 1010, false);
    ASSERT_EQ(f.last_ok_ms, 1000u);
    bad = raw_gz(NAN);
    f = umb_imu_update(st, &bad, true, 1010, false);
    ASSERT_EQ(f.last_ok_ms, 1000u);
    ASSERT_FALSE(isnan(f.yaw_rate_dps));
    ASSERT_FALSE(umb_imu_is_ok(&f, 1200));
}

TEST(test_imu_null_sample_is_failed_read) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_state_t f = umb_imu_update(st, NULL, true, 1010, false);
    ASSERT_FALSE(f.have_sample);
}

TEST(test_imu_sample_after_failure_reseeds_time_base) {
    umb_imu_state_t st = with_bias(0.0f);
    umb_imu_raw_t r = raw_gz(10.0f);
    st = umb_imu_update(st, &r, true, 1000, false);
    st = umb_imu_update(st, &r, true, 1010, false);
    float h = st.heading_deg;
    st = umb_imu_update(st, &r, false, 1020, false);
    st = umb_imu_update(st, &r, true, 1030, false); /* gap 20 ms: must not integrate */
    ASSERT_FLOAT_EQ(st.heading_deg, h, 1e-9f);
    st = umb_imu_update(st, &r, true, 1040, false);
    ASSERT_TRUE(st.heading_deg > h);
}

int main(void) {
    RUN_TEST(test_imu_init_zeroed);
    RUN_TEST(test_imu_cal_bias);
    RUN_TEST(test_imu_cal_bias_all_axes);
    RUN_TEST(test_imu_bias_rejects_motion);
    RUN_TEST(test_imu_bias_rejects_motion_uncalibrated_stays_so);
    RUN_TEST(test_imu_cal_spread_at_limit_accepted);
    RUN_TEST(test_imu_cal_too_few_samples);
    RUN_TEST(test_imu_cal_ignores_nonfinite);
    RUN_TEST(test_imu_cal_null_args);
    RUN_TEST(test_imu_deadzone);
    RUN_TEST(test_imu_ema_and_rotate);
    RUN_TEST(test_imu_heading_integrates);
    RUN_TEST(test_imu_heading_wraps);
    RUN_TEST(test_imu_dt_gap_not_integrated);
    RUN_TEST(test_imu_first_sample_does_not_integrate);
    RUN_TEST(test_imu_same_timestamp_no_integration);
    RUN_TEST(test_imu_dt_wrap_safe);
    RUN_TEST(test_imu_stale_not_ok);
    RUN_TEST(test_imu_future_stamp_is_fresh);
    RUN_TEST(test_imu_is_ok_wrap_and_null);
    RUN_TEST(test_imu_all_ones_frame_is_fault);
    RUN_TEST(test_imu_nan_sample_is_failed_read);
    RUN_TEST(test_imu_null_sample_is_failed_read);
    RUN_TEST(test_imu_sample_after_failure_reseeds_time_base);
    TEST_SUMMARY();
}
