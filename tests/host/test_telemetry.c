#include "test_runner.h"
#include "umb_telemetry.h"
#include <math.h>
#include <string.h>

static umb_range_sample_t rng(int idx, int16_t mm, uint32_t stamp, bool online, uint32_t now) {
    umb_range_sample_t s;
    umb_range_convert(idx, mm, stamp, online, now, &s);
    return s;
}

/* ---- Range ---- */
TEST(range_finite_in_range) {
    umb_range_sample_t s = rng(0, 500, 1000, true, 1010);
    ASSERT_FLOAT_EQ(s.range_m, 0.5f, 1e-6f);
    ASSERT_TRUE(s.acquired);
    ASSERT_EQ(s.stamp_ms, 1000);
    ASSERT_EQ(s.gen_key, 1000);
    ASSERT_FLOAT_EQ(s.min_range_m, 0.03f, 1e-6f);
    ASSERT_FLOAT_EQ(s.max_range_m, 2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(s.field_of_view_rad, 25.0f * 3.14159265f / 180.0f, 1e-4f);
    ASSERT_EQ(s.radiation_type, 1);
    ASSERT_TRUE(strcmp(s.frame, "tof_hr") == 0);
}
TEST(range_frames_fixed) {
    const char *want[6] = {"tof_hr", "tof_fr", "tof_r", "tof_l", "tof_fl", "tof_hl"};
    for (int i = 0; i < 6; i++) ASSERT_TRUE(strcmp(rng(i, 300, 0, true, 0).frame, want[i]) == 0);
    /* out of range index must not overrun: bounded frame, no healthy sample */
    umb_range_sample_t s = rng(9, 300, 0, true, 0);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    ASSERT_FALSE(s.acquired);
    ASSERT_TRUE(strlen(s.frame) < UMB_FRAME_MAX);
    s = rng(-1, 300, 0, true, 0);
    ASSERT_FALSE(s.acquired);
}
TEST(range_9999_is_plus_inf) {
    umb_range_sample_t s = rng(1, 9999, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    ASSERT_TRUE(s.acquired); /* a fresh "no return" is a real acquisition */
}
TEST(range_fresh_oor_8190_is_plus_inf) {
    umb_range_sample_t s = rng(1, 8190, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
}
TEST(range_offline_and_stale_plus_inf) {
    umb_range_sample_t s = rng(2, 500, 1000, false, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    ASSERT_FALSE(s.acquired);
    ASSERT_EQ(s.stamp_ms, 1000); /* now */
    s = rng(2, 500, 1000, true, 1000 + UMB_TOF_STALE_MS + 1);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    ASSERT_FALSE(s.acquired);
    s = rng(2, 500, 1000, true, 1000 + UMB_TOF_STALE_MS);
    ASSERT_TRUE(s.acquired);
}
TEST(range_below_min_minus_inf) {
    umb_range_sample_t s = rng(3, 20, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m < 0);
    ASSERT_TRUE(s.acquired);
    s = rng(3, 30, 1000, true, 1000); /* exactly min is a normal reading */
    ASSERT_FLOAT_EQ(s.range_m, 0.03f, 1e-6f);
}
TEST(range_above_max_plus_inf) {
    umb_range_sample_t s = rng(3, 2001, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    s = rng(3, 2000, 1000, true, 1000);
    ASSERT_FLOAT_EQ(s.range_m, 2.0f, 1e-6f);
}
TEST(range_nonpositive_is_invalid_not_minus_inf) {
    umb_range_sample_t s = rng(3, 0, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
    s = rng(3, -5, 1000, true, 1000);
    ASSERT_TRUE(isinf(s.range_m) && s.range_m > 0);
}
TEST(range_future_stamp_not_stale) {
    umb_range_sample_t s = rng(0, 500, 1003, true, 1000);
    ASSERT_TRUE(s.acquired);
}

/* ---- IMU ---- */
TEST(imu_si_units) {
    float a[3] = {1.0f, -0.5f, 0.0f}, g[3] = {180.0f, 90.0f, -45.0f};
    umb_imu_sample_t s;
    umb_imu_convert(a, g, true, 777, &s);
    ASSERT_TRUE(s.healthy);
    ASSERT_FLOAT_EQ(s.accel_mps2[0], 9.80665f, 1e-5f);
    ASSERT_FLOAT_EQ(s.accel_mps2[1], -4.903325f, 1e-5f);
    ASSERT_FLOAT_EQ(s.gyro_rps[0], 3.14159265f, 1e-5f);
    ASSERT_FLOAT_EQ(s.gyro_rps[2], -0.78539816f, 1e-5f);
    ASSERT_FLOAT_EQ(s.orientation_covariance0, -1.0f, 0.0f);
    ASSERT_TRUE(strcmp(s.frame, "imu_link") == 0);
    ASSERT_EQ(s.stamp_ms, 777);
}
TEST(imu_not_ok_or_nonfinite_unhealthy) {
    float a[3] = {0, 0, 1}, g[3] = {0, 0, 0};
    umb_imu_sample_t s;
    umb_imu_convert(a, g, false, 1, &s);
    ASSERT_FALSE(s.healthy);
    g[1] = NAN;
    umb_imu_convert(a, g, true, 1, &s);
    ASSERT_FALSE(s.healthy);
    g[1] = 0; a[2] = INFINITY;
    umb_imu_convert(a, g, true, 1, &s);
    ASSERT_FALSE(s.healthy);
    ASSERT_FLOAT_EQ(s.orientation_covariance0, -1.0f, 0.0f); /* still unsupported */
}

/* ---- Battery ---- */
TEST(battery_known_voltage_other_fields_unknown) {
    umb_battery_sample_t b;
    umb_battery_convert(7.9f, true, 55, &b);
    ASSERT_TRUE(b.present);
    ASSERT_FLOAT_EQ(b.voltage_v, 7.9f, 1e-6f);
    ASSERT_TRUE(isnan(b.current_a) && isnan(b.charge_ah) && isnan(b.capacity_ah) && isnan(b.percentage));
    ASSERT_EQ(b.status, 0); ASSERT_EQ(b.health, 0); ASSERT_EQ(b.technology, 0);
    ASSERT_EQ(b.stamp_ms, 55);
}
TEST(battery_unknown_or_invalid) {
    umb_battery_sample_t b;
    umb_battery_convert(7.9f, false, 1, &b);
    ASSERT_FALSE(b.present);
    ASSERT_TRUE(isnan(b.voltage_v));
    umb_battery_convert(NAN, true, 1, &b);
    ASSERT_FALSE(b.present);
    ASSERT_TRUE(isnan(b.voltage_v));
    umb_battery_convert(-1.0f, true, 1, &b);
    ASSERT_FALSE(b.present);
}

/* ---- Time ---- */
TEST(time_unsynced_suppresses) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_ros_stamp_t st = {123, 456};
    ASSERT_FALSE(umb_time_stamp(&t, 1000, &st));
    ASSERT_EQ(st.sec, 123); /* untouched, no fake epoch */
}
TEST(time_epoch_offset) {
    umb_time_sync_t t = umb_time_sync_init();
    /* ROS epoch 1700000000.5 s at mono 2000 ms */
    ASSERT_TRUE(umb_time_sync_set(&t, 1700000000500000000LL, 2000));
    umb_ros_stamp_t st;
    ASSERT_TRUE(umb_time_stamp(&t, 2000, &st));
    ASSERT_EQ(st.sec, 1700000000); ASSERT_EQ(st.nanosec, 500000000);
    ASSERT_TRUE(umb_time_stamp(&t, 3600, &st)); /* +1.6 s */
    ASSERT_EQ(st.sec, 1700000002); ASSERT_EQ(st.nanosec, 100000000);
}
TEST(time_acquisition_not_publish_time) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_set(&t, 10000000000LL, 1000); /* 10 s at 1000 ms */
    uint64_t acq;
    ASSERT_TRUE(umb_sensor_stamp64(1050, 1020, &acq)); /* published 30 ms after acquisition */
    umb_ros_stamp_t st;
    ASSERT_TRUE(umb_time_stamp(&t, acq, &st));
    ASSERT_EQ(st.sec, 10); ASSERT_EQ(st.nanosec, 20000000);
}
TEST(time_uint32_wrap) {
    umb_mono_t m = {0};
    uint64_t n = umb_mono_extend(&m, 0xFFFFFFF0u);
    ASSERT_EQ(n, 0xFFFFFFF0ull);
    n = umb_mono_extend(&m, 0x10u); /* wrapped */
    ASSERT_EQ(n, 0x100000010ull);
    uint64_t acq;
    ASSERT_TRUE(umb_sensor_stamp64(n, 0xFFFFFFF8u, &acq)); /* acquired before the wrap */
    ASSERT_EQ(acq, 0xFFFFFFF8ull);
    ASSERT_TRUE(umb_sensor_stamp64(n, 0x08u, &acq));
    ASSERT_EQ(acq, 0x100000008ull);
    /* stamps stay monotone and positive across wrap */
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_set(&t, 5000000000000LL, 0xFFFFFFF0u);
    umb_ros_stamp_t a, b;
    ASSERT_TRUE(umb_time_stamp(&t, 0xFFFFFFF8ull, &a));
    ASSERT_TRUE(umb_time_stamp(&t, 0x100000008ull, &b));
    int64_t da = (int64_t)(b.sec - a.sec) * 1000000000LL + ((int64_t)b.nanosec - (int64_t)a.nanosec);
    ASSERT_EQ(da, 16000000);
}
TEST(time_future_sample_tolerance) {
    uint64_t acq;
    ASSERT_TRUE(umb_sensor_stamp64(1000, 1004, &acq));
    ASSERT_EQ(acq, 1000); /* clamped to now */
    ASSERT_FALSE(umb_sensor_stamp64(1000, 1000 + UMB_FUTURE_TOLERANCE_MS + 1, &acq));
    ASSERT_FALSE(umb_sensor_stamp64(1000, 1000u + 0x40000000u, &acq));
}
TEST(time_stamp_before_boot_rejected) {
    uint64_t acq;
    ASSERT_FALSE(umb_sensor_stamp64(10, 0xFFFFFF00u, &acq)); /* would be before t=0, not wrap */
}
TEST(time_failed_resync_keeps_offset_invalidate_suppresses) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_set(&t, 10000000000LL, 1000);
    umb_time_sync_failed(&t);
    ASSERT_EQ(t.failed_syncs, 1);
    ASSERT_FALSE(umb_time_sync_set(&t, -5, 2000)); /* bad epoch also counted as failure */
    ASSERT_EQ(t.failed_syncs, 2);
    umb_ros_stamp_t st;
    ASSERT_TRUE(umb_time_stamp(&t, 2000, &st));
    ASSERT_EQ(st.sec, 11);
    umb_time_sync_invalidate(&t);
    ASSERT_FALSE(umb_time_stamp(&t, 2000, &st));
}
TEST(time_failed_before_first_sync_stays_unsynced) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_failed(&t);
    umb_ros_stamp_t st;
    ASSERT_FALSE(umb_time_stamp(&t, 5, &st));
}
TEST(time_backwards_epoch_adjust_affects_stamps_only) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_set(&t, 100000000000LL, 1000);  /* 100 s */
    umb_ros_stamp_t a, b;
    umb_time_stamp(&t, 5000, &a);
    umb_time_sync_set(&t, 90000000000LL, 6000);   /* PC clock stepped back ~10 s */
    umb_time_stamp(&t, 6000, &b);
    ASSERT_EQ(b.sec, 90);
    ASSERT_TRUE(b.sec < a.sec);
    /* monotonic timeline (used by safety/odom) is untouched: extension unaffected */
    umb_mono_t m = {0};
    uint64_t n1 = umb_mono_extend(&m, 5000), n2 = umb_mono_extend(&m, 6000);
    ASSERT_EQ(n2 - n1, 1000);
}
TEST(time_negative_result_rejected) {
    umb_time_sync_t t = umb_time_sync_init();
    umb_time_sync_set(&t, 1000000, 5000); /* epoch tiny vs mono: offset negative */
    umb_ros_stamp_t st;
    ASSERT_FALSE(umb_time_stamp(&t, 1000, &st));
}

/* ---- Generation / cadence ---- */
TEST(gen_advances_only_on_change) {
    umb_gen_t g = {0};
    ASSERT_TRUE(umb_gen_advanced(&g, 0)); /* first sight, even key 0 */
    ASSERT_FALSE(umb_gen_advanced(&g, 0));
    ASSERT_TRUE(umb_gen_advanced(&g, 10));
    ASSERT_FALSE(umb_gen_advanced(&g, 10));
}
TEST(range_track_new_and_transition) {
    umb_range_track_t t = {0};
    umb_range_sample_t s = rng(0, 400, 100, true, 110);
    ASSERT_TRUE(umb_range_is_new(&t, &s));
    ASSERT_FALSE(umb_range_is_new(&t, &s)); /* same acquisition republished in snapshot */
    s = rng(0, 400, 140, true, 150);
    ASSERT_TRUE(umb_range_is_new(&t, &s));
    s = rng(0, 400, 140, false, 2000);      /* went offline: one no-return event */
    ASSERT_TRUE(umb_range_is_new(&t, &s));
    s = rng(0, 400, 140, false, 2040);
    ASSERT_FALSE(umb_range_is_new(&t, &s));
    s = rng(0, 400, 2100, true, 2110);      /* recovered */
    ASSERT_TRUE(umb_range_is_new(&t, &s));
}
static int run_rate(umb_stream_t st, uint32_t period_check_ms) {
    umb_sched_t s = umb_sched_init(0);
    int n = 0;
    for (uint32_t now = 0; now < period_check_ms; now++)
        if (umb_sched_take(&s, st, now, true)) n++;
    return n;
}
TEST(cadence_rates_over_one_second) {
    ASSERT_EQ(run_rate(UMB_STREAM_IMU, 1000), 100);
    ASSERT_EQ(run_rate(UMB_STREAM_ODOM, 1000), 50);
    ASSERT_EQ(run_rate(UMB_STREAM_RANGE, 1000), 25);
    ASSERT_EQ(run_rate(UMB_STREAM_BATTERY, 1000), 1);
    ASSERT_EQ(run_rate(UMB_STREAM_DIAG, 1000), 1);
    ASSERT_EQ(run_rate(UMB_STREAM_DIAG, 3000), 3);
}
TEST(cadence_long_gap_emits_once_no_flood) {
    umb_sched_t s = umb_sched_init(0);
    ASSERT_TRUE(umb_sched_take(&s, UMB_STREAM_IMU, 0, true));
    uint32_t now = 5000; /* 5 s scheduler stall */
    ASSERT_TRUE(umb_sched_take(&s, UMB_STREAM_IMU, now, true));
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_IMU, now, true));
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_IMU, now + 9, true));
    ASSERT_TRUE(umb_sched_take(&s, UMB_STREAM_IMU, now + 10, true));
    for (int i = 0; i < UMB_STREAM_COUNT; i++) { /* every stream once */
        umb_sched_t g = umb_sched_init(0);
        umb_sched_take(&g, (umb_stream_t)i, 0, true);
        ASSERT_TRUE(umb_sched_take(&g, (umb_stream_t)i, 60000, true));
        ASSERT_FALSE(umb_sched_take(&g, (umb_stream_t)i, 60000, true));
    }
}
TEST(cadence_not_new_stays_due_until_sample) {
    umb_sched_t s = umb_sched_init(100);
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_RANGE, 100, false));
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_RANGE, 130, false));
    ASSERT_TRUE(umb_sched_take(&s, UMB_STREAM_RANGE, 135, true));  /* emits as soon as new */
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_RANGE, 136, true));
}
TEST(cadence_uint32_wrap) {
    umb_sched_t s = umb_sched_init(0xFFFFFFF0u);
    int n = 0;
    for (uint32_t i = 0; i < 100; i++)
        if (umb_sched_take(&s, UMB_STREAM_ODOM, 0xFFFFFFF0u + i, true)) n++;
    ASSERT_EQ(n, 5);
}
TEST(cadence_bad_stream_never_due) {
    umb_sched_t s = umb_sched_init(0);
    ASSERT_FALSE(umb_sched_take(&s, UMB_STREAM_COUNT, 100, true));
    ASSERT_FALSE(umb_sched_take(&s, (umb_stream_t)-1, 100, true));
}

int main(void) {
    printf("test_telemetry\n");
    RUN_TEST(range_finite_in_range); RUN_TEST(range_frames_fixed);
    RUN_TEST(range_9999_is_plus_inf); RUN_TEST(range_fresh_oor_8190_is_plus_inf);
    RUN_TEST(range_offline_and_stale_plus_inf); RUN_TEST(range_below_min_minus_inf);
    RUN_TEST(range_above_max_plus_inf); RUN_TEST(range_nonpositive_is_invalid_not_minus_inf);
    RUN_TEST(range_future_stamp_not_stale);
    RUN_TEST(imu_si_units); RUN_TEST(imu_not_ok_or_nonfinite_unhealthy);
    RUN_TEST(battery_known_voltage_other_fields_unknown); RUN_TEST(battery_unknown_or_invalid);
    RUN_TEST(time_unsynced_suppresses); RUN_TEST(time_epoch_offset);
    RUN_TEST(time_acquisition_not_publish_time); RUN_TEST(time_uint32_wrap);
    RUN_TEST(time_future_sample_tolerance); RUN_TEST(time_stamp_before_boot_rejected);
    RUN_TEST(time_failed_resync_keeps_offset_invalidate_suppresses);
    RUN_TEST(time_failed_before_first_sync_stays_unsynced);
    RUN_TEST(time_backwards_epoch_adjust_affects_stamps_only);
    RUN_TEST(time_negative_result_rejected);
    RUN_TEST(gen_advances_only_on_change); RUN_TEST(range_track_new_and_transition);
    RUN_TEST(cadence_rates_over_one_second); RUN_TEST(cadence_long_gap_emits_once_no_flood);
    RUN_TEST(cadence_not_new_stays_due_until_sample); RUN_TEST(cadence_uint32_wrap);
    RUN_TEST(cadence_bad_stream_never_due);
    TEST_SUMMARY();
}
