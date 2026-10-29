#include "test_runner.h"
#include "umb_reflex.h"
#include <math.h>
#include <string.h>

#define BIT(id) (1u << (id))
#define FR 1
#define FL 4

static umb_sensor_in_t no_sens(void) {
    umb_sensor_in_t s;
    memset(&s, 0, sizeof s);
    for (int i = 0; i < 6; i++) s.tof_mm[i] = 2000;
    return s;
}
static umb_sensor_in_t tof(int16_t fr) {
    umb_sensor_in_t s = no_sens();
    s.tof_present = true; s.tof_online = 6;
    s.tof_mm[FR] = fr;
    return s;
}
static umb_reflex_out_t step1(umb_drive_mode_t m, const umb_sensor_in_t *s, float speed,
                              const umb_settings_t *c) {
    umb_reflex_out_t o;
    umb_reflex_step(umb_reflex_init(), m, s, speed, 0, 0, 1000u, c, &o);
    return o;
}

TEST(test_reflex_nothing_present_no_effect) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_EQ(o.detected_mask, 0u);
    ASSERT_FALSE(o.stop);
    ASSERT_TRUE(isinf(o.speed_cap_ms));
    ASSERT_FLOAT_EQ(o.nearest_mm, 9999.0f, 0.1f);
    ASSERT_TRUE(isinf(o.ttc_ms));
}

TEST(test_reflex_collision_auto_brakes) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(300);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_TRUE(o.emergency_brake);
    ASSERT_TRUE(o.block_forward);
    ASSERT_TRUE(o.intervene_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_FLOAT_EQ(o.ttc_ms, 300.0f, 0.5f);
    ASSERT_FLOAT_EQ(o.nearest_mm, 300.0f, 0.1f);
}

TEST(test_reflex_collision_manual_warns_only) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(300);
    umb_reflex_out_t o = step1(UMB_DRIVE_MANUAL, &s, 1.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_FALSE(o.emergency_brake);
    ASSERT_FALSE(o.block_forward);
    ASSERT_EQ(o.intervene_mask & BIT(UMB_REFLEX_COLLISION), 0u);
}

TEST(test_reflex_collision_ros_delegates) {
    umb_settings_t c = umb_settings_defaults();
    c.reflex_auto[UMB_REFLEX_COLLISION] = UMB_RX_ROS;
    umb_sensor_in_t s = tof(300);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_TRUE(o.delegated_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_FALSE(o.emergency_brake);
    ASSERT_EQ(o.intervene_mask & BIT(UMB_REFLEX_COLLISION), 0u);
}

TEST(test_reflex_collision_ignores_side_sensors) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(2000);
    s.tof_mm[2] = 50;
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_EQ(o.detected_mask & BIT(UMB_REFLEX_COLLISION), 0u);
    ASSERT_FLOAT_EQ(o.nearest_mm, 2000.0f, 0.1f);
}

TEST(test_reflex_reverse_not_blocked) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(100);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, -0.5f, &c);
    ASSERT_FALSE(o.emergency_brake);
    ASSERT_TRUE(o.block_forward);   /* nearest < min_front still blocks forward */
    ASSERT_TRUE(isinf(o.ttc_ms));
}

TEST(test_reflex_stopped_near_blocks_forward_no_brake) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(100);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_TRUE(o.block_forward);
    ASSERT_FALSE(o.emergency_brake);
    s = tof(200);   /* beyond min_front: not blocked when stopped */
    o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_FALSE(o.block_forward);
}

TEST(test_reflex_invalid_tof_ignored) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(9999);
    s.tof_mm[FL] = 0;
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_EQ(o.detected_mask & (BIT(UMB_REFLEX_COLLISION) | BIT(UMB_REFLEX_SPEED_CAP)), 0u);
    ASSERT_FLOAT_EQ(o.nearest_mm, 9999.0f, 0.1f);
    s.tof_mm[FL] = 800;   /* a valid one on the other sensor is used */
    o = step1(UMB_DRIVE_AUTO, &s, 0.1f, &c);
    ASSERT_FLOAT_EQ(o.nearest_mm, 800.0f, 0.1f);
}

TEST(test_reflex_speed_cap_linear) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(650);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_SPEED_CAP));
    ASSERT_FLOAT_EQ(o.speed_cap_ms, 0.6f, 0.01f);
    ASSERT_FLOAT_EQ(o.throttle_cap, 0.6f, 0.01f);
    s = tof(200);
    o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_FLOAT_EQ(o.speed_cap_ms, 0.2f, 0.001f);
    s = tof(1500);
    o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_EQ(o.detected_mask & BIT(UMB_REFLEX_SPEED_CAP), 0u);
    ASSERT_TRUE(isinf(o.speed_cap_ms));
}

TEST(test_reflex_speed_cap_manual_warn_no_cap) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(650);
    umb_reflex_out_t o = step1(UMB_DRIVE_MANUAL, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_SPEED_CAP));
    ASSERT_TRUE(isinf(o.speed_cap_ms));
    ASSERT_FLOAT_EQ(o.throttle_cap, 1.0f, 0.0001f);
}

TEST(test_reflex_caps_take_minimum) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(650);
    s.tof_online = 5;   /* sensor fault cap 0.2 beats distance cap 0.6 */
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_FLOAT_EQ(o.speed_cap_ms, 0.2f, 0.001f);
    ASSERT_FLOAT_EQ(o.throttle_cap, 0.2f, 0.001f);
}

TEST(test_reflex_stall_latch) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_init();
    uint32_t t = 1000u;
    int stopped_at = -1;
    for (int i = 0; i < 40; i++, t += 20u) {
        r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 800000u, t, &c, &o);
        if (o.stop && stopped_at < 0) stopped_at = i * 20;
        if (i * 20 < 700) ASSERT_FALSE(o.stop);
    }
    ASSERT_EQ(stopped_at, 700);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_STALL));
    /* fresh tach: still latched while offset stays */
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 1000u, t, &c, &o);
    ASSERT_TRUE(o.stop);
    /* neutral releases */
    t += 20u;
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 1000u, t, &c, &o);
    ASSERT_FALSE(o.stop);
    ASSERT_FALSE(r.stall_latched);
}

TEST(test_reflex_stall_not_on_start_from_rest) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_init();
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 10000000u, 1000u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 10000000u, 1020u, &c, &o);
    ASSERT_FALSE(o.stop);
    ASSERT_EQ(o.detected_mask & BIT(UMB_REFLEX_STALL), 0u);
}

TEST(test_reflex_stall_offset_drop_resets_timer) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_init();
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, -100, 900000u, 1000u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 900000u, 1600u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, -100, 900000u, 1620u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, -100, 900000u, 2300u, &c, &o);
    ASSERT_FALSE(o.stop);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, -100, 900000u, 2320u, &c, &o);
    ASSERT_TRUE(o.stop);   /* reverse drive offset counts via |offset| */
}

TEST(test_reflex_stall_across_wrap) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_init();
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 900000u, UINT32_MAX - 100u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 900000u, 598u, &c, &o);
    ASSERT_FALSE(o.stop);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 100, 900000u, 600u, &c, &o);
    ASSERT_TRUE(o.stop);
}

TEST(test_reflex_impact_disarms_auto) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    s.imu_present = true; s.imu_ok = true;
    s.accel_g[2] = 5.0f;
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.stop);
    ASSERT_TRUE(o.disarm);
    ASSERT_TRUE(o.intervene_mask & BIT(UMB_REFLEX_IMPACT));
}

TEST(test_reflex_upside_down_needs_500ms) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    s.imu_present = true; s.imu_ok = true;
    s.accel_g[2] = -1.0f;
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_init();
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 0, 1000u, &c, &o);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 0, 1400u, &c, &o);
    ASSERT_EQ(o.detected_mask, 0u);
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 0, 1500u, &c, &o);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_IMPACT));
    ASSERT_TRUE(o.stop);
    /* righting the car resets the timer */
    s.accel_g[2] = 1.0f;
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 0, 1520u, &c, &o);
    ASSERT_EQ(o.detected_mask, 0u);
    s.accel_g[2] = -1.0f;
    r = umb_reflex_step(r, UMB_DRIVE_AUTO, &s, 0.0f, 0, 0, 1540u, &c, &o);
    ASSERT_EQ(o.detected_mask, 0u);
}

TEST(test_reflex_battery_manual_default_acts) {
    umb_settings_t c = umb_settings_defaults();
    c.bat_enabled = true;
    umb_sensor_in_t s = no_sens();
    s.bat_present = true; s.battery_v = 5.5f;
    umb_reflex_out_t o = step1(UMB_DRIVE_MANUAL, &s, 0.0f, &c);
    ASSERT_TRUE(o.stop);
    ASSERT_TRUE(o.disarm);
    s.battery_v = 7.4f;
    o = step1(UMB_DRIVE_MANUAL, &s, 0.0f, &c);
    ASSERT_EQ(o.detected_mask, 0u);
    c.bat_enabled = false;
    s.battery_v = 5.5f;
    o = step1(UMB_DRIVE_MANUAL, &s, 0.0f, &c);
    ASSERT_EQ(o.detected_mask, 0u);
}

TEST(test_reflex_battery_nan_is_low) {
    umb_settings_t c = umb_settings_defaults();
    c.bat_enabled = true;
    umb_sensor_in_t s = no_sens();
    s.bat_present = true; s.battery_v = NAN;
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.stop);
    ASSERT_TRUE(o.disarm);
}

TEST(test_reflex_tof_fault_caps_speed) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = tof(2000);
    s.tof_online = 5;
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_SENSOR_FAULT));
    ASSERT_FLOAT_EQ(o.speed_cap_ms, 0.2f, 0.0001f);
    s.tof_online = 6; s.tof_recovering = true;
    o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_SENSOR_FAULT));
}

TEST(test_reflex_imu_fault_reports_only) {
    umb_settings_t c = umb_settings_defaults();
    umb_sensor_in_t s = no_sens();
    s.imu_present = true; s.imu_ok = false;
    s.accel_g[2] = 9.0f;   /* ignored: impact detection skipped */
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 0.0f, &c);
    ASSERT_TRUE(o.detected_mask & BIT(UMB_REFLEX_SENSOR_FAULT));
    ASSERT_EQ(o.detected_mask & BIT(UMB_REFLEX_IMPACT), 0u);
    ASSERT_FALSE(o.stop);
    ASSERT_TRUE(isinf(o.speed_cap_ms));
}

TEST(test_reflex_off_not_evaluated) {
    umb_settings_t c = umb_settings_defaults();
    c.reflex_auto[UMB_REFLEX_COLLISION] = UMB_RX_OFF;
    umb_sensor_in_t s = tof(300);
    umb_reflex_out_t o = step1(UMB_DRIVE_AUTO, &s, 1.0f, &c);
    ASSERT_EQ(o.detected_mask & BIT(UMB_REFLEX_COLLISION), 0u);
    ASSERT_FALSE(o.emergency_brake);
}

TEST(test_reflex_stop_disarm_ored_and_state_mask) {
    umb_settings_t c = umb_settings_defaults();
    c.bat_enabled = true;
    umb_sensor_in_t s = tof(300);
    s.bat_present = true; s.battery_v = 1.0f;
    umb_reflex_out_t o;
    umb_reflex_t r = umb_reflex_step(umb_reflex_init(), UMB_DRIVE_AUTO, &s, 1.0f, 0, 0, 1000u, &c, &o);
    ASSERT_TRUE(o.stop && o.disarm && o.emergency_brake);
    ASSERT_EQ(r.active_mask, o.detected_mask);
}

TEST(test_reflex_names) {
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_BATTERY), "battery") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_COLLISION), "collision") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_SPEED_CAP), "speed_cap") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_STALL), "stall") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_IMPACT), "impact") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_SENSOR_FAULT), "sensor_fault") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name(UMB_REFLEX_COUNT), "unknown") == 0);
    ASSERT_TRUE(strcmp(umb_reflex_name((umb_reflex_id_t)-1), "unknown") == 0);
}

int main(void) {
    RUN_TEST(test_reflex_nothing_present_no_effect);
    RUN_TEST(test_reflex_collision_auto_brakes);
    RUN_TEST(test_reflex_collision_manual_warns_only);
    RUN_TEST(test_reflex_collision_ros_delegates);
    RUN_TEST(test_reflex_collision_ignores_side_sensors);
    RUN_TEST(test_reflex_reverse_not_blocked);
    RUN_TEST(test_reflex_stopped_near_blocks_forward_no_brake);
    RUN_TEST(test_reflex_invalid_tof_ignored);
    RUN_TEST(test_reflex_speed_cap_linear);
    RUN_TEST(test_reflex_speed_cap_manual_warn_no_cap);
    RUN_TEST(test_reflex_caps_take_minimum);
    RUN_TEST(test_reflex_stall_latch);
    RUN_TEST(test_reflex_stall_not_on_start_from_rest);
    RUN_TEST(test_reflex_stall_offset_drop_resets_timer);
    RUN_TEST(test_reflex_stall_across_wrap);
    RUN_TEST(test_reflex_impact_disarms_auto);
    RUN_TEST(test_reflex_upside_down_needs_500ms);
    RUN_TEST(test_reflex_battery_manual_default_acts);
    RUN_TEST(test_reflex_battery_nan_is_low);
    RUN_TEST(test_reflex_tof_fault_caps_speed);
    RUN_TEST(test_reflex_imu_fault_reports_only);
    RUN_TEST(test_reflex_off_not_evaluated);
    RUN_TEST(test_reflex_stop_disarm_ored_and_state_mask);
    RUN_TEST(test_reflex_names);
    TEST_SUMMARY();
}
