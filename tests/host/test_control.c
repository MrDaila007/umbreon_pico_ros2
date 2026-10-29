#include "test_runner.h"
#include "umb_control.h"
#include "umb_actuation.h"
#include <math.h>
#include <string.h>

#define FR 1
#define BIT(id) (1u << (id))

/* Simulated car: a fresh command at `now` each step (no timeout), tach driven
 * by a simulated wheel speed independent of the ESC output. */
typedef struct {
    umb_settings_t c;
    umb_control_t s;
    umb_outputs_t out;
    umb_sensor_in_t sens;
    uint32_t now, tach_count, last_tick_ms;
    float wheel_ms, dist_m;
    umb_cmd_source_t source;
    bool agent_up;
} harness_t;

static void h_init(harness_t *h) {
    memset(h, 0, sizeof *h);
    h->c = umb_settings_defaults();
    h->now = 1000u;
    h->last_tick_ms = 0u;
    h->source = UMB_SRC_CONSOLE;
    h->agent_up = true;
    for (int i = 0; i < 6; i++) h->sens.tof_mm[i] = 9999;
    h->s = umb_control_init(&h->c, 0u);
}

static void h_tof(harness_t *h, int16_t fr) {
    h->sens.tof_present = true;
    h->sens.tof_online = 6;
    for (int i = 0; i < 6; i++) h->sens.tof_mm[i] = 2000;
    h->sens.tof_mm[FR] = fr;
}

static umb_control_in_t h_input(harness_t *h, umb_cmd_kind_t kind, umb_drive_mode_t mode,
                                float steer, float value, bool fresh) {
    umb_control_in_t in;
    memset(&in, 0, sizeof in);
    float per_count = 3.14159265f * h->c.wheel_diam_m / (float)h->c.encoder_holes;
    h->dist_m += h->wheel_ms * 0.020f;
    uint32_t want = (uint32_t)(h->dist_m / per_count);
    if (want != h->tach_count) { h->tach_count = want; h->last_tick_ms = h->now; }
    in.cmd.kind = kind; in.cmd.source = h->source; in.cmd.mode = mode;
    in.cmd.steer = steer; in.cmd.value = value; in.cmd.rx_ms = h->now;
    in.safety.now_ms = h->now;
    in.safety.have_cmd = (kind != UMB_CMD_NONE);
    in.safety.last_cmd_ms = fresh ? h->now : 0u;
    in.safety.source = h->source;
    in.safety.wifi_up = true;
    in.safety.agent_up = h->agent_up;
    in.safety.core0_beat_ms = h->now;
    in.sensors = h->sens;
    in.tach_count = h->tach_count;
    in.tach_age_us = (h->now - h->last_tick_ms) * 1000u;
    return in;
}

static void h_step_ex(harness_t *h, umb_cmd_kind_t kind, umb_drive_mode_t mode,
                      float steer, float value, bool fresh) {
    umb_control_in_t in = h_input(h, kind, mode, steer, value, fresh);
    h->s = umb_control_step(h->s, &in, &h->c, &h->out);
    h->now += 20u;
}

static void h_step(harness_t *h, umb_cmd_kind_t kind, umb_drive_mode_t mode, float steer, float value) {
    h_step_ex(h, kind, mode, steer, value, true);
}

static void h_raw(harness_t *h, float v) { h_step(h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.0f, v); }

/* Run for ms, holding the same command. */
static void h_hold(harness_t *h, umb_cmd_kind_t kind, umb_drive_mode_t mode, float v, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 20u) h_step(h, kind, mode, 0.0f, v);
}

static void h_arm(harness_t *h) {
    h->s.arming = umb_arming_car_key(h->s.arming, NULL);
    h->s.arming = umb_arming_ros(h->s.arming, true, NULL);
    h_raw(h, 0.0f);
}

TEST(test_control_boot_outputs_neutral) {
    harness_t h; h_init(&h);
    h_raw(&h, 1.0f);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.servo_us, 1500);
    ASSERT_FALSE(h.out.armed);
    ASSERT_FALSE(h.out.motion_allowed);
    ASSERT_EQ(h.out.gear, UMB_GEAR_NEUTRAL);
}

TEST(test_control_raw_drive_after_arm_and_neutral) {
    harness_t h; h_init(&h);
    ASSERT_EQ(h.c.arming_mode, UMB_ARM_BOTH);
    h_arm(&h);
    ASSERT_TRUE(h.out.armed);
    h_raw(&h, 1.0f);
    ASSERT_EQ(h.out.esc_us, 1600);
    ASSERT_TRUE(h.out.motion_allowed);
    ASSERT_EQ(h.out.gear, UMB_GEAR_FORWARD);
    ASSERT_EQ(h.out.mode, UMB_DRIVE_MANUAL);
}

TEST(test_control_needs_neutral_after_arm) {
    harness_t h; h_init(&h);
    h.s.arming = umb_arming_car_key(h.s.arming, NULL);
    h.s.arming = umb_arming_ros(h.s.arming, true, NULL);
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 1.0f, 200);
    ASSERT_TRUE(h.out.armed);
    ASSERT_EQ(h.out.esc_us, 1500);
}

TEST(test_control_timeout_stops_without_disarm) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, 0.5f);
    ASSERT_TRUE(h.out.esc_us > 1500);
    for (int i = 0; i < 20; i++) h_step_ex(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.0f, 0.5f, false);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_CMD_TIMEOUT);
    ASSERT_TRUE(h.out.armed);
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.5f, 200);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.stop_mask, 0u);
    h_raw(&h, 0.0f);
    h_raw(&h, 0.5f);
    ASSERT_EQ(h.out.esc_us, umb_esc_us_from_throttle(0.5f, &h.c));
}

/* A command stamped by core 0 just after core 1 read the clock keeps driving. */
TEST(test_control_future_cmd_stamp_keeps_driving) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, 0.5f);
    ASSERT_TRUE(h.out.esc_us > 1500);
    umb_control_in_t in = h_input(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.0f, 0.5f, true);
    in.cmd.rx_ms = h.now + 1u;
    in.safety.last_cmd_ms = h.now + 1u;
    h.s = umb_control_step(h.s, &in, &h.c, &h.out);
    ASSERT_EQ(h.out.stop_mask, 0u);
    ASSERT_TRUE(h.out.armed);
    ASSERT_TRUE(h.out.motion_allowed);
    ASSERT_EQ(h.out.esc_us, umb_esc_us_from_throttle(0.5f, &h.c));
}

TEST(test_control_stale_neutral_does_not_rearm_after_timeout) {
    harness_t h; h_init(&h);
    h_arm(&h);
    for (int i = 0; i < 20; i++) h_step_ex(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.0f, 0.0f, false);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_CMD_TIMEOUT);
    h_raw(&h, 0.5f);
    ASSERT_EQ(h.out.esc_us, 1500);
}

TEST(test_control_agent_loss_disarms) {
    harness_t h; h_init(&h);
    h.source = UMB_SRC_ROS;
    h_arm(&h);
    h_raw(&h, 0.5f);
    ASSERT_TRUE(h.out.esc_us > 1500);
    h.agent_up = false;
    h_raw(&h, 0.5f);
    ASSERT_FALSE(h.out.armed);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_AGENT);
}

TEST(test_control_arming_mode_change_disarms) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, 0.5f);
    ASSERT_TRUE(h.out.esc_us > 1500);
    h.c.arming_mode = UMB_ARM_ENCODER;
    h_raw(&h, 0.5f);
    ASSERT_FALSE(h.out.armed);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.s.arming.mode, UMB_ARM_ENCODER);
}

TEST(test_control_nan_command_is_neutral) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, 0.5f);
    h_step(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, NAN, NAN);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.servo_us, 1500);
    h_step(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, INFINITY, NAN);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.servo_us, 1500);
}

TEST(test_control_raw_reverse_goes_through_brake) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, -1.0f);
    ASSERT_EQ(h.out.esc_us, umb_esc_us_from_throttle(-0.25f, &h.c));
    ASSERT_EQ(h.out.gear, UMB_GEAR_BRAKE);
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, -1.0f, 650 + 80 + 20);
    ASSERT_EQ(h.out.gear, UMB_GEAR_REVERSE);
    ASSERT_EQ(h.out.esc_us, 1285);
}

TEST(test_control_speed_reverse_uses_pid) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_hold(&h, UMB_CMD_SPEED, UMB_DRIVE_MANUAL, -0.3f, 1000);
    ASSERT_EQ(h.out.gear, UMB_GEAR_REVERSE);
    ASSERT_TRUE(h.out.esc_us < 1500);
    ASSERT_TRUE(h.out.esc_us >= umb_esc_reverse_limit_us(&h.c));
}

TEST(test_control_speed_reverse_clamped_to_limit) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_hold(&h, UMB_CMD_SPEED, UMB_DRIVE_MANUAL, -9.0f, 3000);
    ASSERT_EQ(h.out.gear, UMB_GEAR_REVERSE);
    ASSERT_EQ(h.out.esc_us, umb_esc_reverse_limit_us(&h.c));
    ASSERT_TRUE(h.out.speed_signed <= 0.0f);
}

TEST(test_control_speed_mode_clamps_target) {
    harness_t a, b; h_init(&a); h_init(&b);
    h_arm(&a); h_arm(&b);
    for (int i = 0; i < 100; i++) {
        h_step(&a, UMB_CMD_SPEED, UMB_DRIVE_MANUAL, 0.0f, 9.0f);
        h_step(&b, UMB_CMD_SPEED, UMB_DRIVE_MANUAL, 0.0f, a.c.max_speed_ms);
        ASSERT_EQ(a.out.esc_us, b.out.esc_us);
    }
    ASSERT_TRUE(a.out.esc_us > 1500);
    ASSERT_TRUE(a.out.esc_us <= a.c.max_speed);
}

TEST(test_control_steer_passthrough) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_step(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, -1.0f, 0.3f);
    ASSERT_EQ(h.out.servo_us, 1233);
    for (int i = 0; i < 20; i++) h_step_ex(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, -1.0f, 0.3f, false);
    ASSERT_EQ(h.out.servo_us, 1500);
}

TEST(test_control_speed_measured_while_stopped) {
    harness_t h; h_init(&h);
    h.wheel_ms = 1.0f;
    h.c.encoder_holes = 50;
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 1.0f, 1000);   /* not armed: pushed by hand */
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_FLOAT_EQ(h.out.speed_signed, 1.0f, 0.25f);
}

/* AUTO, SPEED 1.0, car rolling at 1.0 m/s with fine tach so the EMA is steady. */
static void drive_at_speed(harness_t *h, umb_drive_mode_t mode) {
    h_init(h);
    h->c.encoder_holes = 50;
    h_tof(h, 2000);
    h_arm(h);
    h->wheel_ms = 1.0f;
    h_hold(h, UMB_CMD_SPEED, mode, 1.0f, 1000);
}

TEST(test_control_auto_collision_brakes) {
    harness_t h;
    drive_at_speed(&h, UMB_DRIVE_AUTO);
    ASSERT_EQ(h.out.gear, UMB_GEAR_FORWARD);
    ASSERT_FLOAT_EQ(h.out.speed_signed, 1.0f, 0.25f);
    h_tof(&h, 300);
    h_step(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 0.0f, 1.0f);
    uint16_t brake = umb_esc_us_from_throttle(h.c.reverse_brake_cmd / 1000.0f, &h.c);
    ASSERT_EQ(h.out.esc_us, brake);
    ASSERT_EQ(h.out.gear, UMB_GEAR_BRAKE);
    h.wheel_ms = 0.0f;
    h_hold(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 1.0f, 640);
    ASSERT_EQ(h.out.esc_us, brake);
    h_hold(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 1.0f, 100);
    ASSERT_EQ(h.out.esc_us, 1500);
    h_tof(&h, 100);
    for (int i = 0; i < 50; i++) {
        h_step(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 0.0f, 1.0f);
        ASSERT_EQ(h.out.esc_us, 1500);
    }
    ASSERT_TRUE(h.out.armed);
}

TEST(test_control_emergency_brake_then_held_forward_stays_stopped) {
    harness_t h;
    drive_at_speed(&h, UMB_DRIVE_AUTO);
    h_tof(&h, 100);
    h_step(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 0.0f, 1.0f);
    ASSERT_EQ(h.out.gear, UMB_GEAR_BRAKE);
    h.wheel_ms = 0.0f;
    for (int i = 0; i < 150; i++) {   /* 3 s, past brake + gap */
        h_step(&h, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 0.0f, 1.0f);
        ASSERT_TRUE(h.out.esc_us <= 1500);
    }
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.gear, UMB_GEAR_NEUTRAL);
    ASSERT_TRUE(h.out.reflex.block_forward);
}

TEST(test_control_auto_speed_cap_limits_target) {
    harness_t capped, free_run;
    drive_at_speed(&capped, UMB_DRIVE_AUTO);
    drive_at_speed(&free_run, UMB_DRIVE_MANUAL);
    capped.c.reflex_auto[UMB_REFLEX_COLLISION] = UMB_RX_OFF;
    free_run.c.reflex_manual[UMB_REFLEX_COLLISION] = UMB_RX_OFF;
    h_tof(&capped, 300); h_tof(&free_run, 300);
    for (int i = 0; i < 50; i++) {
        h_step(&capped, UMB_CMD_SPEED, UMB_DRIVE_AUTO, 0.0f, 1.0f);
        h_step(&free_run, UMB_CMD_SPEED, UMB_DRIVE_MANUAL, 0.0f, 1.0f);
    }
    ASSERT_TRUE(capped.out.reflex.speed_cap_ms < 0.5f);
    ASSERT_TRUE(capped.s.pid.ref_applied <= capped.out.reflex.speed_cap_ms + 1e-3f);
    ASSERT_TRUE(free_run.s.pid.ref_applied > 0.9f);
}

TEST(test_control_auto_raw_throttle_capped) {
    harness_t h; h_init(&h);
    h_tof(&h, 300);
    h.c.reflex_auto[UMB_REFLEX_COLLISION] = UMB_RX_OFF;
    h_arm(&h);
    h_step(&h, UMB_CMD_RAW, UMB_DRIVE_AUTO, 0.0f, 1.0f);
    ASSERT_EQ(h.out.esc_us, umb_esc_us_from_throttle(h.c.cap_min_ms / h.c.max_speed_ms, &h.c));
}

TEST(test_control_manual_collision_passes) {
    harness_t h; h_init(&h);
    h.c.encoder_holes = 50;
    h_tof(&h, 2000);
    h_arm(&h);
    h.wheel_ms = 1.0f;
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.5f, 1000);
    uint16_t fwd = umb_esc_us_from_throttle(0.5f, &h.c);
    ASSERT_EQ(h.out.esc_us, fwd);
    h_tof(&h, 300);
    h_raw(&h, 0.5f);
    h_raw(&h, 0.5f);
    ASSERT_EQ(h.out.esc_us, fwd);
    ASSERT_TRUE(h.out.reflex.detected_mask & BIT(UMB_REFLEX_COLLISION));
    ASSERT_EQ(h.out.stop_mask, 0u);
}

TEST(test_control_battery_reflex_disarms) {
    harness_t h; h_init(&h);
    h.c.bat_enabled = true;
    h_arm(&h);
    h_raw(&h, 0.5f);
    ASSERT_TRUE(h.out.esc_us > 1500);
    h.sens.bat_present = true;
    h.sens.battery_v = 5.0f;
    h_raw(&h, 0.5f);
    ASSERT_FALSE(h.out.armed);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_REFLEX);
}

TEST(test_control_stall_latch_holds_until_neutral) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h.last_tick_ms = 0u;   /* tach never ticks: stale */
    h_hold(&h, UMB_CMD_RAW, UMB_DRIVE_AUTO, 1.0f, 800);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_REFLEX);
    for (int i = 0; i < 100; i++) {   /* 2 s, command held */
        h_step(&h, UMB_CMD_RAW, UMB_DRIVE_AUTO, 0.0f, 1.0f);
        ASSERT_EQ(h.out.esc_us, 1500);
    }
    ASSERT_TRUE(h.out.armed);
    h_step(&h, UMB_CMD_RAW, UMB_DRIVE_AUTO, 0.0f, 0.0f);
    ASSERT_EQ(h.out.esc_us, 1500);
    h_step(&h, UMB_CMD_RAW, UMB_DRIVE_AUTO, 0.0f, 1.0f);
    ASSERT_EQ(h.out.esc_us, 1600);
}

TEST(test_control_no_command_stops) {
    harness_t h; h_init(&h);
    h_arm(&h);
    h_raw(&h, 0.5f);
    h_step(&h, UMB_CMD_NONE, UMB_DRIVE_AUTO, 0.0f, 0.5f);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_TRUE(h.out.stop_mask & UMB_STOP_NO_CMD);
    ASSERT_EQ(h.out.mode, UMB_DRIVE_MANUAL);
}

TEST(test_control_persistent_agent_loss_cannot_rearm) {
    harness_t h; h_init(&h); h.source = UMB_SRC_ROS; h.agent_up = false;
    h_raw(&h, 0.0f);
    h.s.arming = umb_arming_car_key(h.s.arming, NULL);
    h.s.arming = umb_arming_ros(h.s.arming, true, NULL);
    h_raw(&h, 0.0f);
    ASSERT_FALSE(h.out.armed);
    ASSERT_FALSE(h.s.arming.car_key);
    ASSERT_FALSE(h.s.arming.ros_key);
}

TEST(test_control_loss_event_disarms_console_and_partial_keys) {
    harness_t h; h_init(&h); h_arm(&h);
    umb_control_in_t in = h_input(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.0f, 0.5f, true);
    in.network_loss_mask = UMB_STOP_AGENT;
    h.s = umb_control_step(h.s, &in, &h.c, &h.out);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_FALSE(h.out.armed);
    ASSERT_FALSE(h.s.arming.neutral_seen);
    h.s.arming = umb_arming_car_key(h.s.arming, NULL);
    in.safety.have_cmd = false;
    h.s = umb_control_step(h.s, &in, &h.c, &h.out);
    ASSERT_FALSE(h.s.arming.car_key);
}

TEST(test_control_maintenance_inhibits_none_mode) {
    harness_t h; h_init(&h); h.c.arming_mode = UMB_ARM_NONE;
    h_raw(&h, 0.0f);
    umb_control_in_t in = h_input(&h, UMB_CMD_RAW, UMB_DRIVE_MANUAL, 0.7f, 0.5f, true);
    in.maintenance = true;
    h.s = umb_control_step(h.s, &in, &h.c, &h.out);
    ASSERT_EQ(h.out.esc_us, 1500);
    ASSERT_EQ(h.out.servo_us, 1500);
    ASSERT_FALSE(h.out.motion_allowed);
    ASSERT_FALSE(h.s.arming.neutral_seen);
}

int main(void) {
    RUN_TEST(test_control_persistent_agent_loss_cannot_rearm);
    RUN_TEST(test_control_loss_event_disarms_console_and_partial_keys);
    RUN_TEST(test_control_maintenance_inhibits_none_mode);
    RUN_TEST(test_control_boot_outputs_neutral);
    RUN_TEST(test_control_raw_drive_after_arm_and_neutral);
    RUN_TEST(test_control_needs_neutral_after_arm);
    RUN_TEST(test_control_timeout_stops_without_disarm);
    RUN_TEST(test_control_future_cmd_stamp_keeps_driving);
    RUN_TEST(test_control_stale_neutral_does_not_rearm_after_timeout);
    RUN_TEST(test_control_agent_loss_disarms);
    RUN_TEST(test_control_arming_mode_change_disarms);
    RUN_TEST(test_control_nan_command_is_neutral);
    RUN_TEST(test_control_raw_reverse_goes_through_brake);
    RUN_TEST(test_control_speed_reverse_uses_pid);
    RUN_TEST(test_control_speed_reverse_clamped_to_limit);
    RUN_TEST(test_control_speed_mode_clamps_target);
    RUN_TEST(test_control_steer_passthrough);
    RUN_TEST(test_control_speed_measured_while_stopped);
    RUN_TEST(test_control_auto_collision_brakes);
    RUN_TEST(test_control_emergency_brake_then_held_forward_stays_stopped);
    RUN_TEST(test_control_auto_speed_cap_limits_target);
    RUN_TEST(test_control_auto_raw_throttle_capped);
    RUN_TEST(test_control_manual_collision_passes);
    RUN_TEST(test_control_battery_reflex_disarms);
    RUN_TEST(test_control_stall_latch_holds_until_neutral);
    RUN_TEST(test_control_no_command_stops);
    TEST_SUMMARY();
}
