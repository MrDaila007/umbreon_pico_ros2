/* Bounded ROS message storage and the telemetry tick, using the generated
 * message structs only (no generated init/fini, no allocation). */
#include "test_runner.h"
#include "ros_messages.h"
#include <stdint.h>
#include <string.h>

#define EPOCH_OFFSET_NS 1700000000000000000ll

static ros_messages_t m;
static ros_telemetry_t t;
static umb_time_sync_t sync;
static ros_diag_info_t diag;
static unsigned published[ROS_ENT_COUNT], fail_publish;
static const void *last_msg[ROS_ENT_COUNT];

static bool fake_publish(void *ctx, ros_entity_id_t id, const void *msg) {
    (void)ctx; published[id]++; last_msg[id] = msg;
    if (fail_publish) { fail_publish--; return false; }
    return true;
}
static bool inside(const void *p) {
    return (const char *)p >= (const char *)&m && (const char *)p < (const char *)(&m + 1);
}
static bool bound(const rosidl_runtime_c__String *s, size_t cap) {
    return s->data && inside(s->data) && s->capacity == cap && s->size == strlen(s->data) && s->size < cap;
}
static bool str_is(const rosidl_runtime_c__String *s, const char *text) {
    return s->data && strcmp(s->data, text) == 0 && s->size == strlen(text);
}
static shared_sensors_t sensors(uint32_t stamp) {
    shared_sensors_t s = {0};
    for (int i = 0; i < UMB_TOF_COUNT; i++) { s.tof_mm[i] = (int16_t)(500 + i); s.tof_stamp_ms[i] = stamp; }
    s.tof_online_mask = 0x3f; s.tof_online = 6;
    s.accel_g[2] = 1.0f; s.gyro_dps[2] = 90.0f; s.yaw_rate_dps = 0.0f; s.imu_ok = true; s.imu_stamp_ms = stamp;
    s.bat_v = 7.4f; s.bat_have = true;
    return s;
}
static void setup(void) {
    ros_messages_reset(&m); ros_telemetry_init(&t);
    sync = umb_time_sync_init(); diag = (ros_diag_info_t){0};
    memset(published, 0, sizeof published); memset(last_msg, 0, sizeof last_msg); fail_publish = 0;
}
static void synced(uint64_t mono_ms) { ASSERT_TRUE(umb_time_sync_set(&sync, EPOCH_OFFSET_NS + (int64_t)mono_ms * 1000000, mono_ms)); }
static unsigned tick(const shared_sensors_t *s, uint64_t now) {
    return ros_telemetry_tick(&t, &m, s, 0.5f, now, &sync, &diag, fake_publish, NULL);
}
static unsigned range_pubs(void) {
    unsigned n = 0;
    for (int i = 0; i < UMB_TOF_COUNT; i++) n += published[ROS_ENT_PUB_RANGE_HR + i];
    return n;
}

TEST(test_reset_binds_every_string_to_owned_bounded_storage) {
    setup();
    static const char *const frames[6] = {"tof_hr", "tof_fr", "tof_r", "tof_l", "tof_fl", "tof_hl"};
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        ASSERT_TRUE(bound(&m.range[i].header.frame_id, UMB_FRAME_MAX));
        ASSERT_TRUE(str_is(&m.range[i].header.frame_id, frames[i]));
        ASSERT_EQ(m.range[i].radiation_type, UMB_RANGE_RADIATION_INFRARED);
        ASSERT_FLOAT_EQ(m.range[i].max_range, UMB_RANGE_MAX_M, 1e-6f);
        ASSERT_FLOAT_EQ(m.range[i].field_of_view, UMB_RANGE_FOV_RAD, 1e-6f);
    }
    ASSERT_TRUE(bound(&m.imu.header.frame_id, UMB_FRAME_MAX) && str_is(&m.imu.header.frame_id, "imu_link"));
    ASSERT_FLOAT_EQ((float)m.imu.orientation.w, 1.0f, 0); ASSERT_FLOAT_EQ((float)m.imu.orientation_covariance[0], -1.0f, 0);
    ASSERT_TRUE(str_is(&m.odom.header.frame_id, "odom") && str_is(&m.odom.child_frame_id, "base_link"));
    ASSERT_TRUE(bound(&m.odom.child_frame_id, UMB_FRAME_MAX));
    ASSERT_TRUE(bound(&m.battery.header.frame_id, 1) && bound(&m.battery.location, 1) && bound(&m.battery.serial_number, 1));
    ASSERT_TRUE(m.battery.cell_voltage.data == NULL && m.battery.cell_voltage.size == 0 && m.battery.cell_voltage.capacity == 0);
    ASSERT_TRUE(m.battery.cell_temperature.size == 0 && !m.battery.present);
    ASSERT_TRUE(isnan(m.battery.voltage) && isnan(m.battery.current) && isnan(m.battery.charge) &&
                isnan(m.battery.capacity) && isnan(m.battery.design_capacity) && isnan(m.battery.percentage) &&
                isnan(m.battery.temperature));
    ASSERT_TRUE(bound(&m.reflex.name, ROS_DIAG_NAME_MAX) && bound(&m.reflex.message, ROS_DIAG_MESSAGE_MAX));
    ASSERT_TRUE(m.reflex.values.size == 0 && m.reflex.values.data == NULL);
    ASSERT_TRUE(m.diagnostics.status.data == m.diag_status && m.diagnostics.status.size == 1 &&
                m.diagnostics.status.capacity == 1);
    diagnostic_msgs__msg__DiagnosticStatus *st = &m.diag_status[0];
    ASSERT_TRUE(bound(&st->name, ROS_DIAG_NAME_MAX) && bound(&st->hardware_id, ROS_DIAG_HWID_MAX));
    ASSERT_TRUE(str_is(&st->hardware_id, ROS_DIAG_HWID));
    ASSERT_TRUE(st->values.data == m.diag_values && st->values.size == ROS_DIAG_VALUES &&
                st->values.capacity == ROS_DIAG_VALUES);
    for (unsigned i = 0; i < ROS_DIAG_VALUES; i++) {
        ASSERT_TRUE(bound(&m.diag_values[i].key, ROS_DIAG_KEY_MAX) && m.diag_values[i].key.size > 0);
        ASSERT_TRUE(bound(&m.diag_values[i].value, ROS_DIAG_VALUE_MAX));
    }
    ASSERT_TRUE(bound(&m.arm_response.message, ROS_ARM_MESSAGE_MAX) && m.arm_response.message.size == 0);
    /* Repeated setup is idempotent: an identical known state, no residue. */
    static ros_messages_t first; memcpy(&first, &m, sizeof m);
    ros_messages_arm_reply(&m, true, "armed"); m.cmd[1].linear.x = 3.0;
    ros_messages_reset(&m);
    ASSERT_TRUE(memcmp(&first, &m, sizeof m) == 0);
}

TEST(test_inputs_reset_per_generation_keeps_bindings) {
    setup();
    m.cmd[0].linear.x = 1.0; m.cmd[2].angular.z = -1.0; m.arm_request.data = true;
    ros_messages_arm_reply(&m, true, "armed");
    ros_messages_reset_inputs(&m);
    ASSERT_TRUE(m.cmd[0].linear.x == 0.0 && m.cmd[2].angular.z == 0.0 && !m.arm_request.data);
    ASSERT_FALSE(m.arm_response.success);
    ASSERT_TRUE(bound(&m.arm_response.message, ROS_ARM_MESSAGE_MAX) && m.arm_response.message.size == 0);
}

TEST(test_twist_and_bounded_arm_reply) {
    setup();
    geometry_msgs__msg__Twist tw = {{1.5, 0.25, -0.5}, {0.1, 0.2, -0.75}};
    umb_twist_fields_t f; ros_messages_twist(&tw, &f);
    ASSERT_TRUE(f.linear[0] == 1.5 && f.linear[1] == 0.25 && f.linear[2] == -0.5);
    ASSERT_TRUE(f.angular[0] == 0.1 && f.angular[1] == 0.2 && f.angular[2] == -0.75);
    ros_messages_arm_reply(&m, true, "armed");
    ASSERT_TRUE(m.arm_response.success && str_is(&m.arm_response.message, "armed"));
    char longer[200]; memset(longer, 'x', sizeof longer - 1); longer[sizeof longer - 1] = 0;
    ros_messages_arm_reply(&m, false, longer);
    ASSERT_FALSE(m.arm_response.success);
    ASSERT_TRUE(bound(&m.arm_response.message, ROS_ARM_MESSAGE_MAX));
    ASSERT_EQ(m.arm_response.message.size, ROS_ARM_MESSAGE_MAX - 1);
    ros_messages_arm_reply(&m, false, NULL);
    ASSERT_EQ(m.arm_response.message.size, 0);
}

TEST(test_unsynced_time_publishes_nothing_but_odometry_integrates) {
    setup();
    shared_sensors_t s = sensors(1000);
    for (uint64_t now = 1000; now < 1200; now += 10) ASSERT_EQ(tick(&s, now), 0);
    for (int i = 0; i < ROS_ENT_COUNT; i++) ASSERT_EQ(published[i], 0);
    ASSERT_TRUE(t.odom.seeded); ASSERT_TRUE(t.odom.x_m > 0.08f); /* 0.5 m/s for ~190 ms */
    ASSERT_EQ(t.unstamped, 0);
}

TEST(test_synced_tick_stamps_from_acquisition_and_throttles) {
    setup(); synced(900);
    shared_sensors_t s = sensors(995);
    ros_telemetry_session_start(&t, 1000);
    ASSERT_EQ(tick(&s, 1000), 0);
    ASSERT_EQ(published[ROS_ENT_PUB_IMU], 1); ASSERT_EQ(published[ROS_ENT_PUB_ODOM], 1);
    ASSERT_EQ(published[ROS_ENT_PUB_BATTERY], 1); ASSERT_EQ(published[ROS_ENT_PUB_DIAGNOSTICS], 1);
    ASSERT_EQ(range_pubs(), 6); ASSERT_EQ(published[ROS_ENT_PUB_REFLEX], 0); /* Task 12 */
    /* IMU stamp = epoch offset + acquisition (995 ms), not publication (1000 ms). */
    int64_t ns = EPOCH_OFFSET_NS + 995ll * 1000000;
    ASSERT_EQ(m.imu.header.stamp.sec, (int32_t)(ns / 1000000000)); ASSERT_EQ(m.imu.header.stamp.nanosec, (uint32_t)(ns % 1000000000));
    ASSERT_EQ(m.range[3].header.stamp.nanosec, (uint32_t)(ns % 1000000000));
    ASSERT_FLOAT_EQ(m.range[3].range, 0.503f, 1e-6f);
    ASSERT_FLOAT_EQ((float)m.imu.angular_velocity.z, 90.0f * UMB_DEG_TO_RAD, 1e-5f);
    ASSERT_FLOAT_EQ((float)m.imu.linear_acceleration.z, UMB_GRAVITY_MPS2, 1e-5f);
    ASSERT_TRUE(m.battery.present); ASSERT_FLOAT_EQ(m.battery.voltage, 7.4f, 1e-6f); ASSERT_TRUE(isnan(m.battery.percentage));
    /* Same samples 10 ms later: nothing new, odometry not yet due (50 Hz). */
    ASSERT_EQ(tick(&s, 1010), 0);
    ASSERT_EQ(published[ROS_ENT_PUB_IMU], 1); ASSERT_EQ(published[ROS_ENT_PUB_ODOM], 1); ASSERT_EQ(range_pubs(), 6);
    ASSERT_EQ(tick(&s, 1020), 0); ASSERT_EQ(published[ROS_ENT_PUB_ODOM], 2);
    /* One second of fresh IMU samples every 10 ms: 100 Hz, battery/diagnostics 1 Hz. */
    for (uint64_t now = 1030; now < 2030; now += 10) { s.imu_stamp_ms = (uint32_t)now - 2; (void)tick(&s, now); }
    ASSERT_EQ(published[ROS_ENT_PUB_IMU], 101); ASSERT_EQ(published[ROS_ENT_PUB_BATTERY], 2);
    ASSERT_EQ(published[ROS_ENT_PUB_DIAGNOSTICS], 2); ASSERT_EQ(published[ROS_ENT_PUB_ODOM], 52);
}

TEST(test_range_samples_between_due_ticks_are_not_lost_and_offline_event_once) {
    setup(); synced(900);
    shared_sensors_t s = sensors(1000);
    s.tof_online_mask = 0x3e; /* sensor 0 offline from the start */
    ros_telemetry_session_start(&t, 1000);
    (void)tick(&s, 1000);
    ASSERT_EQ(published[ROS_ENT_PUB_RANGE_HR], 1); ASSERT_TRUE(isinf(m.range[0].range) && m.range[0].range > 0);
    /* A new sample of sensor 2 lands between range periods (40 ms). */
    s.tof_stamp_ms[2] = 1010; (void)tick(&s, 1010);
    ASSERT_EQ(published[ROS_ENT_PUB_RANGE_R], 1);
    for (uint64_t now = 1020; now <= 1040; now += 10) (void)tick(&s, now);
    ASSERT_EQ(published[ROS_ENT_PUB_RANGE_R], 2); /* emitted at the next due tick */
    ASSERT_EQ(published[ROS_ENT_PUB_RANGE_FR], 1); /* unchanged sensors not republished */
    /* The startup +inf event for the offline sensor is never repeated, even
     * across a new session. */
    for (uint64_t now = 1050; now < 3000; now += 10) (void)tick(&s, now);
    ros_telemetry_session_start(&t, 3000);
    for (uint64_t now = 3000; now < 3500; now += 10) (void)tick(&s, now);
    ASSERT_EQ(published[ROS_ENT_PUB_RANGE_HR], 1);
}

TEST(test_unhealthy_imu_is_not_published_and_publish_failures_count) {
    setup(); synced(900);
    shared_sensors_t s = sensors(1000); s.imu_ok = false;
    ros_telemetry_session_start(&t, 1000);
    fail_publish = 2;
    ASSERT_EQ(tick(&s, 1000), 2);
    ASSERT_EQ(published[ROS_ENT_PUB_IMU], 0); ASSERT_EQ(t.publish_failures, 2);
    s.gyro_dps[0] = NAN; s.imu_ok = true; s.imu_stamp_ms = 1005; (void)tick(&s, 1010);
    ASSERT_EQ(published[ROS_ENT_PUB_IMU], 0);
}

TEST(test_odometry_message_covariance_and_unknown_axes) {
    setup(); synced(900);
    shared_sensors_t s = sensors(1000); s.yaw_rate_dps = 90.0f;
    ros_telemetry_session_start(&t, 1000);
    for (uint64_t now = 1000; now <= 1100; now += 10) (void)tick(&s, now);
    const nav_msgs__msg__Odometry *o = &m.odom;
    double q2 = o->pose.pose.orientation.z * o->pose.pose.orientation.z + o->pose.pose.orientation.w * o->pose.pose.orientation.w;
    ASSERT_FLOAT_EQ((float)q2, 1.0f, 1e-5f); ASSERT_TRUE(o->pose.pose.orientation.z > 0.0);
    ASSERT_FLOAT_EQ((float)o->twist.twist.linear.x, 0.5f, 1e-6f);
    ASSERT_FLOAT_EQ((float)o->twist.twist.angular.z, 90.0f * UMB_DEG_TO_RAD, 1e-5f);
    ASSERT_FLOAT_EQ((float)o->pose.covariance[0], UMB_ODOM_COV_XY, 1e-6f);
    ASSERT_FLOAT_EQ((float)o->pose.covariance[35], UMB_ODOM_COV_YAW, 1e-6f);
    for (int i = 14; i <= 28; i += 7) ASSERT_TRUE(o->pose.covariance[i] >= UMB_ODOM_COV_UNKNOWN);
    ASSERT_TRUE(o->twist.covariance[7] >= UMB_ODOM_COV_UNKNOWN); /* lateral speed not measured */
    ASSERT_TRUE(o->pose.covariance[1] == 0.0);
    s.imu_ok = false;
    for (uint64_t now = 1110; now <= 1140; now += 10) (void)tick(&s, now);
    ASSERT_TRUE(o->pose.covariance[35] >= UMB_ODOM_COV_UNKNOWN);
    ASSERT_TRUE(o->twist.covariance[35] >= UMB_ODOM_COV_UNKNOWN);
    ASSERT_TRUE(o->twist.twist.angular.z == 0.0);
    ASSERT_FLOAT_EQ((float)o->pose.covariance[0], UMB_ODOM_COV_XY_NO_IMU, 1e-6f);
    ASSERT_FLOAT_EQ((float)o->twist.twist.linear.x, 0.5f, 1e-6f); /* speed still finite */
}

TEST(test_diagnostics_summary_values_are_bounded_text) {
    setup(); synced(900);
    diag = (ros_diag_info_t){.generation = 4294967295u, .sessions = 3, .losses = 2, .sync_age_ms = 1234};
    shared_sensors_t s = sensors(1000);
    ros_telemetry_session_start(&t, 1000); (void)tick(&s, 1000);
    const diagnostic_msgs__msg__DiagnosticStatus *st = &m.diag_status[0];
    ASSERT_EQ(st->level, 0); ASSERT_TRUE(str_is(&st->name, "umbreon/session"));
    ASSERT_TRUE(str_is(&m.diag_values[0].key, "generation") && str_is(&m.diag_values[0].value, "4294967295"));
    ASSERT_TRUE(str_is(&m.diag_values[1].value, "3") && str_is(&m.diag_values[2].value, "2"));
    ASSERT_TRUE(str_is(&m.diag_values[3].key, "time_sync_age_ms") && str_is(&m.diag_values[3].value, "1234"));
    for (unsigned i = 0; i < ROS_DIAG_VALUES; i++) ASSERT_TRUE(bound(&m.diag_values[i].value, ROS_DIAG_VALUE_MAX));
    ASSERT_TRUE(last_msg[ROS_ENT_PUB_DIAGNOSTICS] == &m.diagnostics);
}

int main(void) {
    RUN_TEST(test_reset_binds_every_string_to_owned_bounded_storage);
    RUN_TEST(test_inputs_reset_per_generation_keeps_bindings);
    RUN_TEST(test_twist_and_bounded_arm_reply);
    RUN_TEST(test_unsynced_time_publishes_nothing_but_odometry_integrates);
    RUN_TEST(test_synced_tick_stamps_from_acquisition_and_throttles);
    RUN_TEST(test_range_samples_between_due_ticks_are_not_lost_and_offline_event_once);
    RUN_TEST(test_unhealthy_imu_is_not_published_and_publish_failures_count);
    RUN_TEST(test_odometry_message_covariance_and_unknown_axes);
    RUN_TEST(test_diagnostics_summary_values_are_bounded_text);
    TEST_SUMMARY();
}
