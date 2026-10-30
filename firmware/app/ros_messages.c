/* ros_messages.c - see ros_messages.h. No allocation; no generated message
 * functions (they would heap-allocate or free the bound static buffers). */
#include "ros_messages.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const char *const k_diag_keys[ROS_DIAG_VALUES] = {
    "generation", "sessions", "losses", "time_sync_age_ms"};

/* Bind a string to a caller-owned buffer; text is copied bounded and NUL-ended. */
static void bind(rosidl_runtime_c__String *s, char *buf, size_t cap, const char *text) {
    size_t n = 0;
    if (text) while (n + 1 < cap && text[n]) { buf[n] = text[n]; n++; }
    memset(buf + n, 0, cap - n);
    s->data = buf; s->size = n; s->capacity = cap;
}
#define BIND(str, buf, text) bind(&(str), (buf), sizeof(buf), (text))

static void reset_range(ros_messages_t *m, int i) {
    umb_range_sample_t s;
    umb_range_convert(i, UMB_TOF_INVALID_MM, 0, false, 0, &s); /* static profile + frame */
    sensor_msgs__msg__Range *r = &m->range[i];
    BIND(r->header.frame_id, m->text.range_frame[i], s.frame);
    r->radiation_type = s.radiation_type; r->field_of_view = s.field_of_view_rad;
    r->min_range = s.min_range_m; r->max_range = s.max_range_m;
    r->range = INFINITY; r->variance = 0.0f; /* 0 = variance unknown */
}

static void reset_battery(ros_messages_t *m) {
    sensor_msgs__msg__BatteryState *b = &m->battery;
    BIND(b->header.frame_id, m->text.battery_frame, "");
    BIND(b->location, m->text.battery_location, "");
    BIND(b->serial_number, m->text.battery_serial, "");
    b->voltage = b->temperature = b->current = b->charge = NAN;
    b->capacity = b->design_capacity = b->percentage = NAN;
    /* status/health/technology stay 0 = UNKNOWN; cell sequences stay empty. */
}

static void reset_diagnostics(ros_messages_t *m) {
    ros_message_text_t *x = &m->text;
    BIND(m->reflex.name, x->reflex_name, "");
    BIND(m->reflex.message, x->reflex_message, "");
    BIND(m->reflex.hardware_id, x->reflex_hwid, ROS_DIAG_HWID);
    BIND(m->diagnostics.header.frame_id, x->diag_frame, "");
    diagnostic_msgs__msg__DiagnosticStatus *st = &m->diag_status[0];
    BIND(st->name, x->diag_name, "umbreon/session");
    BIND(st->message, x->diag_message, "session ready");
    BIND(st->hardware_id, x->diag_hwid, ROS_DIAG_HWID);
    for (unsigned i = 0; i < ROS_DIAG_VALUES; i++) {
        BIND(m->diag_values[i].key, x->diag_key[i], k_diag_keys[i]);
        BIND(m->diag_values[i].value, x->diag_value[i], "");
    }
    st->values.data = m->diag_values; st->values.size = st->values.capacity = ROS_DIAG_VALUES;
    m->diagnostics.status.data = m->diag_status;
    m->diagnostics.status.size = m->diagnostics.status.capacity = 1;
}

void ros_messages_reset(ros_messages_t *m) {
    memset(m, 0, sizeof *m);
    for (int i = 0; i < UMB_TOF_COUNT; i++) reset_range(m, i);
    BIND(m->imu.header.frame_id, m->text.imu_frame, UMB_FRAME_IMU);
    m->imu.orientation.w = 1.0;
    m->imu.orientation_covariance[0] = -1.0; /* orientation not provided */
    BIND(m->odom.header.frame_id, m->text.odom_frame, UMB_FRAME_ODOM);
    BIND(m->odom.child_frame_id, m->text.odom_child, UMB_FRAME_BASE);
    m->odom.pose.pose.orientation.w = 1.0;
    reset_battery(m);
    reset_diagnostics(m);
    BIND(m->arm_response.message, m->text.arm_message, "");
    BIND(m->save_response.message, m->text.save_message, "");
}

void ros_messages_reset_inputs(ros_messages_t *m) {
    memset(m->cmd, 0, sizeof m->cmd);
    memset(&m->arm_request, 0, sizeof m->arm_request);
    m->arm_response.success = false;
    BIND(m->arm_response.message, m->text.arm_message, "");
    memset(&m->save_request, 0, sizeof m->save_request);
    m->save_response.success = false;
    BIND(m->save_response.message, m->text.save_message, "");
}

void ros_messages_twist(const geometry_msgs__msg__Twist *in, umb_twist_fields_t *out) {
    out->linear[0] = in->linear.x; out->linear[1] = in->linear.y; out->linear[2] = in->linear.z;
    out->angular[0] = in->angular.x; out->angular[1] = in->angular.y; out->angular[2] = in->angular.z;
}

void ros_messages_arm_reply(ros_messages_t *m, bool success, const char *message) {
    m->arm_response.success = success;
    BIND(m->arm_response.message, m->text.arm_message, message);
}

void ros_messages_save_reply(ros_messages_t *m, bool success, const char *message) {
    m->save_response.success = success;
    BIND(m->save_response.message, m->text.save_message, message);
}

/* ---- Telemetry ---------------------------------------------------------- */

void ros_telemetry_init(ros_telemetry_t *t) {
    memset(t, 0, sizeof *t);
    t->odom = umb_odom_init();
    t->sched = umb_sched_init(0);
}

void ros_telemetry_session_start(ros_telemetry_t *t, uint32_t now_ms) {
    t->sched = umb_sched_init(now_ms); /* no catch-up burst after a reconnect */
}

typedef struct {
    ros_telemetry_t *t;
    ros_messages_t *m;
    const umb_time_sync_t *sync;
    uint64_t now64;
    ros_publish_fn publish;
    void *ctx;
    unsigned failures;
} tick_ctx_t;

static bool stamp(tick_ctx_t *c, uint32_t acq_ms, builtin_interfaces__msg__Time *out) {
    uint64_t at64;
    umb_ros_stamp_t s;
    if (!umb_sensor_stamp64(c->now64, acq_ms, &at64) || !umb_time_stamp(c->sync, at64, &s)) {
        c->t->unstamped++;
        return false;
    }
    out->sec = s.sec; out->nanosec = s.nanosec;
    return true;
}

static void send(tick_ctx_t *c, ros_entity_id_t id, const void *msg) {
    if (c->publish(c->ctx, id, msg)) c->t->published++;
    else { c->t->publish_failures++; c->failures++; }
}

static void tick_ranges(tick_ctx_t *c, const shared_sensors_t *s) {
    uint32_t now = (uint32_t)c->now64;
    umb_range_sample_t sample[UMB_TOF_COUNT];
    umb_range_track_t probe[UMB_TOF_COUNT];
    bool fresh[UMB_TOF_COUNT], any = false;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        umb_range_convert(i, s->tof_mm[i], s->tof_stamp_ms[i], (s->tof_online_mask >> i) & 1u, now, &sample[i]);
        probe[i] = c->t->range[i]; /* evaluate on a copy: commit only when emitted */
        fresh[i] = umb_range_is_new(&probe[i], &sample[i]);
        any = any || fresh[i];
    }
    if (!umb_sched_take(&c->t->sched, UMB_STREAM_RANGE, now, any)) return;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        if (!fresh[i]) continue;
        c->t->range[i] = probe[i];
        sensor_msgs__msg__Range *r = &c->m->range[i];
        if (!stamp(c, sample[i].stamp_ms, &r->header.stamp)) continue;
        r->range = sample[i].range_m;
        send(c, (ros_entity_id_t)(ROS_ENT_PUB_RANGE_HR + i), r);
    }
}

static void tick_imu(tick_ctx_t *c, const shared_sensors_t *s) {
    umb_imu_sample_t sample;
    umb_imu_convert(s->accel_g, s->gyro_dps, s->imu_ok, s->imu_stamp_ms, &sample);
    umb_gen_t probe = c->t->imu;
    bool fresh = umb_gen_advanced(&probe, s->imu_stamp_ms) && sample.healthy;
    if (!umb_sched_take(&c->t->sched, UMB_STREAM_IMU, (uint32_t)c->now64, fresh)) return;
    c->t->imu = probe;
    sensor_msgs__msg__Imu *imu = &c->m->imu;
    if (!stamp(c, sample.stamp_ms, &imu->header.stamp)) return;
    imu->angular_velocity.x = sample.gyro_rps[0]; imu->angular_velocity.y = sample.gyro_rps[1];
    imu->angular_velocity.z = sample.gyro_rps[2];
    imu->linear_acceleration.x = sample.accel_mps2[0]; imu->linear_acceleration.y = sample.accel_mps2[1];
    imu->linear_acceleration.z = sample.accel_mps2[2];
    send(c, ROS_ENT_PUB_IMU, imu);
}

static void fill_odom_covariance(const umb_odom_t *o, nav_msgs__msg__Odometry *msg) {
    double *p = msg->pose.covariance, *v = msg->twist.covariance;
    p[0] = p[7] = o->cov_xy;
    p[14] = p[21] = p[28] = UMB_ODOM_COV_UNKNOWN; /* z, roll, pitch unobservable */
    p[35] = o->cov_yaw;
    v[0] = o->vx_valid ? o->cov_vx : UMB_ODOM_COV_UNKNOWN;
    v[7] = v[14] = v[21] = v[28] = UMB_ODOM_COV_UNKNOWN; /* vy, vz, wx, wy not measured */
    v[35] = o->wz_valid ? o->cov_wz : UMB_ODOM_COV_UNKNOWN;
}

static void tick_odom(tick_ctx_t *c) {
    if (!umb_sched_take(&c->t->sched, UMB_STREAM_ODOM, (uint32_t)c->now64, c->t->odom.seeded)) return;
    nav_msgs__msg__Odometry *msg = &c->m->odom;
    const umb_odom_t *o = &c->t->odom;
    if (!stamp(c, (uint32_t)c->now64, &msg->header.stamp)) return;
    float q[4];
    umb_odom_quat(o->yaw_rad, q);
    msg->pose.pose.position.x = o->x_m; msg->pose.pose.position.y = o->y_m;
    msg->pose.pose.orientation.z = q[2]; msg->pose.pose.orientation.w = q[3];
    msg->twist.twist.linear.x = o->vx_valid ? o->vx_mps : 0.0f;
    msg->twist.twist.angular.z = o->wz_valid ? o->wz_rps : 0.0f;
    fill_odom_covariance(o, msg);
    send(c, ROS_ENT_PUB_ODOM, msg);
}

static void tick_battery(tick_ctx_t *c, const shared_sensors_t *s) {
    uint32_t now = (uint32_t)c->now64;
    if (!umb_sched_take(&c->t->sched, UMB_STREAM_BATTERY, now, true)) return;
    umb_battery_sample_t b;
    umb_battery_convert(s->bat_v, s->bat_have, now, &b);
    sensor_msgs__msg__BatteryState *msg = &c->m->battery;
    if (!stamp(c, b.stamp_ms, &msg->header.stamp)) return;
    msg->present = b.present; msg->voltage = b.voltage_v;
    msg->current = b.current_a; msg->charge = b.charge_ah; msg->capacity = b.capacity_ah;
    msg->percentage = b.percentage;
    msg->power_supply_status = b.status; msg->power_supply_health = b.health;
    msg->power_supply_technology = b.technology;
    send(c, ROS_ENT_PUB_BATTERY, msg);
}

static void set_u32(ros_messages_t *m, unsigned i, uint32_t v) {
    char digits[11], *p = digits + sizeof digits - 1;
    *p = 0;
    do { *--p = (char)('0' + v % 10u); v /= 10u; } while (v);
    BIND(m->diag_values[i].value, m->text.diag_value[i], p);
}

static void tick_diagnostics(tick_ctx_t *c, const ros_diag_info_t *d) {
    uint32_t now = (uint32_t)c->now64;
    if (!umb_sched_take(&c->t->sched, UMB_STREAM_DIAG, now, true)) return;
    ros_messages_t *m = c->m;
    if (!stamp(c, now, &m->diagnostics.header.stamp)) return;
    m->diag_status[0].level = 0; /* OK: only published from a ready, synchronized session */
    set_u32(m, 0, d->generation); set_u32(m, 1, d->sessions);
    set_u32(m, 2, d->losses); set_u32(m, 3, d->sync_age_ms);
    send(c, ROS_ENT_PUB_DIAGNOSTICS, &m->diagnostics);
}

unsigned ros_telemetry_tick(ros_telemetry_t *t, ros_messages_t *m, const shared_sensors_t *s,
                            float speed_signed_mps, uint64_t now_ms64,
                            const umb_time_sync_t *sync, const ros_diag_info_t *diag,
                            ros_publish_fn publish, void *ctx) {
    uint32_t now = (uint32_t)now_ms64;
    umb_odom_update(&t->odom, now, speed_signed_mps, s->yaw_rate_dps, s->imu_ok);
    if (!sync->synced) return 0; /* never a fabricated epoch stamp */
    tick_ctx_t c = {t, m, sync, now_ms64, publish, ctx, 0};
    tick_imu(&c, s);
    tick_odom(&c);
    tick_ranges(&c, s);
    tick_battery(&c, s);
    tick_diagnostics(&c, diag);
    return c.failures;
}
