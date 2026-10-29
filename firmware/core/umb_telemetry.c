#include "umb_telemetry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---- Time ---- */
umb_time_sync_t umb_time_sync_init(void) {
    umb_time_sync_t t;
    memset(&t, 0, sizeof t);
    return t;
}

bool umb_time_sync_set(umb_time_sync_t *t, int64_t ros_epoch_ns, uint64_t mono_ms64) {
    if (ros_epoch_ns < 0) { t->failed_syncs++; return false; }
    t->offset_ns = ros_epoch_ns - (int64_t)(mono_ms64 * 1000000ull);
    t->synced = true;
    t->syncs++;
    return true;
}

void umb_time_sync_failed(umb_time_sync_t *t) { t->failed_syncs++; }
void umb_time_sync_invalidate(umb_time_sync_t *t) { t->synced = false; }

bool umb_time_stamp(const umb_time_sync_t *t, uint64_t mono_ms64, umb_ros_stamp_t *out) {
    if (!t->synced) return false;
    const int64_t ns = (int64_t)(mono_ms64 * 1000000ull) + t->offset_ns;
    if (ns < 0 || ns / 1000000000LL > INT32_MAX) return false;
    out->sec = (int32_t)(ns / 1000000000LL);
    out->nanosec = (uint32_t)(ns % 1000000000LL);
    return true;
}

uint64_t umb_mono_extend(umb_mono_t *m, uint32_t now32) {
    if (!m->seeded) {
        m->seeded = true;
        m->now64 = now32;
    } else {
        m->now64 += (uint32_t)(now32 - m->last32);
    }
    m->last32 = now32;
    return m->now64;
}

bool umb_sensor_stamp64(uint64_t now64, uint32_t stamp32, uint64_t *out64) {
    const uint32_t now32 = (uint32_t)now64;
    const uint32_t age = now32 - stamp32;
    if (age <= 0x7FFFFFFFu) {
        if (age > now64) return false;
        *out64 = now64 - age;
        return true;
    }
    if ((uint32_t)(stamp32 - now32) <= UMB_FUTURE_TOLERANCE_MS) {
        *out64 = now64;
        return true;
    }
    return false;
}

/* ---- Range ---- */
static const char *const k_range_frames[UMB_TOF_COUNT] = {
    "tof_hr", "tof_fr", "tof_r", "tof_l", "tof_fl", "tof_hl"};

void umb_range_convert(int index, int16_t mm, uint32_t acq_stamp_ms, bool online,
                       uint32_t now_ms, umb_range_sample_t *out) {
    memset(out, 0, sizeof *out);
    const bool idx_ok = index >= 0 && index < UMB_TOF_COUNT;
    snprintf(out->frame, sizeof out->frame, "%s", idx_ok ? k_range_frames[index] : "tof_invalid");
    out->min_range_m = UMB_RANGE_MIN_M;
    out->max_range_m = UMB_RANGE_MAX_M;
    out->field_of_view_rad = UMB_RANGE_FOV_RAD;
    out->radiation_type = UMB_RANGE_RADIATION_INFRARED;
    out->gen_key = acq_stamp_ms;
    out->range_m = INFINITY;
    out->stamp_ms = now_ms;
    const bool fresh = idx_ok && online &&
                       (int32_t)(now_ms - acq_stamp_ms) <= (int32_t)UMB_TOF_STALE_MS;
    if (!fresh) return;
    out->acquired = true;
    out->stamp_ms = acq_stamp_ms;
    if (mm <= 0 || mm >= UMB_TOF_OOR_MM || mm == UMB_TOF_INVALID_MM) return;  /* no return */
    const float m = (float)mm / 1000.0f;
    if (m < UMB_RANGE_MIN_M) out->range_m = -INFINITY;
    else if (m > UMB_RANGE_MAX_M) out->range_m = INFINITY;
    else out->range_m = m;
}

/* ---- IMU / battery ---- */
void umb_imu_convert(const float accel_g[3], const float gyro_dps[3], bool imu_ok,
                     uint32_t acq_stamp_ms, umb_imu_sample_t *out) {
    memset(out, 0, sizeof *out);
    snprintf(out->frame, sizeof out->frame, "%s", UMB_FRAME_IMU);
    out->orientation_covariance0 = -1.0f;
    out->stamp_ms = acq_stamp_ms;
    bool finite = imu_ok;
    for (int i = 0; i < 3; i++) {
        out->accel_mps2[i] = accel_g[i] * UMB_GRAVITY_MPS2;
        out->gyro_rps[i] = gyro_dps[i] * UMB_DEG_TO_RAD;
        finite = finite && isfinite(accel_g[i]) && isfinite(gyro_dps[i]) &&
                 isfinite(out->accel_mps2[i]) && isfinite(out->gyro_rps[i]);
    }
    out->healthy = finite;
}

void umb_battery_convert(float bat_v, bool bat_have, uint32_t now_ms, umb_battery_sample_t *out) {
    memset(out, 0, sizeof *out);
    out->present = bat_have && isfinite(bat_v) && bat_v >= 0.0f;
    out->voltage_v = out->present ? bat_v : NAN;
    out->current_a = out->charge_ah = out->capacity_ah = out->percentage = NAN;
    out->stamp_ms = now_ms;
}

/* ---- New-sample tracking ---- */
bool umb_gen_advanced(umb_gen_t *g, uint32_t key) {
    if (g->seen && g->last == key) return false;
    g->seen = true;
    g->last = key;
    return true;
}

bool umb_range_is_new(umb_range_track_t *t, const umb_range_sample_t *s) {
    bool is_new;
    if (!t->seen) is_new = true;
    else if (s->acquired) is_new = s->gen_key != t->last_key || !t->last_acquired;
    else is_new = t->last_acquired;
    t->seen = true;
    t->last_key = s->gen_key;
    t->last_acquired = s->acquired;
    return is_new;
}

/* ---- Cadence ---- */
static const uint32_t k_periods[UMB_STREAM_COUNT] = {
    UMB_PERIOD_IMU_MS, UMB_PERIOD_ODOM_MS, UMB_PERIOD_RANGE_MS,
    UMB_PERIOD_BATTERY_MS, UMB_PERIOD_DIAG_MS};

umb_sched_t umb_sched_init(uint32_t now_ms) {
    umb_sched_t s;
    for (int i = 0; i < UMB_STREAM_COUNT; i++) s.next_due[i] = now_ms;
    return s;
}

bool umb_sched_take(umb_sched_t *s, umb_stream_t stream, uint32_t now_ms, bool has_new) {
    if ((unsigned)stream >= (unsigned)UMB_STREAM_COUNT || !has_new) return false;
    const uint32_t due = s->next_due[stream];
    if ((int32_t)(now_ms - due) < 0) return false;
    const uint32_t period = k_periods[stream];
    s->next_due[stream] = (uint32_t)(now_ms - due) >= period ? now_ms + period : due + period;
    return true;
}
