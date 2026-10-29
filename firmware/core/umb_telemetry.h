/*
 * umb_telemetry.h - pure SI telemetry conversion, ROS-time mapping and publish cadence.
 * No rcl/message headers: Task 9 copies these plain structs into the ROS messages.
 * Value semantics; the only mutable state lives in structs the caller owns.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "umb_tof_array.h"

/* ---- Range profile (VL53L0X) ------------------------------------------------
 * max: nominal 2.0 m long-range figure from the ST datasheet; ambient light, target
 *      reflectance and the high-speed profile can make the practical range shorter.
 *      The driver's 8190 mm cutoff is only an invalid sentinel boundary, NOT a range claim.
 * min: CHOSEN conservative application limit, not an ST guarantee. ST gives no universal
 *      minimum (it depends on optics / cover glass). Bench measurement is still owed:
 *      confirm the lowest distance each mounted sensor reads reliably.
 * fov: nominal 25 degree cone. Radiation type = INFRARED (sensor_msgs/Range value 1). */
#define UMB_RANGE_MIN_M 0.03f
#define UMB_RANGE_MAX_M 2.0f
#define UMB_RANGE_FOV_RAD 0.43633231f
#define UMB_RANGE_RADIATION_INFRARED 1u
#define UMB_FRAME_MAX 12
#define UMB_FRAME_IMU "imu_link"
#define UMB_FRAME_ODOM "odom"
#define UMB_FRAME_BASE "base_link"
#define UMB_GRAVITY_MPS2 9.80665f
#define UMB_DEG_TO_RAD 0.017453292519943295f

/* ---- Time mapping -----------------------------------------------------------
 * ROS time = monotonic ms * 1e6 + offset. Only stamps use it; safety, health ages and
 * odometry use the monotonic clock exclusively, so an offset change cannot touch them. */
#define UMB_FUTURE_TOLERANCE_MS 5u   /* sensor stamp ahead of now by <= this is clamped to now */

typedef struct {
    bool synced;
    int64_t offset_ns;
    uint32_t syncs, failed_syncs;
} umb_time_sync_t;

typedef struct { int32_t sec; uint32_t nanosec; } umb_ros_stamp_t;

typedef struct { uint64_t now64; uint32_t last32; bool seeded; } umb_mono_t;

umb_time_sync_t umb_time_sync_init(void);   /* unsynced: telemetry suppressed */
/* ros_epoch_ns: agent epoch sampled at mono_ms64. Rejects negative epochs (returns false,
 * counts a failed sync and leaves the previous mapping untouched). Re-syncing backwards
 * is accepted and only changes future stamps. */
bool umb_time_sync_set(umb_time_sync_t *t, int64_t ros_epoch_ns, uint64_t mono_ms64);
void umb_time_sync_failed(umb_time_sync_t *t);   /* failed resync keeps the last offset */
void umb_time_sync_invalidate(umb_time_sync_t *t); /* session lost: suppress until resynced */
/* false (stamp untouched) while unsynced or if the result would be negative. */
bool umb_time_stamp(const umb_time_sync_t *t, uint64_t mono_ms64, umb_ros_stamp_t *out);

/* Extend a wrapping uint32 ms clock to uint64; call at least every ~24 days. */
uint64_t umb_mono_extend(umb_mono_t *m, uint32_t now32);
/* Reconstruct a sensor's uint32 ms acquisition stamp against now64. A stamp a few ms in
 * the future is clamped to now; further ahead (or before boot) is rejected. */
bool umb_sensor_stamp64(uint64_t now64, uint32_t stamp32, uint64_t *out64);

/* ---- Sample conversion ------------------------------------------------------ */
typedef struct {
    char frame[UMB_FRAME_MAX];     /* "tof_<label>" */
    float range_m;                 /* finite, +inf (no return / above max / invalid), -inf (below min) */
    float min_range_m, max_range_m, field_of_view_rad;
    uint8_t radiation_type;
    bool acquired;                 /* backed by a fresh online acquisition */
    uint32_t stamp_ms;             /* acquisition stamp if acquired, else now_ms */
    uint32_t gen_key;              /* raw snapshot acquisition stamp (new-sample detection) */
} umb_range_sample_t;

void umb_range_convert(int index, int16_t mm, uint32_t acq_stamp_ms, bool online,
                       uint32_t now_ms, umb_range_sample_t *out);

typedef struct {
    char frame[UMB_FRAME_MAX];
    float accel_mps2[3], gyro_rps[3];
    float orientation_covariance0;  /* -1: orientation not provided (quaternion = identity) */
    bool healthy;                   /* false => do not publish as a fresh sample */
    uint32_t stamp_ms;
} umb_imu_sample_t;

/* Raw sensor axes, no rotation: gyro deg/s -> rad/s, accel g -> m/s^2. */
void umb_imu_convert(const float accel_g[3], const float gyro_dps[3], bool imu_ok,
                     uint32_t acq_stamp_ms, umb_imu_sample_t *out);

typedef struct {
    bool present;
    float voltage_v;                /* NaN when unknown */
    float current_a, charge_ah, capacity_ah, percentage;   /* always NaN: not measured */
    uint8_t status, health, technology;                    /* always 0 = UNKNOWN */
    uint32_t stamp_ms;              /* publish-time: the snapshot has no battery acquisition stamp */
} umb_battery_sample_t;

void umb_battery_convert(float bat_v, bool bat_have, uint32_t now_ms, umb_battery_sample_t *out);

/* ---- New-sample tracking -------------------------------------------------- */
typedef struct { uint32_t last; bool seen; } umb_gen_t;
/* True when the acquisition key differs from the last one accepted (updates it). */
bool umb_gen_advanced(umb_gen_t *g, uint32_t key);

typedef struct { uint32_t last_key; bool last_acquired; bool seen; } umb_range_track_t;
/* True for a new acquisition, or once when an acquired sensor drops to no-return. */
bool umb_range_is_new(umb_range_track_t *t, const umb_range_sample_t *s);

/* ---- Cadence ---------------------------------------------------------------- */
typedef enum { UMB_STREAM_IMU, UMB_STREAM_ODOM, UMB_STREAM_RANGE, UMB_STREAM_BATTERY,
               UMB_STREAM_DIAG, UMB_STREAM_COUNT } umb_stream_t;
#define UMB_PERIOD_IMU_MS 10u
#define UMB_PERIOD_ODOM_MS 20u
#define UMB_PERIOD_RANGE_MS 40u
#define UMB_PERIOD_BATTERY_MS 1000u
#define UMB_PERIOD_DIAG_MS 1000u

typedef struct { uint32_t next_due[UMB_STREAM_COUNT]; } umb_sched_t;

umb_sched_t umb_sched_init(uint32_t now_ms);   /* every stream due at now */
/* True when the stream is due AND has a new sample. Not-new leaves it due so the
 * sample goes out as soon as it appears. After emitting, the next due time advances by
 * one period, or restarts from now after a gap >= one period (never a catch-up burst). */
bool umb_sched_take(umb_sched_t *s, umb_stream_t stream, uint32_t now_ms, bool has_new);
