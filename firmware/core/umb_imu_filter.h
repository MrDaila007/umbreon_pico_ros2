/*
 * umb_imu_filter.h — gyro bias calibration, yaw-rate filter, heading and health.
 * Pure logic (no I/O); value semantics. Ported from umbreon_zephyr imu.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct { float accel_g[3]; float gyro_dps[3]; } umb_imu_raw_t;

typedef struct {
    float    gyro_bias_dps[3];
    float    yaw_rate_dps;
    float    heading_deg;
    uint32_t last_ok_ms;      /* time of last good sample */
    uint32_t last_sample_ms;  /* time base for dt; valid only if time_base_valid */
    bool     time_base_valid;
    bool     have_sample;
    bool     calibrated;
} umb_imu_state_t;

typedef struct { int n; float sum[3]; float min_z, max_z; } umb_imu_cal_t;

#define UMB_IMU_EMA 0.3f
#define UMB_IMU_DEADZONE_DPS 0.4f
#define UMB_IMU_CAL_SAMPLES 200
#define UMB_IMU_CAL_MAX_SPREAD_DPS 3.0f /* gyro Z spread above this = car moved -> reject */
#define UMB_IMU_STALE_MS 100

umb_imu_state_t umb_imu_init(void);

/* Accumulate one bias-calibration sample (start from a zeroed umb_imu_cal_t).
 * Non-finite samples are ignored. */
umb_imu_cal_t umb_imu_cal_add(umb_imu_cal_t c, const umb_imu_raw_t *s);

/* Store the mean gyro bias into st and set calibrated. Returns false (st unchanged)
 * if n < UMB_IMU_CAL_SAMPLES or the gyro Z spread exceeds the limit. */
bool umb_imu_cal_finish(const umb_imu_cal_t *c, umb_imu_state_t *st);

/* A failed read (read_ok false, NULL or non-finite sample) leaves the state unchanged
 * except the dt time base is dropped, so the next good sample does not integrate the gap. */
umb_imu_state_t umb_imu_update(umb_imu_state_t st, const umb_imu_raw_t *s, bool read_ok,
                               uint32_t now_ms, bool imu_rotate);

/* True when a good sample arrived within UMB_IMU_STALE_MS (future stamp counts as fresh). */
bool umb_imu_is_ok(const umb_imu_state_t *st, uint32_t now_ms);
