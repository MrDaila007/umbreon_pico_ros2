/*
 * umb_odometry.h - dead-reckoned planar pose from MEASURED signed speed and the
 * bias-corrected IMU yaw rate. Monotonic ms only (uint32, wrap-safe); never ROS time.
 * Pose resets only on umb_odom_init (firmware boot) and survives agent reconnects.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UMB_ODOM_MAX_DT_MS 200u

/* Chosen conservative covariance diagonals (variance). Large = unobservable/unknown. */
#define UMB_ODOM_COV_XY 0.05f
#define UMB_ODOM_COV_YAW 0.05f
#define UMB_ODOM_COV_VX 0.02f
#define UMB_ODOM_COV_WZ 0.02f
#define UMB_ODOM_COV_XY_NO_IMU 1.0f   /* position along a frozen heading */
#define UMB_ODOM_COV_UNKNOWN 1e6f      /* yaw while the IMU is down; also z/roll/pitch */

typedef struct {
    float x_m, y_m;
    float yaw_rad;              /* normalised to (-pi, pi] */
    float vx_mps, wz_rps;       /* last measured, valid flags below */
    bool vx_valid, wz_valid;    /* wz_valid: IMU healthy and rate finite */
    bool yaw_valid;             /* false while the IMU is unhealthy: yaw frozen at last good */
    float cov_xy, cov_yaw, cov_vx, cov_wz;
    uint32_t last_ms;
    bool seeded;
    uint32_t skipped_intervals;
} umb_odom_t;

umb_odom_t umb_odom_init(void);   /* pose (0,0,0) */
/* Advance to now_ms. dt <= 0 or > UMB_ODOM_MAX_DT_MS (incl. the first call) re-seeds
 * the clock without integrating. Non-finite speed: no integration. IMU not ok: position
 * integrates along the last good yaw, yaw is unchanged. */
void umb_odom_update(umb_odom_t *o, uint32_t now_ms, float speed_signed_mps,
                     float yaw_rate_dps, bool imu_ok);
float umb_odom_normalize_yaw(float yaw_rad);
void umb_odom_quat(float yaw_rad, float q_xyzw[4]);
