#include "umb_odometry.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define TWO_PI_F (2.0f * PI_F)

umb_odom_t umb_odom_init(void) {
    umb_odom_t o;
    memset(&o, 0, sizeof o);
    o.yaw_valid = true;
    o.cov_xy = UMB_ODOM_COV_XY; o.cov_yaw = UMB_ODOM_COV_YAW;
    o.cov_vx = UMB_ODOM_COV_VX; o.cov_wz = UMB_ODOM_COV_WZ;
    return o;
}

float umb_odom_normalize_yaw(float yaw_rad) {
    if (!isfinite(yaw_rad)) return 0.0f;
    float a = fmodf(yaw_rad, TWO_PI_F);
    if (a > PI_F) a -= TWO_PI_F;
    else if (a <= -PI_F) a += TWO_PI_F;
    return a;
}

void umb_odom_quat(float yaw_rad, float q[4]) {
    q[0] = 0.0f; q[1] = 0.0f;
    q[2] = sinf(0.5f * yaw_rad);
    q[3] = cosf(0.5f * yaw_rad);
}

void umb_odom_update(umb_odom_t *o, uint32_t now_ms, float speed_signed_mps,
                     float yaw_rate_dps, bool imu_ok) {
    const bool speed_ok = isfinite(speed_signed_mps);
    const bool rate_ok = imu_ok && isfinite(yaw_rate_dps);
    const float wz = rate_ok ? yaw_rate_dps * 0.017453292519943295f : 0.0f;

    o->vx_valid = speed_ok;
    o->vx_mps = speed_ok ? speed_signed_mps : 0.0f;
    o->wz_valid = rate_ok;
    o->wz_rps = wz;
    o->yaw_valid = rate_ok;
    o->cov_xy = rate_ok ? UMB_ODOM_COV_XY : UMB_ODOM_COV_XY_NO_IMU;
    o->cov_yaw = rate_ok ? UMB_ODOM_COV_YAW : UMB_ODOM_COV_UNKNOWN;
    o->cov_vx = UMB_ODOM_COV_VX;
    o->cov_wz = UMB_ODOM_COV_WZ;

    if (!o->seeded) {
        o->seeded = true;
        o->last_ms = now_ms;
        return;
    }
    const int32_t dt_ms = (int32_t)(now_ms - o->last_ms);
    o->last_ms = now_ms;
    if (dt_ms <= 0 || dt_ms > (int32_t)UMB_ODOM_MAX_DT_MS) {
        o->skipped_intervals++;
        return;
    }
    const float dt = (float)dt_ms * 0.001f;
    const float yaw_mid = o->yaw_rad + 0.5f * wz * dt;
    if (speed_ok) {
        o->x_m += speed_signed_mps * cosf(yaw_mid) * dt;
        o->y_m += speed_signed_mps * sinf(yaw_mid) * dt;
    }
    if (rate_ok) o->yaw_rad = umb_odom_normalize_yaw(o->yaw_rad + wz * dt);
}
