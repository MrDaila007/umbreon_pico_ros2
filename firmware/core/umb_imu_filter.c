#include "umb_imu_filter.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MAX_DT_S 0.5f

static bool raw_finite(const umb_imu_raw_t *s) {
    for (int i = 0; i < 3; i++) {
        if (!isfinite(s->accel_g[i]) || !isfinite(s->gyro_dps[i])) return false;
    }
    return true;
}

umb_imu_state_t umb_imu_init(void) {
    umb_imu_state_t st;
    memset(&st, 0, sizeof st);
    return st;
}

umb_imu_cal_t umb_imu_cal_add(umb_imu_cal_t c, const umb_imu_raw_t *s) {
    if (s == NULL || !raw_finite(s)) return c;
    float z = s->gyro_dps[2];
    if (c.n == 0) { c.min_z = z; c.max_z = z; }
    else {
        if (z < c.min_z) c.min_z = z;
        if (z > c.max_z) c.max_z = z;
    }
    for (int i = 0; i < 3; i++) c.sum[i] += s->gyro_dps[i];
    c.n++;
    return c;
}

bool umb_imu_cal_finish(const umb_imu_cal_t *c, umb_imu_state_t *st) {
    if (c == NULL || st == NULL) return false;
    if (c->n < UMB_IMU_CAL_SAMPLES) return false;
    if (c->max_z - c->min_z > UMB_IMU_CAL_MAX_SPREAD_DPS) return false;
    for (int i = 0; i < 3; i++) st->gyro_bias_dps[i] = c->sum[i] / (float)c->n;
    st->calibrated = true;
    return true;
}

umb_imu_state_t umb_imu_update(umb_imu_state_t st, const umb_imu_raw_t *s, bool read_ok,
                               uint32_t now_ms, bool imu_rotate) {
    if (!read_ok || s == NULL || !raw_finite(s)) {
        st.time_base_valid = false;
        return st;
    }

    float rate = s->gyro_dps[2] - st.gyro_bias_dps[2];
    if (imu_rotate) rate = -rate;
    if (fabsf(rate) < UMB_IMU_DEADZONE_DPS) rate = 0.0f;
    st.yaw_rate_dps = UMB_IMU_EMA * rate + (1.0f - UMB_IMU_EMA) * st.yaw_rate_dps;

    if (st.time_base_valid) {
        float dt = (float)(uint32_t)(now_ms - st.last_sample_ms) / 1000.0f;
        if (dt > 0.0f && dt < MAX_DT_S) {
            st.heading_deg = fmodf(st.heading_deg + st.yaw_rate_dps * dt, 360.0f);
        }
    }
    st.last_sample_ms = now_ms;
    st.time_base_valid = true;
    st.last_ok_ms = now_ms;
    st.have_sample = true;
    return st;
}

bool umb_imu_is_ok(const umb_imu_state_t *st, uint32_t now_ms) {
    if (st == NULL || !st->have_sample) return false;
    uint32_t age = now_ms - st->last_ok_ms;
    if ((int32_t)age < 0) return true; /* stamp from the future: fresh */
    return age <= UMB_IMU_STALE_MS;
}
