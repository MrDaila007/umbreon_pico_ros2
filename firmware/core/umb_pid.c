/* umb_pid.c - speed PID, see umb_pid.h. Constants from car_pid_control(). */
#include "umb_pid.h"
#include <math.h>
#include "umb_actuation.h"   /* UMB_ESC_NEUTRAL_US */

#define PID_PI            3.14159265f
#define EMA_NEW           0.7f
#define EMA_OLD           0.3f
#define TACH_STALE_US     500000u
#define MIN_DT_S          0.01f
#define INTEGRAL_LIMIT    50.0f
#define WANT_REF_MS       0.02f    /* setpoint above this = "wants to move" */
#define FF_REF_MS         0.01f    /* setpoint above this gets the feed-forward */
#define KICK_START_MS     0.10f    /* kick only from (near) rest */
#define KICK_END_REF_MS   0.05f
#define KICK_END_RATIO    0.78f    /* kick ends once speed reaches 78 % of setpoint */
#define KICK_MIN_PCT      0.05f
#define KICK_MAX_US       60.0f
#define LAUNCH_SPEED_MS   0.12f    /* below this the launch floor / corner kick apply */
#define OUTPUT_LIMIT_US   2000.0f  /* keeps the float -> int conversion defined */

typedef struct { float error, deriv; } pid_terms_t;

umb_pid_t umb_pid_reset(uint32_t tach_count) {
    umb_pid_t p = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, tach_count, 0u, 0u, false };
    return p;
}

umb_pid_t umb_pid_idle(umb_pid_t p) {
    p.integral = 0.0f;
    p.ref_applied = 0.0f;
    p.kick_prev_ref = 0.0f;
    p.kick_until_ms = 0u;
    return p;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float magnitude_or_zero(float v, float hi) {
    return (isfinite(v) && v > 0.0f) ? (v < hi ? v : hi) : 0.0f;
}

static float slew(float ref, float target, float dt, const umb_settings_t *c) {
    if (!(c->spd_slew > 0.001f)) return target;
    float step = c->spd_slew * dt;
    if (target > ref + step) return ref + step;
    if (target < ref - step) return ref - step;
    return target;
}

static bool kick_active(umb_pid_t p, uint32_t now_ms) {
    return p.kick_until_ms != 0u && (int32_t)(now_ms - p.kick_until_ms) < 0;
}

static bool kick_enabled(const umb_settings_t *c) {
    return c->kick_pct > KICK_MIN_PCT && c->kick_ms > 0;
}

/* Kick-off pulse window: armed on a forward edge from rest, ended once the
 * speed approaches the setpoint or the setpoint drops. Forward only. */
static umb_pid_t update_kick(umb_pid_t p, int8_t dir, uint32_t now_ms, const umb_settings_t *c) {
    bool want = dir > 0 && p.ref_applied > WANT_REF_MS;
    bool had = p.kick_prev_ref > WANT_REF_MS;
    if (!want) {
        p.kick_until_ms = 0u;
    } else if (!had && p.filtered < KICK_START_MS && kick_enabled(c)) {
        uint32_t until = now_ms + (uint32_t)c->kick_ms;
        p.kick_until_ms = (until != 0u) ? until : 1u;   /* 0 means "no kick" */
    }
    if (want && p.ref_applied > KICK_END_REF_MS && p.filtered >= p.ref_applied * KICK_END_RATIO)
        p.kick_until_ms = 0u;
    p.kick_prev_ref = p.ref_applied;
    return p;
}

static int output_us(umb_pid_t p, pid_terms_t t, int8_t dir, float steer_abs, uint32_t now_ms,
                     const umb_settings_t *c) {
    int dead = (dir < 0) ? UMB_ESC_NEUTRAL_US - c->min_bspeed : c->min_speed - UMB_ESC_NEUTRAL_US;
    if (dead < 0) dead = 0;
    float ff = (p.ref_applied > FF_REF_MS) ? (float)dead : 0.0f;
    float out = ff + c->pid_kp * t.error + c->pid_ki * p.integral + c->pid_kd * t.deriv;

    bool moving = dir != 0 && p.ref_applied > WANT_REF_MS;
    bool forward = moving && dir > 0;
    bool launching = moving && p.filtered < LAUNCH_SPEED_MS;
    float kick_us = 0.0f;
    int corner_us = 0;
    if (forward && kick_enabled(c) && kick_active(p, now_ms)) {
        int span = c->max_speed - UMB_ESC_NEUTRAL_US;
        kick_us = (c->kick_pct / 100.0f) * (float)(span < 1 ? 1 : span);
        if (kick_us > KICK_MAX_US) kick_us = KICK_MAX_US;
    }
    if (forward && launching) corner_us = (int)(steer_abs * (float)c->corner_kick_us);

    int off = (int)clampf(out + kick_us, -OUTPUT_LIMIT_US, OUTPUT_LIMIT_US) + corner_us;
    if (launching) {
        int floor_us = dead + (int)kick_us + corner_us;
        if (off < floor_us) off = floor_us;
    }
    return off < 0 ? 0 : off;
}

umb_pid_t umb_pid_step(umb_pid_t p, float target_abs, int8_t dir, float steer_abs,
                       uint32_t tach_count, uint32_t tach_age_us, uint32_t now_ms,
                       const umb_settings_t *c, float *speed_abs_out, int *offset_us_out) {
    float target = magnitude_or_zero(target_abs, INFINITY);
    float steer = magnitude_or_zero(steer_abs, 1.0f);
    pid_terms_t t = { p.ref_applied - p.filtered, 0.0f };

    if (!p.started) {
        p.started = true;
        p.prev_ms = now_ms;
        p.prev_count = tach_count;
        *speed_abs_out = p.filtered;
        *offset_us_out = 0;
        return p;
    }
    float dt = (float)(uint32_t)(now_ms - p.prev_ms) / 1000.0f;
    if (dt < MIN_DT_S) {   /* too soon: hold, no measurement or integration */
        *speed_abs_out = p.filtered;
        *offset_us_out = output_us(p, t, dir, steer, now_ms, c);
        return p;
    }
    p.prev_ms = now_ms;

    uint32_t delta = tach_count - p.prev_count;   /* wrap-safe */
    p.prev_count = tach_count;
    float raw = 0.0f;
    if (c->encoder_holes > 0)
        raw = ((float)delta / (float)c->encoder_holes) * (PID_PI * c->wheel_diam_m) / dt;
    p.filtered = EMA_NEW * raw + EMA_OLD * p.filtered;
    if (tach_age_us > TACH_STALE_US || !isfinite(p.filtered)) p.filtered = 0.0f;

    p.ref_applied = slew(p.ref_applied, target, dt, c);
    p = update_kick(p, dir, now_ms, c);

    t.error = p.ref_applied - p.filtered;
    p.integral = clampf(p.integral + t.error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    t.deriv = -(p.filtered - p.prev_filtered) / dt;
    p.prev_filtered = p.filtered;

    *speed_abs_out = p.filtered;
    *offset_us_out = output_us(p, t, dir, steer, now_ms, c);
    return p;
}
