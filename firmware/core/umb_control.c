/* umb_control.c - the control step, see umb_control.h. */
#include "umb_control.h"
#include <math.h>
#include <stdlib.h>
#include "umb_actuation.h"

typedef struct {
    uint16_t esc_us;
    float speed_abs;
} drive_t;

umb_control_t umb_control_init(const umb_settings_t *c, uint32_t tach_count) {
    umb_control_t s;
    s.arming = umb_arming_init((umb_arm_mode_t)c->arming_mode);
    s.gear = umb_gear_init();
    s.pid = umb_pid_reset(tach_count);
    s.reflex = umb_reflex_init();
    s.stop_mask = 0u;
    s.last_speed_signed = 0.0f;
    s.last_esc_offset_us = 0;
    return s;
}

static float finite_or_zero(float v) {
    return isfinite(v) ? v : 0.0f;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Same test umb_arming_on_command uses to set neutral_seen. */
static bool cmd_is_neutral(const umb_cmd_t *cmd) {
    return cmd->kind != UMB_CMD_NONE && isfinite(cmd->value) && fabsf(cmd->value) < UMB_DIR_DEADBAND;
}

static int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t brake_us(const umb_settings_t *c) {
    return umb_esc_us_from_throttle((float)c->reverse_brake_cmd / 1000.0f, c);
}

/* Run the PID for measurement only (output ignored) and drop its drive state. */
static umb_pid_t pid_measure(umb_pid_t p, const umb_control_in_t *in, const umb_settings_t *c,
                             float *speed_abs) {
    int unused;
    p = umb_pid_step(p, 0.0f, 0, 0.0f, in->tach_count, in->tach_age_us, in->safety.now_ms, c,
                     speed_abs, &unused);
    return umb_pid_idle(p);
}

/* SPEED: closed loop on |target| in FORWARD / REVERSE. */
static drive_t drive_speed(umb_control_t *s, float target, float steer, const umb_control_in_t *in,
                           const umb_settings_t *c) {
    drive_t d = { UMB_ESC_NEUTRAL_US, 0.0f };
    int8_t dir = umb_gear_drive_dir(s->gear);
    if (dir == 0) {
        s->pid = pid_measure(s->pid, in, c, &d.speed_abs);
        if (umb_gear_braking(s->gear)) d.esc_us = brake_us(c);
        return d;
    }
    int off = 0;
    s->pid = umb_pid_step(s->pid, fabsf(target), dir, fabsf(steer), in->tach_count,
                          in->tach_age_us, in->safety.now_ms, c, &d.speed_abs, &off);
    int us = (dir > 0)
        ? clamp_int(UMB_ESC_NEUTRAL_US + off, UMB_ESC_NEUTRAL_US, c->max_speed)
        : clamp_int(UMB_ESC_NEUTRAL_US - off, umb_esc_reverse_limit_us(c), UMB_ESC_NEUTRAL_US);
    d.esc_us = (uint16_t)us;
    return d;
}

/* RAW: open-loop throttle in FORWARD / REVERSE. */
static drive_t drive_raw(umb_control_t *s, float throttle, const umb_control_in_t *in,
                         const umb_settings_t *c) {
    drive_t d = { UMB_ESC_NEUTRAL_US, 0.0f };
    s->pid = pid_measure(s->pid, in, c, &d.speed_abs);
    if (umb_gear_drive_dir(s->gear) != 0) d.esc_us = umb_esc_us_from_throttle(throttle, c);
    else if (umb_gear_braking(s->gear)) d.esc_us = brake_us(c);
    return d;
}

/* Motion allowed: apply reflex effects, step the gear, compute the ESC. */
static drive_t drive(umb_control_t *s, const umb_control_in_t *in, const umb_reflex_out_t *rx,
                     const umb_settings_t *c) {
    uint32_t now = in->safety.now_ms;
    float value = finite_or_zero(in->cmd.value);
    float steer = finite_or_zero(in->cmd.steer);
    float gear_speed = fabsf(s->last_speed_signed);   /* previous step: this step measures after the gear */

    if (rx->emergency_brake) s->gear = umb_gear_emergency_brake(s->gear, now, c);
    if (rx->block_forward && value > 0.0f) value = 0.0f;   /* before the gear sees it */

    if (in->cmd.kind == UMB_CMD_SPEED) {
        float target = clampf(value, -c->max_reverse_speed_ms, c->max_speed_ms);
        target = clampf(target, -rx->speed_cap_ms, rx->speed_cap_ms);
        s->gear = umb_gear_step(s->gear, target, gear_speed, now, c);
        return drive_speed(s, target, steer, in, c);
    }
    float throttle = (in->cmd.kind == UMB_CMD_RAW) ? clampf(value, -1.0f, 1.0f) : 0.0f;
    throttle = clampf(throttle, -rx->throttle_cap, rx->throttle_cap);
    s->gear = umb_gear_step(s->gear, throttle, gear_speed, now, c);
    return drive_raw(s, throttle, in, c);
}

/* Steps 1-4: stop bits, disarm, neutral latch, command to arming. */
static umb_control_t update_safety(umb_control_t s, const umb_control_in_t *in, umb_drive_mode_t mode,
                                   const umb_settings_t *c, uint32_t *mask_out, umb_reflex_out_t *rx) {
    bool have = in->safety.have_cmd;
    uint32_t mask = umb_safety_evaluate(&in->safety, c);

    /* While stalled, keep the latch until a neutral command (no stop / retry cycling). */
    bool hold_stall = s.reflex.stall_latched && have && !cmd_is_neutral(&in->cmd);
    int rx_offset = hold_stall ? (int)c->stall_min_us : s.last_esc_offset_us;
    s.reflex = umb_reflex_step(s.reflex, mode, &in->sensors, s.last_speed_signed, rx_offset,
                               in->tach_age_us, in->safety.now_ms, c, rx);
    if (rx->stop) mask |= UMB_STOP_REFLEX;

    uint32_t newly = mask & ~s.stop_mask;
    if ((newly & UMB_STOP_DISARM_MASK) || rx->disarm) s.arming = umb_arming_disarm(s.arming);
    if (mask != 0u) {
        /* A stopped car needs a fresh neutral command; a command received
         * during (or before) the stop must not count as that neutral. */
        s.arming.neutral_seen = false;
    } else if (have) {
        s.arming = umb_arming_on_command(s.arming, &in->cmd);
    }
    s.stop_mask = mask;
    *mask_out = mask;
    return s;
}

umb_control_t umb_control_step(umb_control_t s, const umb_control_in_t *in,
                               const umb_settings_t *c, umb_outputs_t *out) {
    if ((umb_arm_mode_t)c->arming_mode != s.arming.mode)
        s.arming = umb_arming_set_mode(s.arming, (umb_arm_mode_t)c->arming_mode);

    umb_drive_mode_t mode = in->safety.have_cmd ? in->cmd.mode : UMB_DRIVE_MANUAL;
    umb_reflex_out_t rx;
    uint32_t mask;
    s = update_safety(s, in, mode, c, &mask, &rx);

    bool motion = in->safety.have_cmd && mask == 0u && umb_arming_motion_allowed(s.arming);
    drive_t d = { UMB_ESC_NEUTRAL_US, 0.0f };
    float steer = 0.0f;
    if (motion) {
        d = drive(&s, in, &rx, c);
        steer = finite_or_zero(in->cmd.steer);
    } else {
        s.gear = umb_gear_init();
        s.pid = pid_measure(s.pid, in, c, &d.speed_abs);
    }

    s.last_esc_offset_us = (umb_gear_drive_dir(s.gear) != 0) ? abs((int)d.esc_us - UMB_ESC_NEUTRAL_US) : 0;
    s.last_speed_signed = d.speed_abs * umb_gear_speed_sign(s.gear);

    out->servo_us = umb_servo_us(steer, c);
    out->esc_us = d.esc_us;
    out->stop_mask = mask;
    out->armed = umb_arming_is_armed(s.arming);
    out->motion_allowed = motion;
    out->speed_signed = s.last_speed_signed;
    out->gear = s.gear.phase;
    out->mode = mode;
    out->reflex = rx;
    return s;
}
