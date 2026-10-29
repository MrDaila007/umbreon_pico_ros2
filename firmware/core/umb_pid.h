/* umb_pid.h - closed-loop speed controller (pure C11, no SDK includes).
 * Port of car_pid_control() from umbreon_zephyr/src/car.c: EMA 0.7/0.3 on the
 * tach speed, setpoint slew, kick-off pulse and corner kick (forward only),
 * derivative on measurement, integral clamp +-50, feed-forward past the ESC
 * dead zone. Works on speed magnitude; the caller applies the direction. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_settings.h"

typedef struct {
    float filtered, prev_filtered, integral, ref_applied, kick_prev_ref;
    uint32_t prev_count, prev_ms, kick_until_ms;
    bool started;
} umb_pid_t;

/* Fresh controller with the tach baseline at tach_count. The first step
 * after a reset only starts the clock (speed 0, offset 0). */
umb_pid_t umb_pid_reset(uint32_t tach_count);

/* One control step. target_abs: setpoint magnitude in m/s (non-finite or
 * negative -> 0). dir: +1 forward, -1 reverse, 0 = measure only (no kick,
 * no launch floor). steer_abs: 0..1 for the corner kick. Steps closer than
 * 10 ms apart do not measure or integrate. Outputs the filtered speed
 * magnitude (m/s) and the ESC offset from neutral in us (>= 0). */
umb_pid_t umb_pid_step(umb_pid_t p, float target_abs, int8_t dir, float steer_abs,
                       uint32_t tach_count, uint32_t tach_age_us, uint32_t now_ms,
                       const umb_settings_t *c, float *speed_abs_out, int *offset_us_out);

/* Drop the integral, setpoint and kick state; keep the speed measurement
 * (filter, tach baseline, clock) so speed stays correct while not driving. */
umb_pid_t umb_pid_idle(umb_pid_t p);
