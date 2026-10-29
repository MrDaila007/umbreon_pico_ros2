/* umb_gear.h - forward / brake / reverse gear state machine (pure C11).
 * Runs the ESC brake -> neutral -> reverse sequence so callers can send
 * signed speeds. Ported from umbreon_zephyr/src/control.c MNV_BACK_*. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_settings.h"

typedef enum {
    UMB_GEAR_NEUTRAL, UMB_GEAR_FORWARD, UMB_GEAR_WAIT_STOP,
    UMB_GEAR_BRAKE, UMB_GEAR_GAP, UMB_GEAR_REVERSE
} umb_gear_phase_t;

/* target_dir: direction the sequence is heading to (+1 forward, -1 reverse,
 * 0 emergency brake: ends in NEUTRAL). Meaningful in WAIT_STOP/BRAKE/GAP. */
typedef struct {
    umb_gear_phase_t phase;
    int8_t           target_dir;
    uint32_t         deadline_ms;
} umb_gear_t;

#define UMB_DIR_DEADBAND 0.05f        /* |request| below this = neutral request */
#define UMB_GEAR_STOP_SPEED 0.10f     /* m/s, "stopped" */
#define UMB_GEAR_WAIT_STOP_MS 1000u
#define UMB_GEAR_GAP_MS 80u

umb_gear_t umb_gear_init(void);
/* Advance the machine. Transitions whose condition already holds cascade
 * within one call; timed phases (BRAKE, GAP) advance only past their deadline.
 * Non-finite request is treated as 0. */
umb_gear_t umb_gear_step(umb_gear_t g, float request, float speed_abs, uint32_t now_ms,
                         const umb_settings_t *c);
int8_t umb_gear_drive_dir(umb_gear_t g);   /* +1 FORWARD, -1 REVERSE, 0 otherwise */
bool   umb_gear_braking(umb_gear_t g);     /* true in BRAKE: ESC gets reverse_brake_cmd */
/* Sign for tach speed (odometry): -1 in REVERSE and in WAIT_STOP heading
 * forward (still rolling backward); +1 otherwise. */
float  umb_gear_speed_sign(umb_gear_t g);
/* FORWARD -> BRAKE (target_dir 0): brake reverse_brake_ms, GAP, then NEUTRAL;
 * not cancelled by any request. Other phases unchanged. */
umb_gear_t umb_gear_emergency_brake(umb_gear_t g, uint32_t now_ms, const umb_settings_t *c);
