/* umb_control.h - the 50 Hz control step (pure C11, no SDK includes).
 * Ties arming, link safety, reflexes, gear and the speed PID together and
 * produces the servo / ESC pulse widths. Value semantics: the step takes the
 * state by value and returns the new state. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"
#include "umb_settings.h"
#include "umb_arming.h"
#include "umb_gear.h"
#include "umb_pid.h"
#include "umb_reflex.h"
#include "umb_safety.h"

typedef struct {
    umb_arming_t arming;
    umb_gear_t gear;
    umb_pid_t pid;
    umb_reflex_t reflex;
    uint32_t stop_mask;          /* previous step's stop bits */
    float last_speed_signed;     /* previous step's speed, m/s, + forward */
    int last_esc_offset_us;      /* previous step's DRIVE offset (0 unless FORWARD/REVERSE) */
} umb_control_t;

typedef struct {
    umb_cmd_t cmd;               /* valid when safety.have_cmd */
    umb_safety_in_t safety;
    umb_sensor_in_t sensors;
    uint32_t tach_count;
    uint32_t tach_age_us;
} umb_control_in_t;

typedef struct {
    uint16_t servo_us, esc_us;
    uint32_t stop_mask;
    bool armed;
    bool motion_allowed;         /* the car was allowed to move this step */
    float speed_signed;
    umb_gear_phase_t gear;
    umb_drive_mode_t mode;       /* command's drive mode; MANUAL without a command */
    umb_reflex_out_t reflex;
} umb_outputs_t;

/* Disarmed, gear NEUTRAL, tach baseline at tach_count. */
umb_control_t umb_control_init(const umb_settings_t *c, uint32_t tach_count);
umb_control_t umb_control_step(umb_control_t s, const umb_control_in_t *in,
                               const umb_settings_t *c, umb_outputs_t *out);
