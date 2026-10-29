/* umb_reflex.h - configurable safety reflexes (pure C11, no SDK includes).
 *
 * Each reflex has an action per driving mode (off/warn/pico/ros). Detection
 * runs unless the action is OFF. Only PICO produces effects in umb_reflex_out_t;
 * ROS sets delegated_mask only; WARN sets detected_mask only. The control step
 * (Task 10) applies the effects.
 *
 * Input conventions:
 *  - esc_offset_us is the DRIVE offset from neutral. The control step passes 0
 *    during BRAKE / GAP / WAIT_STOP so brake pulses never look like a stall.
 *  - A valid ToF reading is 0 < mm < 9999 (9999 is the driver's invalid sentinel).
 *    With no valid reading in collision_sensor_mask, collision and speed_cap are
 *    not detected and nearest_mm reports 9999.
 *  - All elapsed times use wrap-safe uint32_t subtraction.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"
#include "umb_settings.h"

typedef struct {
    bool tof_present; int16_t tof_mm[6]; uint8_t tof_online; bool tof_recovering;
    bool imu_present; bool imu_ok; float accel_g[3];
    bool bat_present; float battery_v;
} umb_sensor_in_t;   /* Plan 1 target: all *_present = false */

typedef struct {
    uint32_t stall_since_ms;
    uint32_t upside_since_ms;
    bool stall_tracking;     /* offset has been >= stall_min_us since stall_since_ms */
    bool upside_tracking;    /* Z < -0.5 g since upside_since_ms */
    bool stall_latched;
    uint32_t active_mask;    /* bit i = reflex i detected */
} umb_reflex_t;

typedef struct {
    uint32_t detected_mask, intervene_mask, delegated_mask; /* PICO / ROS actions */
    bool stop, disarm, emergency_brake, block_forward;
    float speed_cap_ms;    /* +inf = no cap */
    float throttle_cap;    /* 1.0 = no cap */
    float ttc_ms;          /* +inf when not moving forward or no valid reading */
    float nearest_mm;      /* 9999 when no valid reading */
} umb_reflex_out_t;

umb_reflex_t umb_reflex_init(void);
umb_reflex_t umb_reflex_step(umb_reflex_t r, umb_drive_mode_t mode, const umb_sensor_in_t *sens,
                             float speed_signed, int esc_offset_us, uint32_t tach_age_us,
                             uint32_t now_ms, const umb_settings_t *c, umb_reflex_out_t *out);
/* Key name as used in set_kv ("battery", "collision", ...); "unknown" if out of range. */
const char *umb_reflex_name(umb_reflex_id_t id);
