/* umb_actuation.h - steer/throttle to servo/ESC pulse width (pure C11).
 * Ported from umbreon_zephyr/src/car.c (car_write_steer / car_write_speed). */
#pragma once
#include <stdint.h>
#include "umb_settings.h"

#define UMB_ESC_NEUTRAL_US 1500

/* steer -1..1 -> servo pulse us; applies servo_reverse; non-finite -> centre. */
uint16_t umb_servo_us(float steer, const umb_settings_t *c);
/* Open-loop throttle -1..1 -> ESC us; applies max_throttle / max_throttle_rev;
 * non-finite -> neutral. Result clamped to 1000..2000. */
uint16_t umb_esc_us_from_throttle(float t, const umb_settings_t *c);
/* Most-reverse ESC value permitted by max_throttle_rev. */
uint16_t umb_esc_reverse_limit_us(const umb_settings_t *c);
