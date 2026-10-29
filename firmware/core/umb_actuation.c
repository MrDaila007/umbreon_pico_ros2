#include "umb_actuation.h"
#include <math.h>

#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

static int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t servo_us_from_angle(int angle)
{
    return (uint16_t)(700 + angle * 1600 / 180);
}

uint16_t umb_servo_us(float steer, const umb_settings_t *c)
{
    if (!isfinite(steer)) {
        return servo_us_from_angle(c->neutral_point);
    }
    steer = clampf(steer, -1.0f, 1.0f);
    if (c->servo_reverse) {
        steer = -steer;
    }
    int s = (int)lroundf(steer * 1000.0f);
    int angle = (s < 0) ? map_val(s, -1000, 0, c->min_point, c->neutral_point)
                        : map_val(s, 0, 1000, c->neutral_point, c->max_point);
    return servo_us_from_angle(angle);
}

uint16_t umb_esc_us_from_throttle(float t, const umb_settings_t *c)
{
    if (!isfinite(t)) {
        return UMB_ESC_NEUTRAL_US;
    }
    t = clampf(t, -1.0f, 1.0f);
    if (t > 0.0f) {
        t = clampf(t, 0.0f, c->max_throttle);
    } else if (t < 0.0f) {
        t = clampf(t, -c->max_throttle_rev, 0.0f);
    }
    int s = (int)lroundf(t * 1000.0f);
    int us;
    if (s > 0) {
        us = map_val(s, 1, 1000, c->min_speed, c->max_speed);
    } else if (s < 0) {
        us = map_val(s, -1000, -1, ESC_MIN_US, c->min_bspeed);
    } else {
        us = UMB_ESC_NEUTRAL_US;
    }
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;
    return (uint16_t)us;
}

uint16_t umb_esc_reverse_limit_us(const umb_settings_t *c)
{
    return umb_esc_us_from_throttle(-c->max_throttle_rev, c);
}
