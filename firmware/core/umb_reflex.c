/* umb_reflex.c - configurable safety reflexes (pure C11) */
#include "umb_reflex.h"
#include <math.h>

#define NO_READING_MM   9999.0f
#define TOF_COUNT       6
#define MOVING_MS       0.05f     /* forward speed above this counts as moving */
#define UPSIDE_Z_G      (-0.5f)
#define UPSIDE_HOLD_MS  500u

typedef struct {
    float nearest_mm, ttc_ms;
    bool have_reading, moving_fwd;
} dist_t;

static const char *const NAMES[UMB_REFLEX_COUNT] = {
    "battery", "collision", "speed_cap", "stall", "impact", "sensor_fault"
};

const char *umb_reflex_name(umb_reflex_id_t id) {
    return ((unsigned)id < (unsigned)UMB_REFLEX_COUNT) ? NAMES[id] : "unknown";
}

umb_reflex_t umb_reflex_init(void) {
    umb_reflex_t r = {0u, 0u, false, false, false, 0u};
    return r;
}

static umb_reflex_action_t action_for(umb_drive_mode_t mode, umb_reflex_id_t id,
                                      const umb_settings_t *c) {
    uint8_t a = (mode == UMB_DRIVE_AUTO) ? c->reflex_auto[id] : c->reflex_manual[id];
    return (a <= UMB_RX_ROS) ? (umb_reflex_action_t)a : UMB_RX_OFF;
}

static dist_t measure(const umb_sensor_in_t *s, float speed, const umb_settings_t *c) {
    dist_t d = { NO_READING_MM, INFINITY, false, speed > MOVING_MS };
    if (s->tof_present) {
        for (int i = 0; i < TOF_COUNT; i++) {
            int16_t mm = s->tof_mm[i];
            if (!(c->collision_sensor_mask & (1u << i)) || mm <= 0 || mm >= 9999) continue;
            if (!d.have_reading || (float)mm < d.nearest_mm) d.nearest_mm = (float)mm;
            d.have_reading = true;
        }
    }
    if (d.have_reading && d.moving_fwd) d.ttc_ms = d.nearest_mm / (speed * 1000.0f) * 1000.0f;
    return d;
}

static float distance_cap(const dist_t *d, const umb_settings_t *c) {
    if (!d->have_reading) return INFINITY;
    float far = (float)c->cap_far_mm, near = (float)c->cap_near_mm;
    if (d->nearest_mm >= far) return c->max_speed_ms;
    if (d->nearest_mm <= near || far <= near) return c->cap_min_ms;
    float t = (d->nearest_mm - near) / (far - near);
    return c->cap_min_ms + t * (c->max_speed_ms - c->cap_min_ms);
}

static void apply_cap(umb_reflex_out_t *o, float cap, const umb_settings_t *c) {
    if (cap < o->speed_cap_ms) o->speed_cap_ms = cap;
    float tc = (c->max_speed_ms > 0.0f) ? cap / c->max_speed_ms : 1.0f;
    if (tc < 0.0f) tc = 0.0f;
    if (tc < o->throttle_cap) o->throttle_cap = tc;
}

/* Stall: drive offset held >= stall_min_us for stall_ms AND tach silent for stall_ms. */
static umb_reflex_t stall_update(umb_reflex_t r, int offset_us, uint32_t tach_age_us,
                                 uint32_t now_ms, const umb_settings_t *c, bool *detected) {
    int mag = offset_us < 0 ? -offset_us : offset_us;
    if (mag < (int)c->stall_min_us) {
        r.stall_tracking = false;
        r.stall_latched = false;
    } else {
        if (!r.stall_tracking) { r.stall_tracking = true; r.stall_since_ms = now_ms; }
        if (!r.stall_latched && (uint32_t)(now_ms - r.stall_since_ms) >= c->stall_ms &&
            tach_age_us >= (uint32_t)c->stall_ms * 1000u)
            r.stall_latched = true;
    }
    *detected = r.stall_latched;
    return r;
}

static umb_reflex_t impact_update(umb_reflex_t r, const umb_sensor_in_t *s, uint32_t now_ms,
                                  const umb_settings_t *c, bool *detected) {
    *detected = false;
    if (!(s->imu_present && s->imu_ok)) { r.upside_tracking = false; return r; }
    float ax = s->accel_g[0], ay = s->accel_g[1], az = s->accel_g[2];
    bool hit = sqrtf(ax * ax + ay * ay + az * az) > c->impact_g;
    if (az < UPSIDE_Z_G) {
        if (!r.upside_tracking) { r.upside_tracking = true; r.upside_since_ms = now_ms; }
    } else {
        r.upside_tracking = false;
    }
    bool upside = r.upside_tracking && (uint32_t)(now_ms - r.upside_since_ms) >= UPSIDE_HOLD_MS;
    *detected = hit || upside;
    return r;
}

umb_reflex_t umb_reflex_step(umb_reflex_t r, umb_drive_mode_t mode, const umb_sensor_in_t *sens,
                             float speed_signed, int esc_offset_us, uint32_t tach_age_us,
                             uint32_t now_ms, const umb_settings_t *c, umb_reflex_out_t *out) {
    umb_reflex_out_t o = { 0u, 0u, 0u, false, false, false, false,
                           INFINITY, 1.0f, INFINITY, NO_READING_MM };
    umb_reflex_action_t act[UMB_REFLEX_COUNT];
    for (int i = 0; i < UMB_REFLEX_COUNT; i++) act[i] = action_for(mode, (umb_reflex_id_t)i, c);

    dist_t d = measure(sens, speed_signed, c);
    o.nearest_mm = d.nearest_mm;
    o.ttc_ms = d.ttc_ms;

    bool det[UMB_REFLEX_COUNT] = { false };
    bool brake = false, block = false, tof_fault = false;

    det[UMB_REFLEX_BATTERY] = sens->bat_present && c->bat_enabled && !(sens->battery_v >= c->bat_low);

    bool near_front = d.have_reading && d.nearest_mm < (float)c->min_front_mm;
    if (d.have_reading) {
        if (d.moving_fwd) {
            brake = d.ttc_ms < (float)c->ttc_ms || near_front;
            block = brake;
        } else if (speed_signed <= MOVING_MS) {
            block = near_front;
        }
        det[UMB_REFLEX_COLLISION] = block;
    }

    float cap = distance_cap(&d, c);
    det[UMB_REFLEX_SPEED_CAP] = d.have_reading && cap < c->max_speed_ms;

    if (act[UMB_REFLEX_STALL] == UMB_RX_OFF) { r.stall_tracking = false; r.stall_latched = false; }
    else r = stall_update(r, esc_offset_us, tach_age_us, now_ms, c, &det[UMB_REFLEX_STALL]);

    if (act[UMB_REFLEX_IMPACT] == UMB_RX_OFF) r.upside_tracking = false;
    else r = impact_update(r, sens, now_ms, c, &det[UMB_REFLEX_IMPACT]);

    tof_fault = sens->tof_present && (sens->tof_online < TOF_COUNT || sens->tof_recovering);
    det[UMB_REFLEX_SENSOR_FAULT] = tof_fault || (sens->imu_present && !sens->imu_ok);

    for (int i = 0; i < UMB_REFLEX_COUNT; i++) {
        if (act[i] == UMB_RX_OFF || !det[i]) continue;
        uint32_t bit = 1u << i;
        o.detected_mask |= bit;
        if (act[i] == UMB_RX_ROS) { o.delegated_mask |= bit; continue; }
        if (act[i] != UMB_RX_PICO) continue;
        o.intervene_mask |= bit;
        switch ((umb_reflex_id_t)i) {
        case UMB_REFLEX_BATTERY:
        case UMB_REFLEX_IMPACT:
            o.stop = true; o.disarm = true; break;
        case UMB_REFLEX_COLLISION:
            o.block_forward = true;
            o.emergency_brake = o.emergency_brake || brake;
            break;
        case UMB_REFLEX_SPEED_CAP:
            apply_cap(&o, cap, c); break;
        case UMB_REFLEX_STALL:
            o.stop = true; break;
        case UMB_REFLEX_SENSOR_FAULT:
            if (tof_fault) apply_cap(&o, c->cap_min_ms, c);
            break;
        default: break;
        }
    }
    r.active_mask = o.detected_mask;
    *out = o;
    return r;
}
