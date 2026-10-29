/* umb_gear.c - gear state machine, see umb_gear.h. */
#include "umb_gear.h"
#include <math.h>

#define GEAR_MAX_CASCADE 8

static bool reached(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int8_t request_dir(float request) {
    if (!isfinite(request)) return 0;
    if (request > UMB_DIR_DEADBAND) return 1;
    if (request < -UMB_DIR_DEADBAND) return -1;
    return 0;
}

static umb_gear_t make(umb_gear_phase_t phase, int8_t target_dir, uint32_t deadline_ms) {
    umb_gear_t g = { phase, target_dir, deadline_ms };
    return g;
}

static uint32_t brake_ms(const umb_settings_t *c) {
    return c->reverse_brake_ms > 0 ? (uint32_t)c->reverse_brake_ms : 0u;
}

umb_gear_t umb_gear_init(void) {
    return make(UMB_GEAR_NEUTRAL, 0, 0);
}

/* One transition attempt. Returns true if the state changed and the
 * caller should evaluate again (cascade). *stop_cascade ends evaluation
 * for this call even when the state changed. */
static bool advance(umb_gear_t *g, int8_t d, float speed_abs, uint32_t now,
                    const umb_settings_t *c, bool *stop_cascade) {
    switch (g->phase) {
    case UMB_GEAR_NEUTRAL:
        if (d > 0) { *g = make(UMB_GEAR_FORWARD, 1, 0); return true; }
        if (d < 0) { *g = make(UMB_GEAR_WAIT_STOP, -1, now + UMB_GEAR_WAIT_STOP_MS); return true; }
        return false;
    case UMB_GEAR_FORWARD:
        if (d == 0) { *g = make(UMB_GEAR_NEUTRAL, 0, 0); return true; }
        if (d < 0) { *g = make(UMB_GEAR_WAIT_STOP, -1, now + UMB_GEAR_WAIT_STOP_MS); return true; }
        return false;
    case UMB_GEAR_REVERSE:
        if (d == 0) { *g = make(UMB_GEAR_NEUTRAL, 0, 0); return true; }
        if (d > 0) { *g = make(UMB_GEAR_WAIT_STOP, 1, now + UMB_GEAR_WAIT_STOP_MS); return true; }
        return false;
    case UMB_GEAR_WAIT_STOP:
        if (d == 0) { *g = make(UMB_GEAR_NEUTRAL, 0, 0); return true; }
        if (d != g->target_dir) { g->target_dir = d; return true; }
        if (speed_abs < UMB_GEAR_STOP_SPEED || reached(now, g->deadline_ms)) {
            *g = (d > 0) ? make(UMB_GEAR_FORWARD, 1, 0)
                         : make(UMB_GEAR_BRAKE, -1, now + brake_ms(c));
            return true;
        }
        return false;
    case UMB_GEAR_BRAKE:
    case UMB_GEAR_GAP:
        if (g->target_dir != 0) {   /* normal sequence: cancel / redirect by request */
            if (d == 0) { *g = make(UMB_GEAR_NEUTRAL, 0, 0); return true; }
            if (d > 0) { *g = make(UMB_GEAR_WAIT_STOP, 1, now + UMB_GEAR_WAIT_STOP_MS); return true; }
        }
        if (!reached(now, g->deadline_ms)) return false;
        if (g->phase == UMB_GEAR_BRAKE) {
            *g = make(UMB_GEAR_GAP, g->target_dir, now + UMB_GEAR_GAP_MS);
        } else if (g->target_dir != 0) {
            *g = make(UMB_GEAR_REVERSE, -1, 0);
        } else {
            *g = make(UMB_GEAR_NEUTRAL, 0, 0);
            *stop_cascade = true;   /* emergency stop: stay neutral this step */
        }
        return true;
    }
    return false;
}

umb_gear_t umb_gear_step(umb_gear_t g, float request, float speed_abs, uint32_t now_ms,
                         const umb_settings_t *c) {
    int8_t d = request_dir(request);
    bool stop = false;
    for (int i = 0; i < GEAR_MAX_CASCADE && !stop; i++) {
        if (!advance(&g, d, speed_abs, now_ms, c, &stop)) break;
    }
    return g;
}

int8_t umb_gear_drive_dir(umb_gear_t g) {
    if (g.phase == UMB_GEAR_FORWARD) return 1;
    if (g.phase == UMB_GEAR_REVERSE) return -1;
    return 0;
}

bool umb_gear_braking(umb_gear_t g) {
    return g.phase == UMB_GEAR_BRAKE;
}

float umb_gear_speed_sign(umb_gear_t g) {
    if (g.phase == UMB_GEAR_REVERSE) return -1.0f;
    if (g.phase == UMB_GEAR_WAIT_STOP && g.target_dir > 0) return -1.0f;
    return 1.0f;
}

umb_gear_t umb_gear_emergency_brake(umb_gear_t g, uint32_t now_ms, const umb_settings_t *c) {
    if (g.phase != UMB_GEAR_FORWARD) return g;
    return make(UMB_GEAR_BRAKE, 0, now_ms + brake_ms(c));
}
