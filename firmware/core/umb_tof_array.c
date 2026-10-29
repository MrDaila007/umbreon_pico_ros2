#include "umb_tof_array.h"
#include <stddef.h>

static bool valid_idx(int i) { return i >= 0 && i < UMB_TOF_COUNT; }

/* Wrap-safe elapsed ms; a stamp in the future yields a negative value. */
static int32_t elapsed(uint32_t now_ms, uint32_t then_ms) {
    return (int32_t)(now_ms - then_ms);
}

static bool slot_fresh(const umb_tof_slot_t *s, uint32_t now_ms) {
    return s->link == UMB_TOF_ONLINE && elapsed(now_ms, s->last_ok_ms) <= UMB_TOF_STALE_MS;
}

umb_tof_array_t umb_tof_init(void) {
    umb_tof_array_t a = {0};
    return a;
}

uint8_t umb_tof_addr(int i) {
    return valid_idx(i) ? (uint8_t)(UMB_TOF_BASE_ADDR + i) : 0;
}

umb_tof_array_t umb_tof_on_init_result(umb_tof_array_t a, int i, bool ok, uint32_t now_ms) {
    if (!valid_idx(i)) return a;
    umb_tof_slot_t *s = &a.s[i];
    s->boot_init = !s->attempted;
    s->attempted = true;
    s->last_restart_ms = now_ms;
    s->restarts++;
    s->consec_errors = 0;
    s->have_reading = false;
    s->link = ok ? UMB_TOF_ONLINE : UMB_TOF_OFFLINE;
    if (ok) s->last_ok_ms = now_ms;
    return a;
}

umb_tof_array_t umb_tof_on_poll(umb_tof_array_t a, int i, int rc, uint16_t mm, uint32_t now_ms) {
    if (!valid_idx(i) || a.s[i].link != UMB_TOF_ONLINE) return a;
    umb_tof_slot_t *s = &a.s[i];
    if (rc < 0) {
        s->errors++;
        if (++s->consec_errors >= UMB_TOF_MAX_CONSEC_ERRORS) s->link = UMB_TOF_OFFLINE;
    } else if (rc > 0) {
        s->mm = (mm == 0 || mm >= UMB_TOF_OOR_MM) ? UMB_TOF_INVALID_MM : mm;
        s->have_reading = true;
        s->last_ok_ms = now_ms;
        s->consec_errors = 0;
    }
    return a;
}

umb_tof_action_t umb_tof_next_action(const umb_tof_array_t *a, int i, uint32_t now_ms) {
    if (!a || !valid_idx(i)) return UMB_TOF_ACT_NONE;
    const umb_tof_slot_t *s = &a->s[i];
    if (slot_fresh(s, now_ms)) return UMB_TOF_ACT_NONE;
    if (!s->attempted) return UMB_TOF_ACT_RESTART;
    return elapsed(now_ms, s->last_restart_ms) >= UMB_TOF_RESTART_COOLDOWN_MS
               ? UMB_TOF_ACT_RESTART : UMB_TOF_ACT_NONE;
}

void umb_tof_distances(const umb_tof_array_t *a, uint32_t now_ms, int16_t out_mm[6]) {
    if (!out_mm) return;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        const umb_tof_slot_t *s = a ? &a->s[i] : NULL;
        out_mm[i] = (s && slot_fresh(s, now_ms) && s->have_reading)
                        ? (int16_t)s->mm : (int16_t)UMB_TOF_INVALID_MM;
    }
}

uint8_t umb_tof_online(const umb_tof_array_t *a, uint32_t now_ms) {
    uint8_t n = 0;
    if (!a) return 0;
    for (int i = 0; i < UMB_TOF_COUNT; i++)
        if (slot_fresh(&a->s[i], now_ms)) n++;
    return n;
}

bool umb_tof_recovering(const umb_tof_array_t *a, uint32_t now_ms) {
    if (!a) return false;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        const umb_tof_slot_t *s = &a->s[i];
        int32_t dt = elapsed(now_ms, s->last_restart_ms);
        if (s->attempted && !s->boot_init && dt >= 0 && dt < UMB_TOF_RESTART_COOLDOWN_MS)
            return true;
    }
    return false;
}
