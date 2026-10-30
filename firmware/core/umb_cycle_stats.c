#include "umb_cycle_stats.h"

static uint32_t bump(uint32_t v) { return v == UINT32_MAX ? v : v + 1u; }

umb_cycle_stats_t umb_cycle_stats_record(umb_cycle_stats_t s, int32_t late_ms, bool maintenance) {
    s.cycles = bump(s.cycles);
    bool fresh = late_ms > s.last_late_ms;
    s.last_late_ms = late_ms;
    if (late_ms <= (int32_t)UMB_CYCLE_JITTER_TOL_MS || !fresh) return s;
    uint32_t late = (uint32_t)late_ms;
    if (maintenance) {
        s.flash_pauses = bump(s.flash_pauses);
        if (late > s.flash_pause_max_ms) s.flash_pause_max_ms = late;
    } else {
        s.jitter_late = bump(s.jitter_late);
        if (late > s.jitter_max_ms) s.jitter_max_ms = late;
    }
    return s;
}
