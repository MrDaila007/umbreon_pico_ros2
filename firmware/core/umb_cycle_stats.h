/* umb_cycle_stats.h - control-cycle lateness accounting (pure C11). A cycle that
 * wakes late while a maintenance reservation is held is a flash-induced pause
 * (the other core is parked for erase/program); any other late cycle is ordinary
 * scheduling jitter. The two are counted apart so a save never inflates jitter. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define UMB_CYCLE_JITTER_TOL_MS 5u  /* lateness up to this is on time */

typedef struct {
    uint32_t cycles;
    uint32_t jitter_late, jitter_max_ms;
    uint32_t flash_pauses, flash_pause_max_ms;
    int32_t last_late_ms; /* previous cycle, to tell a new stall from catch-up */
} umb_cycle_stats_t;

/* late_ms: wake time minus the scheduled wake time (negative = early). A late
 * cycle that is not later than the one before it is catch-up after a stall, not a
 * new event. Counters saturate rather than wrap. */
umb_cycle_stats_t umb_cycle_stats_record(umb_cycle_stats_t s, int32_t late_ms, bool maintenance);
