/* hal_encoder.h - rotary encoder + button sampled by a 1 ms repeating timer running umb_encoder_step.
 * CLK GP22, DT GP12, button GP19; all active low with pull-ups. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* MUST be called from a core-1 task, once. The default alarm pool is global and runs on core 0,
 * so this creates a dedicated alarm pool (first unused hardware alarm) bound to the calling core;
 * the 1 ms callback then runs on core 1. Returns false if no alarm/timer slot was available. */
bool hal_encoder_init(void);

/* Atomically take (and clear) the accumulated UMB_ENC_EVT_* bits and detent count (CW positive).
 * rotation may be NULL. Safe from any core/task. */
uint8_t hal_encoder_take(int *rotation);
