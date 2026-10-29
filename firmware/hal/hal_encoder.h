/* hal_encoder.h - rotary encoder + button sampled by a 1 ms repeating timer running umb_encoder_step.
 * CLK GP22, DT GP12, button GP19; all active low with pull-ups. */
#pragma once

#include <stdint.h>

/* MUST be called from a core-1 task (before use): the repeating timer lives in the calling
 * core's default alarm pool, so the 1 ms callback then runs on core 1. Call once. */
void hal_encoder_init(void);

/* Atomically take (and clear) the accumulated UMB_ENC_EVT_* bits and detent count (CW positive).
 * rotation may be NULL. Safe from any core/task. */
uint8_t hal_encoder_take(int *rotation);
