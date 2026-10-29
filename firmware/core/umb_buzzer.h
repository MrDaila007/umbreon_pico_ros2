/*
 * umb_buzzer.h - buzzer melodies, note sequencer and cue selection (pure).
 * Melodies ported from umbreon_zephyr buzzer.c; the UI task turns the freq
 * from umb_player_step() into PWM and picks cues from consecutive control
 * outputs. Wrap-safe uint32 millisecond time.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "umb_control.h"

typedef enum { UMB_CUE_NONE, UMB_CUE_BOOT, UMB_CUE_ARMED, UMB_CUE_DISARMED, UMB_CUE_CAL_DONE,
               UMB_CUE_ERROR, UMB_CUE_REFLEX, UMB_CUE_LOW_BATTERY } umb_cue_t;

typedef struct { uint16_t freq_hz; uint16_t ms; } umb_note_t;   /* freq 0 = rest */

typedef struct {
    const umb_note_t *notes;
    uint8_t n, idx;
    uint32_t note_end_ms;
    bool playing;
} umb_player_t;

/* Note table of a cue; NULL and *n = 0 for NONE / unknown. */
const umb_note_t *umb_cue_notes(umb_cue_t cue, uint8_t *n);
/* A player that plays `cue` from now_ms (stopped for NONE). */
umb_player_t umb_player_start(umb_cue_t cue, uint32_t now_ms);
/* Advance to now_ms; *freq_out is the frequency to sound (0 = silent: rest, finished or stopped). */
umb_player_t umb_player_step(umb_player_t p, uint32_t now_ms, uint16_t *freq_out);
/* Cue for the change prev -> cur. Priority: LOW_BATTERY (battery bit newly in
 * detected_mask) > REFLEX (any bit newly in intervene_mask) > DISARMED > ARMED.
 * prev == NULL (or cur == NULL) -> NONE. */
umb_cue_t umb_cue_from_outputs(const umb_outputs_t *prev, const umb_outputs_t *cur);
