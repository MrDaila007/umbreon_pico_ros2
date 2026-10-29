#include "umb_buzzer.h"
#include <stddef.h>

static const umb_note_t BOOT[] = {{1047, 55}, {1319, 55}, {1568, 75}};
static const umb_note_t ARMED[] = {{784, 55}, {1047, 70}};
static const umb_note_t DISARMED[] = {{1319, 45}, {1047, 75}};
static const umb_note_t CAL_DONE[] = {{1047, 45}, {1319, 45}, {1568, 45}, {2093, 90}};
static const umb_note_t ERROR_[] = {{392, 130}, {0, 60}, {392, 130}};
static const umb_note_t REFLEX[] = {{2093, 40}, {0, 40}, {2093, 40}};
static const umb_note_t LOW_BAT[] = {{523, 150}, {0, 80}, {523, 150}, {0, 80}, {523, 150}};

#define TABLE(t) do { *n = (uint8_t)(sizeof(t) / sizeof((t)[0])); return (t); } while (0)

const umb_note_t *umb_cue_notes(umb_cue_t cue, uint8_t *n) {
    switch (cue) {
    case UMB_CUE_BOOT: TABLE(BOOT);
    case UMB_CUE_ARMED: TABLE(ARMED);
    case UMB_CUE_DISARMED: TABLE(DISARMED);
    case UMB_CUE_CAL_DONE: TABLE(CAL_DONE);
    case UMB_CUE_ERROR: TABLE(ERROR_);
    case UMB_CUE_REFLEX: TABLE(REFLEX);
    case UMB_CUE_LOW_BATTERY: TABLE(LOW_BAT);
    default: break;
    }
    *n = 0;
    return NULL;
}

umb_player_t umb_player_start(umb_cue_t cue, uint32_t now_ms) {
    umb_player_t p = {0};
    p.notes = umb_cue_notes(cue, &p.n);
    if (p.notes != NULL && p.n > 0) {
        p.playing = true;
        p.note_end_ms = now_ms + p.notes[0].ms;
    }
    return p;
}

umb_player_t umb_player_step(umb_player_t p, uint32_t now_ms, uint16_t *freq_out) {
    while (p.playing && (int32_t)(now_ms - p.note_end_ms) >= 0) {
        p.idx++;
        if (p.idx >= p.n) {
            p.playing = false;
        } else {
            p.note_end_ms += p.notes[p.idx].ms;
        }
    }
    *freq_out = p.playing ? p.notes[p.idx].freq_hz : 0;
    return p;
}

umb_cue_t umb_cue_from_outputs(const umb_outputs_t *prev, const umb_outputs_t *cur) {
    if (prev == NULL || cur == NULL) return UMB_CUE_NONE;
    uint32_t bat = 1u << UMB_REFLEX_BATTERY;
    if ((cur->reflex.detected_mask & ~prev->reflex.detected_mask) & bat) return UMB_CUE_LOW_BATTERY;
    if (cur->reflex.intervene_mask & ~prev->reflex.intervene_mask) return UMB_CUE_REFLEX;
    if (prev->armed && !cur->armed) return UMB_CUE_DISARMED;
    if (!prev->armed && cur->armed) return UMB_CUE_ARMED;
    return UMB_CUE_NONE;
}
