#include "test_runner.h"
#include "umb_buzzer.h"
#include <stddef.h>
#include <string.h>

static uint16_t freq_at(umb_player_t *p, uint32_t t) {
    uint16_t f = 0xFFFF;
    *p = umb_player_step(*p, t, &f);
    return f;
}

TEST(test_cue_notes_tables) {
    uint8_t n = 0;
    const umb_note_t *b = umb_cue_notes(UMB_CUE_BOOT, &n);
    ASSERT_EQ(n, 3);
    ASSERT_EQ(b[0].freq_hz, 1047);
    ASSERT_EQ(b[0].ms, 55);
    ASSERT_EQ(b[2].freq_hz, 1568);
    ASSERT_EQ(b[2].ms, 75);
    const umb_note_t *a = umb_cue_notes(UMB_CUE_ARMED, &n);
    ASSERT_EQ(n, 2);
    ASSERT_EQ(a[0].freq_hz, 784);
    ASSERT_EQ(a[1].ms, 70);
    const umb_note_t *d = umb_cue_notes(UMB_CUE_DISARMED, &n);
    ASSERT_EQ(n, 2);
    ASSERT_EQ(d[0].freq_hz, 1319);
    ASSERT_EQ(d[1].freq_hz, 1047);
    ASSERT_EQ(d[1].ms, 75);
    const umb_note_t *c = umb_cue_notes(UMB_CUE_CAL_DONE, &n);
    ASSERT_EQ(n, 4);
    ASSERT_EQ(c[3].freq_hz, 2093);
    ASSERT_EQ(c[3].ms, 90);
    const umb_note_t *e = umb_cue_notes(UMB_CUE_ERROR, &n);
    ASSERT_EQ(n, 3);
    ASSERT_EQ(e[0].freq_hz, 392);
    ASSERT_EQ(e[1].freq_hz, 0);
    ASSERT_EQ(e[1].ms, 60);
    const umb_note_t *r = umb_cue_notes(UMB_CUE_REFLEX, &n);
    ASSERT_EQ(n, 3);
    ASSERT_EQ(r[0].freq_hz, 2093);
    ASSERT_EQ(r[0].ms, 40);
    ASSERT_EQ(r[1].freq_hz, 0);
    const umb_note_t *l = umb_cue_notes(UMB_CUE_LOW_BATTERY, &n);
    ASSERT_EQ(n, 5);
    ASSERT_EQ(l[0].freq_hz, 523);
    ASSERT_EQ(l[0].ms, 150);
    ASSERT_EQ(l[1].ms, 80);
    ASSERT_EQ(l[4].freq_hz, 523);
    ASSERT_TRUE(umb_cue_notes(UMB_CUE_NONE, &n) == NULL);
    ASSERT_EQ(n, 0);
    ASSERT_TRUE(umb_cue_notes((umb_cue_t)99, &n) == NULL);
    ASSERT_EQ(n, 0);
}

TEST(test_player_sequence) {
    umb_player_t p = umb_player_start(UMB_CUE_ARMED, 1000);   /* 784/55, 1047/70 */
    ASSERT_TRUE(p.playing);
    ASSERT_EQ(freq_at(&p, 1000), 784);
    ASSERT_EQ(freq_at(&p, 1054), 784);
    ASSERT_EQ(freq_at(&p, 1055), 1047);
    ASSERT_EQ(freq_at(&p, 1124), 1047);
    ASSERT_TRUE(p.playing);
    ASSERT_EQ(freq_at(&p, 1125), 0);
    ASSERT_FALSE(p.playing);
    ASSERT_EQ(freq_at(&p, 1200), 0);
}

TEST(test_player_rest_is_silent_then_resumes) {
    umb_player_t p = umb_player_start(UMB_CUE_ERROR, 0);      /* 392/130, 0/60, 392/130 */
    ASSERT_EQ(freq_at(&p, 129), 392);
    ASSERT_EQ(freq_at(&p, 130), 0);
    ASSERT_TRUE(p.playing);
    ASSERT_EQ(freq_at(&p, 189), 0);
    ASSERT_EQ(freq_at(&p, 190), 392);
    ASSERT_EQ(freq_at(&p, 320), 0);
    ASSERT_FALSE(p.playing);
}

TEST(test_player_sparse_step_skips_notes) {
    umb_player_t p = umb_player_start(UMB_CUE_BOOT, 0);       /* 55, 55, 75 */
    ASSERT_EQ(freq_at(&p, 120), 1568);
    ASSERT_EQ(freq_at(&p, 500), 0);
}

TEST(test_player_wrap_safe) {
    umb_player_t p = umb_player_start(UMB_CUE_ARMED, 0xFFFFFFF0u);
    ASSERT_EQ(freq_at(&p, 0xFFFFFFF0u + 54), 784);
    ASSERT_EQ(freq_at(&p, 0xFFFFFFF0u + 55), 1047);           /* crosses the wrap */
    ASSERT_EQ(freq_at(&p, 0xFFFFFFF0u + 125), 0);
    ASSERT_FALSE(p.playing);
}

TEST(test_player_none_and_stopped) {
    umb_player_t p = umb_player_start(UMB_CUE_NONE, 100);
    ASSERT_FALSE(p.playing);
    ASSERT_EQ(freq_at(&p, 100), 0);
    umb_player_t z;
    memset(&z, 0, sizeof z);
    ASSERT_EQ(freq_at(&z, 5), 0);
}

static umb_outputs_t outs(bool armed, uint32_t detected, uint32_t intervene) {
    umb_outputs_t o;
    memset(&o, 0, sizeof o);
    o.armed = armed;
    o.reflex.detected_mask = detected;
    o.reflex.intervene_mask = intervene;
    return o;
}

TEST(test_cue_arm_disarm_transitions) {
    umb_outputs_t off = outs(false, 0, 0), on = outs(true, 0, 0);
    ASSERT_EQ(umb_cue_from_outputs(&off, &on), UMB_CUE_ARMED);
    ASSERT_EQ(umb_cue_from_outputs(&on, &off), UMB_CUE_DISARMED);
    ASSERT_EQ(umb_cue_from_outputs(&on, &on), UMB_CUE_NONE);
    ASSERT_EQ(umb_cue_from_outputs(&off, &off), UMB_CUE_NONE);
}

TEST(test_cue_null_prev_is_none) {
    umb_outputs_t on = outs(true, 1u << UMB_REFLEX_BATTERY, 1u << UMB_REFLEX_COLLISION);
    ASSERT_EQ(umb_cue_from_outputs(NULL, &on), UMB_CUE_NONE);
    ASSERT_EQ(umb_cue_from_outputs(&on, NULL), UMB_CUE_NONE);
}

TEST(test_cue_reflex_newly_intervening_only) {
    uint32_t col = 1u << UMB_REFLEX_COLLISION, stall = 1u << UMB_REFLEX_STALL;
    umb_outputs_t none = outs(true, 0, 0), a = outs(true, 0, col), ab = outs(true, 0, col | stall);
    ASSERT_EQ(umb_cue_from_outputs(&none, &a), UMB_CUE_REFLEX);
    ASSERT_EQ(umb_cue_from_outputs(&a, &a), UMB_CUE_NONE);      /* already intervening */
    ASSERT_EQ(umb_cue_from_outputs(&a, &ab), UMB_CUE_REFLEX);   /* a second bit is new */
    ASSERT_EQ(umb_cue_from_outputs(&ab, &a), UMB_CUE_NONE);     /* clearing is silent */
}

TEST(test_cue_low_battery_on_detect_edge) {
    uint32_t bat = 1u << UMB_REFLEX_BATTERY;
    umb_outputs_t clear = outs(true, 0, 0), low = outs(true, bat, 0);
    ASSERT_EQ(umb_cue_from_outputs(&clear, &low), UMB_CUE_LOW_BATTERY);
    ASSERT_EQ(umb_cue_from_outputs(&low, &low), UMB_CUE_NONE);
    umb_outputs_t other = outs(true, 1u << UMB_REFLEX_COLLISION, 0);
    ASSERT_EQ(umb_cue_from_outputs(&clear, &other), UMB_CUE_NONE);  /* detect-only, not battery */
}

TEST(test_cue_priority_low_battery_over_disarm) {
    uint32_t bat = 1u << UMB_REFLEX_BATTERY;
    umb_outputs_t prev = outs(true, 0, 0);
    umb_outputs_t cur = outs(false, bat, 1u << UMB_REFLEX_COLLISION);
    ASSERT_EQ(umb_cue_from_outputs(&prev, &cur), UMB_CUE_LOW_BATTERY);
    cur = outs(false, 0, 1u << UMB_REFLEX_COLLISION);
    ASSERT_EQ(umb_cue_from_outputs(&prev, &cur), UMB_CUE_REFLEX);   /* reflex over disarm */
}

int main(void) {
    RUN_TEST(test_cue_notes_tables);
    RUN_TEST(test_player_sequence);
    RUN_TEST(test_player_rest_is_silent_then_resumes);
    RUN_TEST(test_player_sparse_step_skips_notes);
    RUN_TEST(test_player_wrap_safe);
    RUN_TEST(test_player_none_and_stopped);
    RUN_TEST(test_cue_arm_disarm_transitions);
    RUN_TEST(test_cue_null_prev_is_none);
    RUN_TEST(test_cue_reflex_newly_intervening_only);
    RUN_TEST(test_cue_low_battery_on_detect_edge);
    RUN_TEST(test_cue_priority_low_battery_over_disarm);
    TEST_SUMMARY();
}
