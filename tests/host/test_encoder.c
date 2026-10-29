#include "test_runner.h"
#include "umb_encoder.h"
#include <stdint.h>

typedef struct {
    umb_encoder_t e;
    uint32_t now;
    uint8_t ev;      /* OR of every event seen */
    int rot;
    bool clk, dt, btn;
} rig_t;

static rig_t rig(uint32_t t0) {
    rig_t r = {0};
    r.clk = r.dt = true;
    r.now = t0;
    r.e = umb_encoder_init(true, true, false, t0);
    return r;
}
/* One 1 ms tick with the current pin levels. */
static void tick(rig_t *r) {
    r->now++;
    r->e = umb_encoder_step(r->e, r->clk, r->dt, r->btn, r->now, &r->ev, &r->rot);
}
static void run(rig_t *r, uint32_t ms) { for (uint32_t i = 0; i < ms; i++) tick(r); }
static void ab(rig_t *r, bool clk, bool dt) { r->clk = clk; r->dt = dt; tick(r); }
static void press_for(rig_t *r, uint32_t ms) { r->btn = true; run(r, ms); r->btn = false; }

/* CW (rotation +1): 11 -> 10 -> 00 -> 01 -> 11 */
static void cw(rig_t *r) { ab(r, true, false); ab(r, false, false); ab(r, false, true); ab(r, true, true); }
static void ccw(rig_t *r) { ab(r, false, true); ab(r, false, false); ab(r, true, false); ab(r, true, true); }

TEST(test_encoder_cw_one_detent) {
    rig_t r = rig(1000);
    cw(&r);
    ASSERT_EQ(r.rot, 1);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CW);
    cw(&r);
    ASSERT_EQ(r.rot, 2);
}

TEST(test_encoder_ccw_one_detent) {
    rig_t r = rig(1000);
    ccw(&r);
    ASSERT_EQ(r.rot, -1);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CCW);
}

TEST(test_encoder_partial_reverse_cancels) {
    rig_t r = rig(1000);
    ab(&r, true, false);
    ab(&r, false, false);
    ab(&r, true, false);
    ab(&r, true, true);
    ASSERT_EQ(r.rot, 0);
    ASSERT_EQ(r.ev, 0);
}

TEST(test_encoder_invalid_transition_ignored) {
    rig_t r = rig(1000);
    ab(&r, false, false);        /* 11 -> 00 is already a two-bit jump */
    ab(&r, true, true);          /* 00 -> 11 likewise */
    ASSERT_EQ(r.rot, 0);
    ASSERT_EQ(r.ev, 0);
    ASSERT_EQ(r.e.ab_prev, 3);   /* state not advanced by the invalid jumps */
    ASSERT_EQ(r.e.sub, 0);
}

TEST(test_encoder_bounce_no_click) {
    rig_t r = rig(1000);
    for (int i = 0; i < 8; i++) { r.btn = !r.btn; run(&r, 5); }
    ASSERT_FALSE(r.btn);
    run(&r, 1000);
    ASSERT_EQ(r.ev, 0);
}

TEST(test_encoder_click) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 400);
    ASSERT_EQ(r.ev, 0);          /* held back until the window passes */
    run(&r, 200);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CLICK);
}

TEST(test_encoder_click_emitted_once) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 700);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CLICK);
    r.ev = 0;
    run(&r, 2000);
    ASSERT_EQ(r.ev, 0);
}

TEST(test_encoder_double) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 200);
    press_for(&r, 100);
    ASSERT_EQ(r.ev, 0);
    run(&r, 100);                /* DOUBLE fires on the second release, after debounce */
    ASSERT_EQ(r.ev, UMB_ENC_EVT_DOUBLE);
    r.ev = 0;
    run(&r, 2000);
    ASSERT_EQ(r.ev, 0);          /* no trailing CLICK */
}

TEST(test_encoder_slow_second_press_still_double) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 300);
    press_for(&r, 400);          /* released after the window, but pressed inside it */
    run(&r, 100);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_DOUBLE);
}

TEST(test_encoder_late_second_press_is_two_clicks) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 800);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CLICK);
    r.ev = 0;
    press_for(&r, 100);
    run(&r, 700);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CLICK);
}

TEST(test_encoder_hold_no_click_on_release) {
    rig_t r = rig(1000);
    press_for(&r, 700);
    ASSERT_TRUE(r.ev & UMB_ENC_EVT_HOLD);
    ASSERT_FALSE(r.ev & UMB_ENC_EVT_LONG_HOLD);
    run(&r, 2000);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_HOLD);
}

TEST(test_encoder_hold_after_click_cancels_double) {
    rig_t r = rig(1000);
    press_for(&r, 100);
    run(&r, 100);
    press_for(&r, 800);          /* second press becomes a HOLD */
    run(&r, 2000);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_HOLD);
}

TEST(test_encoder_long_hold_once) {
    rig_t r = rig(1000);
    r.btn = true;
    run(&r, 3500);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_HOLD | UMB_ENC_EVT_LONG_HOLD);
    r.ev = 0;
    run(&r, 3000);
    ASSERT_EQ(r.ev, 0);          /* neither repeats within the same press */
    r.btn = false;
    run(&r, 2000);
    ASSERT_EQ(r.ev, 0);
}

TEST(test_encoder_events_or_not_cleared) {
    rig_t r = rig(1000);
    r.ev = 0x80;
    cw(&r);
    ASSERT_EQ(r.ev, 0x80 | UMB_ENC_EVT_CW);
}

TEST(test_encoder_wrap_safe) {
    rig_t r = rig(0xFFFFFFFFu - 150u);
    press_for(&r, 100);          /* press/release straddle the 32-bit wrap */
    run(&r, 700);
    ASSERT_EQ(r.ev, UMB_ENC_EVT_CLICK);
    rig_t h = rig(0xFFFFFFFFu - 1000u);
    h.btn = true;
    run(&h, 3500);
    ASSERT_EQ(h.ev, UMB_ENC_EVT_HOLD | UMB_ENC_EVT_LONG_HOLD);
    rig_t d = rig(0xFFFFFFFFu - 250u);
    press_for(&d, 100);
    run(&d, 100);
    press_for(&d, 100);
    run(&d, 100);
    ASSERT_EQ(d.ev, UMB_ENC_EVT_DOUBLE);
}

TEST(test_encoder_pressed_at_init_holds) {
    umb_encoder_t e = umb_encoder_init(true, true, true, 0);
    uint8_t ev = 0; int rot = 0;
    for (uint32_t t = 1; t <= 700; t++) e = umb_encoder_step(e, true, true, true, t, &ev, &rot);
    ASSERT_EQ(ev, UMB_ENC_EVT_HOLD);
}

int main(void) {
    RUN_TEST(test_encoder_cw_one_detent);
    RUN_TEST(test_encoder_ccw_one_detent);
    RUN_TEST(test_encoder_partial_reverse_cancels);
    RUN_TEST(test_encoder_invalid_transition_ignored);
    RUN_TEST(test_encoder_bounce_no_click);
    RUN_TEST(test_encoder_click);
    RUN_TEST(test_encoder_click_emitted_once);
    RUN_TEST(test_encoder_double);
    RUN_TEST(test_encoder_slow_second_press_still_double);
    RUN_TEST(test_encoder_late_second_press_is_two_clicks);
    RUN_TEST(test_encoder_hold_no_click_on_release);
    RUN_TEST(test_encoder_hold_after_click_cancels_double);
    RUN_TEST(test_encoder_long_hold_once);
    RUN_TEST(test_encoder_events_or_not_cleared);
    RUN_TEST(test_encoder_wrap_safe);
    RUN_TEST(test_encoder_pressed_at_init_holds);
    TEST_SUMMARY();
}
