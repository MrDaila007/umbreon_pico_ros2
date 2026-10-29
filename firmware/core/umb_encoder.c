#include "umb_encoder.h"

static uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }

umb_encoder_t umb_encoder_init(bool clk, bool dt, bool btn_pressed, uint32_t now_ms) {
    umb_encoder_t e = {0};
    e.ab_prev = (uint8_t)(((clk ? 1 : 0) << 1) | (dt ? 1 : 0));
    e.btn_stable = e.btn_raw = btn_pressed;
    e.btn_change_ms = now_ms;
    e.press_ms = btn_pressed ? now_ms : 0;
    return e;
}

/* EncButton step4: valid transitions change exactly one bit; each adds +/-1 to
 * sub. A detent is reported when both lines are high again and sub != 0, so a
 * partial cycle that reverses cancels out. Sign matches umbreon_zephyr with
 * ENCODER_REVERSE false (sub > 0 -> rotation -1). */
static umb_encoder_t decode_rotation(umb_encoder_t e, uint8_t ab, uint8_t *events, int *rotation) {
    if (ab == e.ab_prev) return e;
    bool p0 = (e.ab_prev >> 1) & 1, p1 = e.ab_prev & 1;
    bool e0 = (ab >> 1) & 1, e1 = ab & 1;
    if (!(p0 ^ p1 ^ e0 ^ e1)) return e;            /* both bits changed: invalid */
    e.sub = (int8_t)(e.sub + ((p1 ^ e0) ? 1 : -1));
    e.ab_prev = ab;
    if (e.sub == 0 || !(e0 && e1)) return e;
    int step = e.sub > 0 ? -1 : 1;
    *rotation += step;
    *events |= (step > 0) ? UMB_ENC_EVT_CW : UMB_ENC_EVT_CCW;
    e.sub = 0;
    return e;
}

static umb_encoder_t on_press(umb_encoder_t e, uint32_t now, uint8_t *events) {
    e.press_ms = now;
    e.hold_sent = e.long_sent = false;
    /* A stale pending click (second press outside the window) is emitted now. */
    if (e.click_pending && elapsed(now, e.last_click_ms) > UMB_ENC_CLICK_MS) {
        *events |= UMB_ENC_EVT_CLICK;
        e.click_pending = false;
    }
    return e;
}

static umb_encoder_t on_release(umb_encoder_t e, uint32_t now, uint8_t *events) {
    if (e.hold_sent) return e;
    if (e.click_pending) {                           /* second press began inside the window */
        *events |= UMB_ENC_EVT_DOUBLE;
        e.click_pending = false;
    } else {
        e.click_pending = true;
        e.last_click_ms = now;
    }
    return e;
}

static umb_encoder_t step_button(umb_encoder_t e, bool pressed, uint32_t now, uint8_t *events) {
    if (pressed != e.btn_raw) {
        e.btn_raw = pressed;
        e.btn_change_ms = now;
    }
    if (e.btn_raw != e.btn_stable && elapsed(now, e.btn_change_ms) >= UMB_ENC_DEBOUNCE_MS) {
        e.btn_stable = e.btn_raw;
        e = e.btn_stable ? on_press(e, now, events) : on_release(e, now, events);
    }
    if (e.btn_stable) {
        uint32_t held = elapsed(now, e.press_ms);
        if (!e.hold_sent && held >= UMB_ENC_HOLD_MS) {
            *events |= UMB_ENC_EVT_HOLD;
            e.hold_sent = true;
            e.click_pending = false;                 /* a hold never contributes to click/double */
        }
        if (!e.long_sent && held >= UMB_ENC_LONG_HOLD_MS) {
            *events |= UMB_ENC_EVT_LONG_HOLD;
            e.long_sent = true;
        }
    } else if (e.click_pending && elapsed(now, e.last_click_ms) > UMB_ENC_CLICK_MS) {
        *events |= UMB_ENC_EVT_CLICK;
        e.click_pending = false;
    }
    return e;
}

umb_encoder_t umb_encoder_step(umb_encoder_t e, bool clk, bool dt, bool btn_pressed,
                               uint32_t now_ms, uint8_t *events, int *rotation) {
    uint8_t ab = (uint8_t)(((clk ? 1 : 0) << 1) | (dt ? 1 : 0));
    e = decode_rotation(e, ab, events, rotation);
    return step_button(e, btn_pressed, now_ms, events);
}
