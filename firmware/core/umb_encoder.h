/*
 * umb_encoder.h - rotary encoder + push-button decoder (pure, value semantics).
 * Ported from umbreon_zephyr encoder.c (EncButton-style step4 decoding,
 * ENCODER_REVERSE false). The 1 ms timer reads the pins and calls
 * umb_encoder_step(); clk/dt/btn are the logical levels: clk/dt as read
 * (idle = high with pull-ups), btn_pressed true while pressed.
 * All time arithmetic is wrap-safe (uint32 milliseconds).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UMB_ENC_EVT_CW 0x01
#define UMB_ENC_EVT_CCW 0x02
#define UMB_ENC_EVT_CLICK 0x04
#define UMB_ENC_EVT_DOUBLE 0x08
#define UMB_ENC_EVT_HOLD 0x10
#define UMB_ENC_EVT_LONG_HOLD 0x20

#define UMB_ENC_DEBOUNCE_MS 50
#define UMB_ENC_CLICK_MS 500
#define UMB_ENC_HOLD_MS 600
#define UMB_ENC_LONG_HOLD_MS 3000

typedef struct {
    uint8_t ab_prev;            /* (clk << 1) | dt */
    int8_t sub;                 /* quadrature accumulator, see umb_encoder.c */
    bool btn_stable, btn_raw;   /* debounced / last raw button level */
    uint32_t btn_change_ms, press_ms, last_click_ms;
    bool hold_sent, long_sent, click_pending;
} umb_encoder_t;

umb_encoder_t umb_encoder_init(bool clk, bool dt, bool btn_pressed, uint32_t now_ms);

/* Called every 1 ms. Event bits are OR-ed into *events; detents are added to
 * *rotation (CW positive). CLICK is emitted only once the double-click window
 * (500 ms after release) passes without a second press; DOUBLE is emitted on
 * the second release (no CLICK for either press). A press that becomes HOLD
 * never produces CLICK/DOUBLE and cancels a pending first click. */
umb_encoder_t umb_encoder_step(umb_encoder_t e, bool clk, bool dt, bool btn_pressed,
                               uint32_t now_ms, uint8_t *events, int *rotation);
