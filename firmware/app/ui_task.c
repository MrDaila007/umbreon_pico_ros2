/* ui_task.c - see ui_task.h. Arming decisions stay in the core-1 control task:
 * the encoder only posts arm requests. Plan 4 remaps keys inside menus. */
#include "ui_task.h"
#include <stdio.h>
#include "task.h"
#include "hal_buzzer.h"
#include "hal_encoder.h"
#include "hal_time.h"
#include "shared_state.h"
#include "umb_buzzer.h"
#include "umb_encoder.h"

typedef struct {
    umb_player_t player;
    uint16_t freq;        /* tone currently driven (0 = silent) */
} ui_sound_t;

/* A new cue replaces whatever is playing. */
static void play(ui_sound_t *snd, umb_cue_t cue, uint32_t now_ms) {
    snd->player = umb_player_start(cue, now_ms);
}

static void sound_step(ui_sound_t *snd, uint32_t now_ms) {
    uint16_t f = 0;
    snd->player = umb_player_step(snd->player, now_ms, &f);
    if (f != snd->freq) {        /* reprogramming the PWM every tick would click */
        hal_buzzer_tone(f);
        snd->freq = f;
    }
}

static void handle_keys(void) {
    uint8_t ev = hal_encoder_take(NULL);
    if (ev & UMB_ENC_EVT_HOLD) shared_post_arm_request(SHARED_ARM_CAR);
    if (ev & UMB_ENC_EVT_CLICK) shared_post_arm_request(SHARED_ARM_DISARM);
}

void ui_task(void *param) {
    (void)param;
    ui_sound_t snd = { { 0 }, 0 };
    hal_buzzer_init();
    play(&snd, UMB_CUE_BOOT, hal_now_ms());
    bool enc_ok = hal_encoder_init();      /* from this core-1 task: its timer runs on core 1 */
    shared_set_encoder_state(enc_ok ? SHARED_ENC_OK : SHARED_ENC_FAILED);
    if (!enc_ok) {
        printf("ui: ERROR encoder init failed (no free alarm/timer); keys disabled\n");
        play(&snd, UMB_CUE_ERROR, hal_now_ms());
    }

    umb_outputs_t prev, cur;
    shared_get_outputs(&prev);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(UI_PERIOD_MS));
        if (enc_ok) handle_keys();
        uint32_t now = hal_now_ms();
        shared_get_outputs(&cur);
        umb_cue_t cue = umb_cue_from_outputs(&prev, &cur);
        prev = cur;
        if (cue != UMB_CUE_NONE) play(&snd, cue, now);
        int posted;
        if (shared_take_cue(&posted)) play(&snd, (umb_cue_t)posted, now);
        sound_step(&snd, now);
    }
}
