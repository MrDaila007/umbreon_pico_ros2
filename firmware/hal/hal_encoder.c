/* hal_encoder.c - see hal_encoder.h. No FreeRTOS calls in the timer callback. */
#include "hal_encoder.h"
#include <stdbool.h>
#include "umb_encoder.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"

#define ENC_CLK_GPIO 22u
#define ENC_DT_GPIO 12u
#define ENC_BTN_GPIO 19u
#define ENC_PERIOD_US 1000
#define ENC_POOL_MAX_TIMERS 2

static repeating_timer_t s_timer;
static alarm_pool_t *s_pool;
static umb_encoder_t s_enc;
static spin_lock_t *s_lock;
static volatile uint8_t s_events;
static volatile int s_rotation;

static void pin_in(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

static bool timer_cb(repeating_timer_t *rt) {
    (void)rt;
    bool clk = gpio_get(ENC_CLK_GPIO);
    bool dt = gpio_get(ENC_DT_GPIO);
    bool btn = !gpio_get(ENC_BTN_GPIO);
    uint8_t ev = 0;
    int rot = 0;
    s_enc = umb_encoder_step(s_enc, clk, dt, btn, to_ms_since_boot(get_absolute_time()), &ev, &rot);
    if (ev != 0 || rot != 0) {
        uint32_t save = spin_lock_blocking(s_lock);
        s_events = (uint8_t)(s_events | ev);
        s_rotation = s_rotation + rot;
        spin_unlock(s_lock, save);
    }
    return true;
}

bool hal_encoder_init(void) {
    pin_in(ENC_CLK_GPIO);
    pin_in(ENC_DT_GPIO);
    pin_in(ENC_BTN_GPIO);
    s_lock = spin_lock_init(spin_lock_claim_unused(true));
    s_events = 0;
    s_rotation = 0;
    s_enc = umb_encoder_init(gpio_get(ENC_CLK_GPIO), gpio_get(ENC_DT_GPIO), !gpio_get(ENC_BTN_GPIO),
                             to_ms_since_boot(get_absolute_time()));
    /* The default alarm pool is global and lives on core 0. A dedicated pool created here binds
     * its IRQ to the calling core (core 1). It claims the first unused hardware alarm of the
     * timer (the default pool holds alarm 3; FreeRTOS uses SysTick, not an alarm). */
    s_pool = alarm_pool_create_with_unused_hardware_alarm(ENC_POOL_MAX_TIMERS);
    if (s_pool == NULL) return false;
    /* Negative delay: period measured start-to-start, no drift. */
    return alarm_pool_add_repeating_timer_us(s_pool, -ENC_PERIOD_US, timer_cb, NULL, &s_timer);
}

uint8_t hal_encoder_take(int *rotation) {
    uint32_t save = spin_lock_blocking(s_lock);
    uint8_t ev = s_events;
    int rot = s_rotation;
    s_events = 0;
    s_rotation = 0;
    spin_unlock(s_lock, save);
    if (rotation) *rotation = rot;
    return ev;
}
