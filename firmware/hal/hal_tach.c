/* hal_tach.c - see hal_tach.h */
#include "hal_tach.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#define TACH_GPIO 13u
#define GLITCH_MIN_US 1u
#define GLITCH_MAX_US 500u

/* Written only by the ISR (after init); aligned 32-bit accesses are atomic on the M33. */
static volatile uint32_t s_count;
static volatile uint32_t s_last_us;
static uint32_t s_glitch_us = GLITCH_MAX_US;

static void tach_isr(void) {
    if (!(gpio_get_irq_event_mask(TACH_GPIO) & GPIO_IRQ_EDGE_RISE)) return;
    gpio_acknowledge_irq(TACH_GPIO, GPIO_IRQ_EDGE_RISE);
    uint32_t now = time_us_32();
    if ((uint32_t)(now - s_last_us) < s_glitch_us) return;
    s_count = s_count + 1u;
    s_last_us = now;
}

void hal_tach_init(uint16_t glitch_us) {
    uint32_t g = glitch_us;
    s_glitch_us = g < GLITCH_MIN_US ? GLITCH_MIN_US : (g > GLITCH_MAX_US ? GLITCH_MAX_US : g);
    s_count = 0u;
    s_last_us = time_us_32();
    gpio_init(TACH_GPIO);
    gpio_set_dir(TACH_GPIO, GPIO_IN);
    gpio_disable_pulls(TACH_GPIO);   /* as in the Zephyr overlay: plain input, sensor drives the line */
    gpio_add_raw_irq_handler(TACH_GPIO, tach_isr);
    gpio_set_irq_enabled(TACH_GPIO, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

uint32_t hal_tach_count(void) {
    return s_count;
}

uint32_t hal_tach_age_us(void) {
    return time_us_32() - s_last_us;
}
