/* hal_tach.h - Hall tachometer on GP13 (rising edge IRQ with glitch filter).
 * Port of umbreon_zephyr/src/tachometer.c (counting part; speed lives in core/umb_pid). */
#pragma once
#include <stdint.h>

/* glitch_us is clamped to 1..500; pulses closer than that to the previous one are ignored. */
void hal_tach_init(uint16_t glitch_us);
uint32_t hal_tach_count(void);
/* Microseconds since the last accepted pulse (since init before the first pulse). */
uint32_t hal_tach_age_us(void);
