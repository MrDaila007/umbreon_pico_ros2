/* hal_buzzer.h - passive buzzer on GP18 (PWM slice 1, channel A). Slice 5 (servo/ESC) is never touched. */
#pragma once

#include <stdint.h>

#define HAL_BUZZER_MIN_HZ 100u
#define HAL_BUZZER_MAX_HZ 10000u

void hal_buzzer_init(void);
/* 50 % duty tone; 0 or a frequency outside 100..10000 Hz turns it off. */
void hal_buzzer_tone(uint16_t freq_hz);
