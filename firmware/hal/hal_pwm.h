/* hal_pwm.h - servo (GP10) and ESC (GP11) pulses on PWM slice 5, 50 Hz, 1 us steps.
 * Only the control task calls hal_pwm_write. */
#pragma once
#include <stdint.h>

/* Writes 1500/1500 before the slice is enabled. */
void hal_pwm_init(void);
void hal_pwm_write(uint16_t servo_us, uint16_t esc_us);
