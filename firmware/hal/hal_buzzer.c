/* hal_buzzer.c - see hal_buzzer.h */
#include "hal_buzzer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define BUZZER_GPIO 18u
#define PWM_WRAP_MAX 65535u

void hal_buzzer_init(void) {
    gpio_set_function(BUZZER_GPIO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_GPIO);
    uint chan = pwm_gpio_to_channel(BUZZER_GPIO);
    pwm_set_chan_level(slice, chan, 0);
    pwm_set_enabled(slice, false);
}

void hal_buzzer_tone(uint16_t freq_hz) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_GPIO);
    uint chan = pwm_gpio_to_channel(BUZZER_GPIO);
    if (freq_hz < HAL_BUZZER_MIN_HZ || freq_hz > HAL_BUZZER_MAX_HZ) {
        pwm_set_chan_level(slice, chan, 0);
        pwm_set_enabled(slice, false);
        return;
    }
    float sys = (float)clock_get_hz(clk_sys);
    float div = sys / ((float)freq_hz * (float)(PWM_WRAP_MAX + 1u));
    if (div < 1.0f) div = 1.0f;
    uint32_t period = (uint32_t)(sys / (div * (float)freq_hz) + 0.5f);   /* counts per cycle */
    if (period < 2u) period = 2u;
    if (period > PWM_WRAP_MAX + 1u) period = PWM_WRAP_MAX + 1u;
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, (uint16_t)(period - 1u));
    pwm_set_chan_level(slice, chan, (uint16_t)(period / 2u));
    pwm_set_enabled(slice, true);
}
