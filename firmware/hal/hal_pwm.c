/* hal_pwm.c - see hal_pwm.h */
#include "hal_pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define SERVO_GPIO 10u
#define ESC_GPIO 11u
#define NEUTRAL_US 1500u
#define PERIOD_US 20000u       /* 50 Hz */
#define TICK_HZ 1000000u       /* 1 us per counter tick */
/* Hardware sanity bound only; the control step produces the real limits. */
#define PULSE_MIN_US 500u
#define PULSE_MAX_US 2500u

static uint16_t clamp_pulse(uint16_t us) {
    if (us < PULSE_MIN_US) return PULSE_MIN_US;
    if (us > PULSE_MAX_US) return PULSE_MAX_US;
    return us;
}

void hal_pwm_init(void) {
    uint slice = pwm_gpio_to_slice_num(SERVO_GPIO);   /* slice 5; GP11 is its channel B */
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (float)TICK_HZ);
    pwm_config_set_wrap(&cfg, PERIOD_US - 1u);
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(SERVO_GPIO, NEUTRAL_US);
    pwm_set_gpio_level(ESC_GPIO, NEUTRAL_US);
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(ESC_GPIO, GPIO_FUNC_PWM);
    pwm_set_enabled(slice, true);
}

void hal_pwm_write(uint16_t servo_us, uint16_t esc_us) {
    pwm_set_gpio_level(SERVO_GPIO, clamp_pulse(servo_us));
    pwm_set_gpio_level(ESC_GPIO, clamp_pulse(esc_us));
}
