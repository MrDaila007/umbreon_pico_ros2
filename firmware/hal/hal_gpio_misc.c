/* hal_gpio_misc.c - see hal_gpio_misc.h */
#include "hal_gpio_misc.h"
#include "hardware/gpio.h"

static const unsigned char XSHUT_PINS[HAL_XSHUT_COUNT] = {6, 7, 8, 9, 14, 15};

void hal_xshut_init(void) {
    for (int i = 0; i < HAL_XSHUT_COUNT; i++) {
        gpio_init(XSHUT_PINS[i]);
        gpio_put(XSHUT_PINS[i], 0);
        gpio_set_dir(XSHUT_PINS[i], GPIO_OUT);
    }
}

void hal_xshut_set(int i, bool on) {
    if (i < 0 || i >= HAL_XSHUT_COUNT) return;
    gpio_put(XSHUT_PINS[i], on);
}
