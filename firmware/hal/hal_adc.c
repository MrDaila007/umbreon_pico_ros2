/* hal_adc.c - see hal_adc.h */
#include "hal_adc.h"
#include "hardware/adc.h"

#define BATTERY_GPIO 26u
#define BATTERY_ADC_INPUT 0u

void hal_adc_init(void) {
    adc_init();
    adc_gpio_init(BATTERY_GPIO);
}

uint16_t hal_adc_battery_raw(void) {
    adc_select_input(BATTERY_ADC_INPUT);
    return adc_read();
}
