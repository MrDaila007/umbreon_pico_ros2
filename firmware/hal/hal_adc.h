/* hal_adc.h - battery voltage divider on ADC0 (GP26). */
#pragma once

#include <stdint.h>

void hal_adc_init(void);
/* Raw 12-bit conversion (0..4095). Single caller (one task) assumed. */
uint16_t hal_adc_battery_raw(void);
