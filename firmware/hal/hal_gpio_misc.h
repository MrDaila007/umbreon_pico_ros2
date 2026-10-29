/* hal_gpio_misc.h - ToF XSHUT lines (GP6,7,8,9,14,15 = sensor 0..5: HR,FR,R,L,FL,HL). */
#pragma once

#include <stdbool.h>

#define HAL_XSHUT_COUNT 6

/* Outputs, driven LOW (all sensors held in reset). */
void hal_xshut_init(void);
/* on=true releases the sensor from reset (drives high). Out-of-range i is ignored. */
void hal_xshut_set(int i, bool on);
