/*
 * umb_battery.h - battery voltage EMA filter (pure, value semantics).
 * Ported in intent from umbreon_zephyr battery.c. The IMU/battery task feeds
 * raw 12-bit ADC samples; the first accepted sample seeds the filter.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UMB_BAT_EMA 0.05f
#define UMB_BAT_ADC_MAX 4095

typedef struct { float v_filtered, v_raw, v_min; bool have; } umb_battery_t;

umb_battery_t umb_battery_init(void);
/* v = adc12 * 3.3/4095 * multiplier. The first sample seeds filtered and min.
 * adc12 > 4095 or a non-finite multiplier -> sample ignored (state unchanged). */
umb_battery_t umb_battery_add(umb_battery_t b, uint16_t adc12, float multiplier);
