#include "umb_battery.h"
#include <math.h>

umb_battery_t umb_battery_init(void) {
    umb_battery_t b = {0.0f, 0.0f, 0.0f, false};
    return b;
}

umb_battery_t umb_battery_add(umb_battery_t b, uint16_t adc12, float multiplier) {
    if (adc12 > UMB_BAT_ADC_MAX || !isfinite(multiplier)) return b;
    float v = (float)adc12 * (3.3f / 4095.0f) * multiplier;
    b.v_raw = v;
    if (!b.have) {
        b.v_filtered = v;
        b.v_min = v;
        b.have = true;
    } else {
        b.v_filtered = UMB_BAT_EMA * v + (1.0f - UMB_BAT_EMA) * b.v_filtered;
        if (v < b.v_min) b.v_min = v;
    }
    return b;
}
