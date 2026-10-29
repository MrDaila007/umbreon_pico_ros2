#include "hal_rand.h"
#include <string.h>
#include "pico/rand.h"
bool hal_rand_fill16(uint8_t out[16]) {
    if (!out) return false;
    rng_128_t value;
    get_rand_128(&value);
    memcpy(out, value.r, 16);
    memset(&value, 0, sizeof value);
    return true;
}
