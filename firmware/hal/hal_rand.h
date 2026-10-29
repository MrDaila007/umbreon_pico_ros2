/* Hardware entropy for one-time secrets (RP2350 TRNG-backed pico_rand). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
bool hal_rand_fill16(uint8_t out[16]);
