/* hal_time.h - monotonic milliseconds since boot (wraps after ~49 days). */
#pragma once
#include <stdint.h>

uint32_t hal_now_ms(void);
