/* hal_time.h - monotonic milliseconds since boot (wraps after ~49 days). */
#pragma once
#include <stdint.h>

uint32_t hal_now_ms(void);

uint64_t hal_now_us64(void); /* full monotonic boot time for ROS stamp mapping */
