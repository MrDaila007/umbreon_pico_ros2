/* One-core rcl allocator/platform hooks. Install before creating any ROS objects. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
typedef struct { size_t live_blocks, live_bytes, peak_bytes, failures; } hal_microros_alloc_stats_t;
bool hal_microros_platform_init(void);
/* Read from the network owner; publish a snapshot for cross-core readers. */
hal_microros_alloc_stats_t hal_microros_allocator_stats(void);
