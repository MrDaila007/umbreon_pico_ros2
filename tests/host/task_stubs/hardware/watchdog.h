#pragma once
#include <stdbool.h>
#include <stdint.h>
void watchdog_enable(uint32_t delay_ms, bool pause_on_debug);
void watchdog_update(void);
