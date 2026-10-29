/* hal_time.c - see hal_time.h */
#include "hal_time.h"
#include "pico/time.h"

uint32_t hal_now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}
