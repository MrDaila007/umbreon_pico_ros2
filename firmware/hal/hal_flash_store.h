/* hal_flash_store.h - two-slot settings storage in the last two 4 KiB flash sectors.
 * Slot 0 at PICO_FLASH_SIZE_BYTES - 8192, slot 1 at PICO_FLASH_SIZE_BYTES - 4096.
 * Blob format and slot choice are core/umb_settings (encode / decode / pick_slot). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_settings.h"

/* True when a valid slot was found; false -> *out holds the defaults. */
bool hal_settings_load(umb_settings_t *out);
/* Writes the slot not holding the newest blob, with seq + 1, via flash_safe_execute,
 * then reads it back. Only call while disarmed. True on verified success. */
bool hal_settings_save(const umb_settings_t *s);
/* Slot holding the newest valid blob (0 / 1), or -1 when running on defaults. */
int hal_settings_slot(void);
uint32_t hal_settings_seq(void);
