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
 * then reads it back. Only call from a task holding the maintenance reservation with
 * control's neutral ACK. Temporarily extends the hardware watchdog around the write
 * (blocks ~one control period first). True on verified success. */
bool hal_settings_save(const umb_settings_t *s);
/* Copy of what flash holds now: the loaded blob (or the boot defaults when no slot
 * was valid), updated after each verified save. Never includes unsaved RAM edits.
 * Call only while holding the maintenance reservation (serializes with save). */
bool hal_settings_persisted(umb_settings_t *out);
/* Slot holding the newest valid blob (0 / 1), or -1 when running on defaults. */
int hal_settings_slot(void);
uint32_t hal_settings_seq(void);
/* Duration of the last flash_safe_execute erase+program, for bench evidence. */
uint32_t hal_settings_last_write_ms(void);
