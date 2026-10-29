/* hal_flash_store.c - see hal_flash_store.h */
#include "hal_flash_store.h"
#include <string.h>
#include "hardware/flash.h"
#include "pico/error.h"
#include "pico/flash.h"
#include "pico/platform.h"

#define SLOT0_OFFSET (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)
#define FLASH_LOCKOUT_TIMEOUT_MS 100u

_Static_assert(UMB_SETTINGS_SLOT_SIZE == FLASH_SECTOR_SIZE, "one slot per flash sector");

static int s_slot = -1;        /* newest valid slot, -1 = defaults */
static uint32_t s_seq;
static uint8_t s_buf[UMB_SETTINGS_SLOT_SIZE] __attribute__((aligned(4)));

typedef struct {
    uint32_t offset;
    const uint8_t *data;
} write_job_t;

static uint32_t slot_offset(int slot) {
    return SLOT0_OFFSET + (uint32_t)slot * FLASH_SECTOR_SIZE;
}

static const uint8_t *slot_ptr(int slot) {
    return (const uint8_t *)(XIP_BASE + slot_offset(slot));
}

bool hal_settings_load(umb_settings_t *out) {
    uint32_t seq = 0;
    s_slot = umb_settings_pick_slot(slot_ptr(0), slot_ptr(1), UMB_SETTINGS_SLOT_SIZE, out, &seq);
    s_seq = seq;
    return s_slot >= 0;
}

/* Runs with the other core locked out and interrupts off. */
static void __no_inline_not_in_flash_func(write_sector)(void *param) {
    const write_job_t *job = (const write_job_t *)param;
    flash_range_erase(job->offset, FLASH_SECTOR_SIZE);
    flash_range_program(job->offset, job->data, FLASH_SECTOR_SIZE);
}

bool hal_settings_save(const umb_settings_t *s) {
    int target = (s_slot == 0) ? 1 : 0;
    uint32_t seq = s_seq + 1u;
    memset(s_buf, 0xFF, sizeof s_buf);
    if (umb_settings_encode(s, seq, s_buf, sizeof s_buf) == 0u) return false;

    write_job_t job = { slot_offset(target), s_buf };
    /* Caller holds the maintenance reservation and has the control task's neutral
     * ACK after its watchdog feed. Never feed the watchdog from the flash path. */
    if (flash_safe_execute(write_sector, &job, FLASH_LOCKOUT_TIMEOUT_MS) != PICO_OK) return false;

    umb_settings_t check;
    uint32_t check_seq = 0;
    if (!umb_settings_decode(slot_ptr(target), UMB_SETTINGS_SLOT_SIZE, &check, &check_seq) ||
        check_seq != seq) {
        return false;
    }
    s_slot = target;
    s_seq = seq;
    return true;
}

int hal_settings_slot(void) {
    return s_slot;
}

uint32_t hal_settings_seq(void) {
    return s_seq;
}
