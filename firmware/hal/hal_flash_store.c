/* hal_flash_store.c - see hal_flash_store.h */
#include "hal_flash_store.h"
#include <string.h>
#include "hardware/flash.h"
#include "pico/error.h"
#include "pico/flash.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "control_task.h"

#define SLOT0_OFFSET (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)
#define FLASH_LOCKOUT_TIMEOUT_MS 100u
/* Covers a worst-case 4 KiB sector erase (~400 ms for typical QSPI NOR) plus
 * program, lockout entry/exit and margin. Applies only around one write. */
#define FLASH_WATCHDOG_MS 2000u

_Static_assert(UMB_SETTINGS_SLOT_SIZE == FLASH_SECTOR_SIZE, "one slot per flash sector");

static int s_slot = -1;        /* newest valid slot, -1 = defaults */
static uint32_t s_seq;
static uint32_t s_last_write_ms;
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
     * ACK after its watchdog feed. flash_safe_execute parks core 1 (the control
     * task, sole routine watchdog feeder) with interrupts off for the whole erase +
     * program, which can exceed the 250 ms control watchdog. So, only around this
     * write: re-arm the watchdog with a longer timeout. The SDK keeps one shared
     * reload value that core 1's watchdog_update() also reads, so wait one control
     * period: a feed that raced with the old value is superseded by control's own
     * feed of the extended value (we never feed on its behalf). Afterwards restore
     * the normal timeout; control's next feed reloads 250 ms. Outputs are neutral
     * and disarmed throughout (maintenance), so the longer window cannot hide motion. */
    watchdog_enable(FLASH_WATCHDOG_MS, true);
    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS + 5u));
    absolute_time_t start = get_absolute_time();
    int rc = flash_safe_execute(write_sector, &job, FLASH_LOCKOUT_TIMEOUT_MS);
    s_last_write_ms = (uint32_t)(absolute_time_diff_us(start, get_absolute_time()) / 1000);
    watchdog_enable(CONTROL_WATCHDOG_MS, true);
    if (rc != PICO_OK) return false;

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

uint32_t hal_settings_last_write_ms(void) {
    return s_last_write_ms;
}
