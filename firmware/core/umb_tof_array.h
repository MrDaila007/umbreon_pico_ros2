/*
 * umb_tof_array.h — health, staleness and restart policy for the 6 VL53L0X sensors.
 * Pure logic (no I/O); value semantics. The ToF task calls it around the real driver.
 * Ported in intent from umbreon_zephyr sensors.c.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UMB_TOF_COUNT 6
#define UMB_TOF_INVALID_MM 9999
#define UMB_TOF_STALE_MS 1500
#define UMB_TOF_RESTART_COOLDOWN_MS 3000
#define UMB_TOF_BASE_ADDR 0x30
#define UMB_TOF_OOR_MM 8190             /* mm == 0 or >= this reports INVALID_MM */
#define UMB_TOF_MAX_CONSEC_ERRORS 10    /* consecutive poll errors before OFFLINE */

typedef enum { UMB_TOF_OFFLINE, UMB_TOF_ONLINE } umb_tof_link_t;

typedef struct {
    umb_tof_link_t link;
    uint16_t mm;                /* last good reading (already 9999 if out of range) */
    uint32_t last_ok_ms;        /* last good poll, or the successful init */
    uint32_t last_restart_ms;   /* last init attempt; meaningful only if attempted */
    uint32_t errors;            /* total poll errors */
    uint32_t restarts;          /* init attempts, INCLUDING the first boot init */
    uint32_t consec_errors;     /* poll errors since last good poll / init */
    bool     have_reading;
    bool     attempted;         /* an init has been reported (false = never restarted) */
    bool     boot_init;         /* the last init attempt was the first one (not a recovery) */
} umb_tof_slot_t;

typedef struct { umb_tof_slot_t s[UMB_TOF_COUNT]; } umb_tof_array_t;

typedef enum { UMB_TOF_ACT_NONE, UMB_TOF_ACT_RESTART } umb_tof_action_t;

/* All slots OFFLINE and never attempted: next_action returns RESTART immediately. */
umb_tof_array_t  umb_tof_init(void);
uint8_t          umb_tof_addr(int i);   /* 0x30 + i; 0 if i out of range */

/* Report an init/restart attempt. ok: ONLINE with fresh grace period; else OFFLINE. */
umb_tof_array_t  umb_tof_on_init_result(umb_tof_array_t a, int i, bool ok, uint32_t now_ms);
/* rc: 1 new reading, 0 not ready, <0 error. Polls of OFFLINE slots are ignored. */
umb_tof_array_t  umb_tof_on_poll(umb_tof_array_t a, int i, int rc, uint16_t mm, uint32_t now_ms);
umb_tof_action_t umb_tof_next_action(const umb_tof_array_t *a, int i, uint32_t now_ms);
/* invalid/stale/offline -> 9999 */
void             umb_tof_distances(const umb_tof_array_t *a, uint32_t now_ms, int16_t out_mm[6]);
uint8_t          umb_tof_online(const umb_tof_array_t *a, uint32_t now_ms); /* online and fresh */
/* Any slot restarted within the last 3 s, ignoring each slot's first (boot) init. */
bool             umb_tof_recovering(const umb_tof_array_t *a, uint32_t now_ms);
