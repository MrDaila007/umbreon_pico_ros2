/* console_cmds.h - executes parsed console lines (core 0, console task only). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"

#define CONSOLE_REPEAT_MS 20u

/* Which console a line came from. */
typedef enum { CONSOLE_ORIGIN_USB, CONSOLE_ORIGIN_UART } console_origin_t;

/* A raw / speed / auto command being re-posted every CONSOLE_REPEAT_MS.
 * A USB-origin repeat stops as soon as the USB CDC host disconnects; a
 * UART-origin repeat runs to its [ms]. */
typedef struct {
    bool active;
    console_origin_t origin;
    umb_cmd_t cmd;
    uint32_t end_ms;
    uint32_t next_ms;
} console_repeat_t;

/* reset_reason must outlive the console (a string literal). */
void console_cmds_init(const char *reset_reason);
void console_execute(const char *line, console_origin_t origin, console_repeat_t *rep, uint32_t now_ms);
