/* umb_console_parse.h - console line parser (pure C11, no SDK includes) */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_types.h"

typedef enum { UMB_CC_INVALID, UMB_CC_HELP, UMB_CC_STATUS, UMB_CC_RAW, UMB_CC_SPEED, UMB_CC_AUTO, UMB_CC_STOP,
               UMB_CC_ARM_CAR, UMB_CC_ARM_ROS, UMB_CC_DISARM, UMB_CC_MODE, UMB_CC_GET, UMB_CC_SET,
               UMB_CC_SAVE, UMB_CC_DEFAULTS, UMB_CC_REBOOT, UMB_CC_WIFI_SET, UMB_CC_AGENT_SET } umb_cc_kind_t;

typedef struct {
    umb_cc_kind_t kind;
    float steer, value;     /* RAW: steer, throttle; SPEED/AUTO: steer, m/s */
    uint16_t duration_ms;   /* RAW/SPEED/AUTO: default 1000, capped at 3000 */
    umb_arm_mode_t mode;    /* MODE */
    char key[32];           /* GET/SET key */
    char arg1[65];          /* SET value; WIFI ssid; AGENT ip text or "discover" */
    char arg2[65];          /* WIFI password; AGENT port text */
} umb_console_cmd_t;

/* Parses one console line. Always fully initialises *out (zeroed on failure,
 * kind == UMB_CC_INVALID). Returns true iff the line is a valid command. */
bool umb_console_parse(const char *line, umb_console_cmd_t *out);
