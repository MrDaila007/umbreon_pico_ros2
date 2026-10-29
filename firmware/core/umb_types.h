/* umb_types.h - shared enums and command type (pure C11, no SDK includes) */
#pragma once
#include <stdint.h>

typedef enum { UMB_ARM_NONE = 0, UMB_ARM_ENCODER = 1, UMB_ARM_ROS = 2, UMB_ARM_BOTH = 3 } umb_arm_mode_t;
typedef enum { UMB_WIFI_AUTO = 0, UMB_WIFI_STA_ONLY = 1, UMB_WIFI_AP_ONLY = 2 } umb_wifi_mode_t;
typedef enum { UMB_SRC_CONSOLE = 0, UMB_SRC_ROS = 1 } umb_cmd_source_t;
typedef enum { UMB_DRIVE_MANUAL = 0, UMB_DRIVE_AUTO = 1 } umb_drive_mode_t;
typedef enum { UMB_RX_OFF = 0, UMB_RX_WARN = 1, UMB_RX_PICO = 2, UMB_RX_ROS = 3 } umb_reflex_action_t;
typedef enum { UMB_REFLEX_BATTERY = 0, UMB_REFLEX_COLLISION, UMB_REFLEX_SPEED_CAP, UMB_REFLEX_STALL,
               UMB_REFLEX_IMPACT, UMB_REFLEX_SENSOR_FAULT, UMB_REFLEX_COUNT } umb_reflex_id_t;
typedef enum { UMB_CMD_NONE = 0, UMB_CMD_SPEED = 1, UMB_CMD_RAW = 2 } umb_cmd_kind_t;

typedef struct {
    umb_cmd_kind_t kind;
    umb_cmd_source_t source;
    umb_drive_mode_t mode;
    float steer;   /* -1 left ... +1 right */
    float value;   /* SPEED: m/s signed; RAW: throttle -1...+1 */
    uint32_t rx_ms;
} umb_cmd_t;
