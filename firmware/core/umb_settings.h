/* umb_settings.h - settings model, sanitization, key/value access and
 * the two-slot flash blob format (pure C11, no SDK includes). */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "umb_types.h"

typedef struct {
    uint8_t  arming_mode;          /* umb_arm_mode_t */
    uint16_t cmd_timeout_ms;
    float    max_speed_ms, max_reverse_speed_ms;
    float    max_throttle, max_throttle_rev;
    int16_t  min_speed, max_speed, min_bspeed;      /* ESC us */
    int16_t  min_point, max_point, neutral_point;   /* servo deg */
    bool     servo_reverse;
    float    pid_kp, pid_ki, pid_kd;
    float    spd_slew;
    float    kick_pct;
    int16_t  kick_ms, corner_kick_us;
    int16_t  reverse_brake_cmd, reverse_brake_ms;
    int16_t  encoder_holes;
    float    wheel_diam_m;
    uint16_t tach_glitch_filter_us;
    bool     bat_enabled;
    float    bat_multiplier, bat_low;
    bool     imu_rotate;
    uint8_t  wifi_mode;            /* umb_wifi_mode_t */
    char     wifi_ssid[33];
    char     wifi_pass[64];
    char     ap_pass[64];
    uint32_t agent_ip;             /* host order, a.b.c.d = a<<24|..|d; 0 = discover */
    uint16_t agent_port;
    uint16_t sta_timeout_s;
    uint8_t  ros_domain_id;
    bool     calibrated;
    uint8_t  reflex_manual[UMB_REFLEX_COUNT];   /* umb_reflex_action_t */
    uint8_t  reflex_auto[UMB_REFLEX_COUNT];
    uint16_t ttc_ms, min_front_mm;
    uint16_t cap_far_mm, cap_near_mm;
    float    cap_min_ms;
    uint8_t  collision_sensor_mask;             /* bit i = ToF index i */
    uint16_t stall_min_us, stall_ms;
    float    impact_g;
} umb_settings_t;

/* Build-time defaults for network identity; the target build defines these
 * from config/secrets.cmake. Host builds see the empty defaults. */
#ifndef UMB_DEFAULT_WIFI_SSID
#define UMB_DEFAULT_WIFI_SSID ""
#endif
#ifndef UMB_DEFAULT_WIFI_PASS
#define UMB_DEFAULT_WIFI_PASS ""
#endif
#ifndef UMB_DEFAULT_AGENT_IP
#define UMB_DEFAULT_AGENT_IP 0u
#endif
#ifndef UMB_DEFAULT_AGENT_PORT
#define UMB_DEFAULT_AGENT_PORT 8888u
#endif

umb_settings_t umb_settings_defaults(void);
/* Clamp to ranges; non-finite floats become the default. */
umb_settings_t umb_settings_sanitize(umb_settings_t s);
/* Set one field. Rejects (false, *out untouched) unknown keys and
 * out-of-range or unparsable values. `in` and `out` may alias. */
bool umb_settings_set_kv(const umb_settings_t *in, const char *key, const char *value, umb_settings_t *out);
/* Print one field; passwords print as "***". False on unknown key or small buffer. */
bool umb_settings_get_kv(const umb_settings_t *s, const char *key, char *buf, size_t cap);

#define UMB_SETTINGS_SLOT_SIZE 4096u
#define UMB_SETTINGS_MAGIC 0x554D4250u   /* "UMBP" */
#define UMB_SETTINGS_VERSION 1u

/* Returns blob length, or 0 if cap is too small. */
size_t umb_settings_encode(const umb_settings_t *s, uint32_t seq, uint8_t *buf, size_t cap);
/* Validates magic, version, length and CRC; sanitizes the result. */
bool   umb_settings_decode(const uint8_t *buf, size_t len, umb_settings_t *out, uint32_t *seq);
/* Returns 0/1 (slot used), or -1 -> defaults (seq 0). Newest by serial arithmetic. */
int    umb_settings_pick_slot(const uint8_t *slot0, const uint8_t *slot1, size_t len,
                              umb_settings_t *out, uint32_t *seq);
