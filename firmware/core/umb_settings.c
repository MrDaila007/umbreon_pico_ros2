/* umb_settings.c - defaults and the two-slot flash blob format.
 * Key/value access and sanitization live in umb_settings_kv.c. */
#include "umb_settings.h"
#include "umb_crc32.h"
#include <string.h>

#define HDR_LEN 12u   /* magic u32, version u16, payload_len u16, seq u32 */
#define CRC_LEN 4u

umb_settings_t umb_settings_defaults(void)
{
    umb_settings_t s;
    memset(&s, 0, sizeof s);
    s.arming_mode = UMB_ARM_BOTH;
    s.cmd_timeout_ms = 250;
    s.max_speed_ms = 1.0f;  s.max_reverse_speed_ms = 0.5f;
    s.max_throttle = 1.0f;  s.max_throttle_rev = 0.38f;
    s.min_speed = 1540;  s.max_speed = 1600;  s.min_bspeed = 1460;
    s.min_point = 60;  s.max_point = 120;  s.neutral_point = 90;
    s.servo_reverse = false;
    s.pid_kp = 66.4f;  s.pid_ki = 243.5f;  s.pid_kd = 4.16f;
    s.spd_slew = 0.85f;
    s.kick_pct = 18.0f;  s.kick_ms = 300;  s.corner_kick_us = 30;
    s.reverse_brake_cmd = -250;  s.reverse_brake_ms = 650;
    s.encoder_holes = 2;  s.wheel_diam_m = 0.060f;
    s.tach_glitch_filter_us = 500;
    s.bat_enabled = false;  s.bat_multiplier = 4.85f;  s.bat_low = 6.0f;
    s.imu_rotate = true;
    s.wifi_mode = UMB_WIFI_AUTO;
    strncpy(s.wifi_ssid, UMB_DEFAULT_WIFI_SSID, sizeof s.wifi_ssid - 1);
    strncpy(s.wifi_pass, UMB_DEFAULT_WIFI_PASS, sizeof s.wifi_pass - 1);
    s.agent_ip = UMB_DEFAULT_AGENT_IP;
    s.agent_port = UMB_DEFAULT_AGENT_PORT;
    s.sta_timeout_s = 15;  s.ros_domain_id = 0;
    s.calibrated = false;
    for (int i = 0; i < UMB_REFLEX_COUNT; i++) {
        s.reflex_manual[i] = UMB_RX_WARN;
        s.reflex_auto[i] = UMB_RX_PICO;
    }
    s.reflex_manual[UMB_REFLEX_BATTERY] = UMB_RX_PICO;
    s.ttc_ms = 400;  s.min_front_mm = 150;
    s.cap_far_mm = 1000;  s.cap_near_mm = 300;  s.cap_min_ms = 0.2f;
    s.collision_sensor_mask = (1u << 1) | (1u << 4);
    s.stall_min_us = 60;  s.stall_ms = 700;
    s.impact_g = 4.0f;
    return s;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t umb_settings_encode(const umb_settings_t *s, uint32_t seq, uint8_t *buf, size_t cap)
{
    size_t total = HDR_LEN + sizeof *s + CRC_LEN;
    if (cap < total) return 0;
    put_u32(buf, UMB_SETTINGS_MAGIC);
    put_u16(buf + 4, UMB_SETTINGS_VERSION);
    put_u16(buf + 6, (uint16_t)sizeof *s);
    put_u32(buf + 8, seq);
    umb_settings_t c = umb_settings_canonical(s);
    memcpy(buf + HDR_LEN, &c, sizeof c);
    put_u32(buf + HDR_LEN + sizeof *s, umb_crc32(buf, HDR_LEN + sizeof *s));
    return total;
}

bool umb_settings_decode(const uint8_t *buf, size_t len, umb_settings_t *out, uint32_t *seq)
{
    size_t total = HDR_LEN + sizeof *out + CRC_LEN;
    if (len < total) return false;
    if (get_u32(buf) != UMB_SETTINGS_MAGIC) return false;
    if (get_u16(buf + 4) != UMB_SETTINGS_VERSION) return false;
    if (get_u16(buf + 6) != sizeof *out) return false;
    if (get_u32(buf + HDR_LEN + sizeof *out) != umb_crc32(buf, HDR_LEN + sizeof *out)) return false;
    umb_settings_t tmp;
    memcpy(&tmp, buf + HDR_LEN, sizeof tmp);
    *out = umb_settings_sanitize(tmp);
    if (seq) *seq = get_u32(buf + 8);
    return true;
}

int umb_settings_pick_slot(const uint8_t *slot0, const uint8_t *slot1, size_t len,
                           umb_settings_t *out, uint32_t *seq)
{
    umb_settings_t a, b;
    uint32_t sa = 0, sb = 0;
    bool va = umb_settings_decode(slot0, len, &a, &sa);
    bool vb = umb_settings_decode(slot1, len, &b, &sb);
    if (va && (!vb || (int32_t)(sb - sa) <= 0)) {
        *out = a;  if (seq) *seq = sa;
        return 0;
    }
    if (vb) {
        *out = b;  if (seq) *seq = sb;
        return 1;
    }
    *out = umb_settings_defaults();
    if (seq) *seq = 0;
    return -1;
}
