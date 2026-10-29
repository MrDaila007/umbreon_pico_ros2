/* umb_settings_kv.c - table-driven sanitize / set_kv / get_kv for umb_settings_t. */
#include "umb_settings.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { K_U8, K_U16, K_I16, K_F32, K_BOOL, K_STR, K_IP, K_ARM, K_WIFI, K_RX } kind_t;

typedef struct {
    const char *key;
    kind_t kind;
    uint16_t off;
    uint16_t size;     /* K_STR: buffer capacity */
    double min, max;
    bool secret;
} field_t;

#define F(k, kd, name, lo, hi) { k, kd, offsetof(umb_settings_t, name), 0, lo, hi, false }
#define FN(name, kd, lo, hi) F(#name, kd, name, lo, hi)
#define FSTR(name, secret_) { #name, K_STR, offsetof(umb_settings_t, name), \
                              sizeof(((umb_settings_t *)0)->name), 0, 0, secret_ }
#define FRX(id, idname, mode) { "reflex." idname "." #mode, K_RX, \
        (uint16_t)(offsetof(umb_settings_t, reflex_##mode) + (id)), 0, 0, 3, false }
#define FRX2(id, idname) FRX(id, idname, manual), FRX(id, idname, auto)

static const field_t FIELDS[] = {
    FN(arming_mode, K_ARM, 0, 3),
    FN(cmd_timeout_ms, K_U16, 100, 1000),
    FN(max_speed_ms, K_F32, 0, 5), FN(max_reverse_speed_ms, K_F32, 0, 5),
    FN(max_throttle, K_F32, 0, 1), FN(max_throttle_rev, K_F32, 0, 1),
    FN(min_speed, K_I16, 1500, 2000), FN(max_speed, K_I16, 1500, 2000),
    FN(min_bspeed, K_I16, 1000, 1500),
    FN(min_point, K_I16, 0, 180), FN(max_point, K_I16, 0, 180), FN(neutral_point, K_I16, 0, 180),
    FN(servo_reverse, K_BOOL, 0, 1),
    FN(pid_kp, K_F32, 0, 5000), FN(pid_ki, K_F32, 0, 10000), FN(pid_kd, K_F32, 0, 2000),
    FN(spd_slew, K_F32, 0, 20),
    FN(kick_pct, K_F32, 0, 100), FN(kick_ms, K_I16, 0, 2000), FN(corner_kick_us, K_I16, 0, 200),
    FN(reverse_brake_cmd, K_I16, -1000, 0), FN(reverse_brake_ms, K_I16, 0, 5000),
    FN(encoder_holes, K_I16, 1, 2000), FN(wheel_diam_m, K_F32, 0.01, 1.0),
    FN(tach_glitch_filter_us, K_U16, 1, 500),
    FN(bat_enabled, K_BOOL, 0, 1), FN(bat_multiplier, K_F32, 0.1, 20), FN(bat_low, K_F32, 0.1, 20),
    FN(imu_rotate, K_BOOL, 0, 1),
    FN(wifi_mode, K_WIFI, 0, 2),
    FSTR(wifi_ssid, false), FSTR(wifi_pass, true), FSTR(ap_pass, true),
    FN(agent_ip, K_IP, 0, 0), FN(agent_port, K_U16, 1, 65535),
    FN(sta_timeout_s, K_U16, 5, 120), FN(ros_domain_id, K_U8, 0, 232),
    FN(calibrated, K_BOOL, 0, 1),
    FRX2(UMB_REFLEX_BATTERY, "battery"), FRX2(UMB_REFLEX_COLLISION, "collision"),
    FRX2(UMB_REFLEX_SPEED_CAP, "speed_cap"), FRX2(UMB_REFLEX_STALL, "stall"),
    FRX2(UMB_REFLEX_IMPACT, "impact"), FRX2(UMB_REFLEX_SENSOR_FAULT, "sensor_fault"),
    FN(ttc_ms, K_U16, 100, 3000), FN(min_front_mm, K_U16, 30, 1000),
    FN(cap_far_mm, K_U16, 100, 2000), FN(cap_near_mm, K_U16, 50, 1500),
    FN(cap_min_ms, K_F32, 0, 5),
    FN(collision_sensor_mask, K_U8, 1, 63),
    FN(stall_min_us, K_U16, 10, 500), FN(stall_ms, K_U16, 200, 5000),
    FN(impact_g, K_F32, 1.5, 16),
};
#define N_FIELDS (sizeof FIELDS / sizeof FIELDS[0])

static const char *const ARM_NAMES[] = { "none", "encoder", "ros", "both" };
static const char *const WIFI_NAMES[] = { "auto", "sta", "ap" };
static const char *const RX_NAMES[] = { "off", "warn", "pico", "ros" };

static const char *const *enum_names(kind_t k, size_t *n)
{
    switch (k) {
    case K_ARM:  *n = 4; return ARM_NAMES;
    case K_WIFI: *n = 3; return WIFI_NAMES;
    case K_RX:   *n = 4; return RX_NAMES;
    default:     *n = 0; return NULL;
    }
}

static size_t int_size(kind_t k)
{
    switch (k) {
    case K_U16: case K_I16: return 2;
    case K_IP:  return 4;
    default:    return 1;
    }
}

static bool is_signed(kind_t k) { return k == K_I16; }

static bool is_float(kind_t k) { return k == K_F32; }

/* Read an integer-kind field as long long. */
static long long read_int(const uint8_t *base, const field_t *f)
{
    const uint8_t *p = base + f->off;
    switch (int_size(f->kind)) {
    case 2:
        if (is_signed(f->kind)) { int16_t v; memcpy(&v, p, 2); return v; }
        else { uint16_t v; memcpy(&v, p, 2); return v; }
    case 4: { uint32_t v; memcpy(&v, p, 4); return v; }
    default: return *p;
    }
}

static void write_int(uint8_t *base, const field_t *f, long long v)
{
    uint8_t *p = base + f->off;
    switch (int_size(f->kind)) {
    case 2: { uint16_t u = (uint16_t)(int16_t)v; memcpy(p, &u, 2); break; }
    case 4: { uint32_t u = (uint32_t)v; memcpy(p, &u, 4); break; }
    default: *p = (uint8_t)v; break;
    }
}

static const field_t *find_field(const char *key)
{
    for (size_t i = 0; i < N_FIELDS; i++) {
        if (strcmp(FIELDS[i].key, key) == 0) return &FIELDS[i];
    }
    return NULL;
}

static size_t field_bytes(const field_t *f)
{
    if (f->kind == K_STR) return f->size;
    if (is_float(f->kind)) return sizeof(float);
    return int_size(f->kind);
}

/* Field-wise equality; struct padding bytes never take part. */
bool umb_settings_equal(const umb_settings_t *a, const umb_settings_t *b)
{
    for (size_t i = 0; i < N_FIELDS; i++) {
        const field_t *f = &FIELDS[i];
        const uint8_t *pa = (const uint8_t *)a + f->off;
        const uint8_t *pb = (const uint8_t *)b + f->off;
        if (f->kind == K_STR) {
            if (strncmp((const char *)pa, (const char *)pb, f->size) != 0) return false;
        } else if (memcmp(pa, pb, field_bytes(f)) != 0) {
            return false;
        }
    }
    return true;
}

/* Copy of s with zeroed padding and zeroed string tails (deterministic bytes). */
umb_settings_t umb_settings_canonical(const umb_settings_t *s)
{
    umb_settings_t c;
    memset(&c, 0, sizeof c);
    for (size_t i = 0; i < N_FIELDS; i++) {
        const field_t *f = &FIELDS[i];
        size_t n = field_bytes(f);
        if (f->kind == K_STR) {
            const char *str = (const char *)s + f->off;
            n = 0;
            while (n < (size_t)f->size - 1 && str[n]) n++;
        }
        memcpy((uint8_t *)&c + f->off, (const uint8_t *)s + f->off, n);
    }
    return c;
}

umb_settings_t umb_settings_sanitize(umb_settings_t s)
{
    const umb_settings_t d = umb_settings_defaults();
    uint8_t *sb = (uint8_t *)&s;
    const uint8_t *db = (const uint8_t *)&d;

    for (size_t i = 0; i < N_FIELDS; i++) {
        const field_t *f = &FIELDS[i];
        if (f->kind == K_STR) {
            sb[f->off + f->size - 1] = 0;
        } else if (f->kind == K_IP) {
            continue;
        } else if (is_float(f->kind)) {
            float v;
            memcpy(&v, sb + f->off, sizeof v);
            if (!isfinite(v)) {
                memcpy(&v, db + f->off, sizeof v);
            } else if (v < f->min) {
                v = (float)f->min;
            } else if (v > f->max) {
                v = (float)f->max;
            }
            memcpy(sb + f->off, &v, sizeof v);
        } else if (f->kind == K_BOOL) {
            sb[f->off] = sb[f->off] ? 1 : 0;
        } else {
            long long v = read_int(sb, f);
            if (v < f->min) v = (long long)f->min;
            if (v > f->max) v = (long long)f->max;
            write_int(sb, f, v);
        }
    }
    if (s.max_point < s.min_point) s.max_point = s.min_point;
    if (s.neutral_point < s.min_point) s.neutral_point = s.min_point;
    if (s.neutral_point > s.max_point) s.neutral_point = s.max_point;
    if (s.cap_near_mm >= s.cap_far_mm) s.cap_near_mm = (uint16_t)(s.cap_far_mm - 1);
    return s;
}

static bool parse_int(const char *v, long long *out)
{
    char *end;
    if (!v[0] || isspace((unsigned char)v[0])) return false;
    errno = 0;
    long long x = strtoll(v, &end, 10);
    if (errno || *end) return false;
    *out = x;
    return true;
}

static bool parse_float(const char *v, float *out)
{
    char *end;
    if (!v[0] || isspace((unsigned char)v[0])) return false;
    errno = 0;
    float x = strtof(v, &end);
    if (errno || *end || !isfinite(x)) return false;
    *out = x;
    return true;
}

static bool parse_ip(const char *v, uint32_t *out)
{
    if (strcmp(v, "discover") == 0) { *out = 0; return true; }
    uint32_t ip = 0;
    const char *p = v;
    for (int i = 0; i < 4; i++) {
        if (!isdigit((unsigned char)*p)) return false;
        unsigned oct = 0;
        int digits = 0;
        while (isdigit((unsigned char)*p)) {
            oct = oct * 10 + (unsigned)(*p++ - '0');
            if (++digits > 3 || oct > 255) return false;
        }
        ip = (ip << 8) | oct;
        if (i < 3 && *p++ != '.') return false;
    }
    if (*p) return false;
    *out = ip;
    return true;
}

static bool parse_enum(const field_t *f, const char *v, long long *out)
{
    size_t n;
    const char *const *names = enum_names(f->kind, &n);
    for (size_t i = 0; i < n; i++) {
        if (strcmp(names[i], v) == 0) { *out = (long long)i; return true; }
    }
    return parse_int(v, out);
}

static bool parse_bool(const char *v, long long *out)
{
    if (!strcmp(v, "1") || !strcmp(v, "true"))  { *out = 1; return true; }
    if (!strcmp(v, "0") || !strcmp(v, "false")) { *out = 0; return true; }
    return false;
}

/* Parse and store `value` into `s`; false if unparsable or out of range. */
static bool store_value(umb_settings_t *s, const field_t *f, const char *value)
{
    uint8_t *base = (uint8_t *)s;
    long long iv;
    float fv;
    size_t en;

    if (f->kind == K_STR) {
        size_t len = strlen(value);
        if (len >= f->size) return false;
        memset(base + f->off, 0, f->size);
        memcpy(base + f->off, value, len);
        return true;
    }
    if (f->kind == K_IP) {
        uint32_t ip;
        if (!parse_ip(value, &ip)) return false;
        memcpy(base + f->off, &ip, sizeof ip);
        return true;
    }
    if (is_float(f->kind)) {
        if (!parse_float(value, &fv) || fv < f->min || fv > f->max) return false;
        memcpy(base + f->off, &fv, sizeof fv);
        return true;
    }
    if (f->kind == K_BOOL) {
        if (!parse_bool(value, &iv)) return false;
    } else if (enum_names(f->kind, &en)) {
        if (!parse_enum(f, value, &iv)) return false;
    } else if (!parse_int(value, &iv)) {
        return false;
    }
    if (iv < f->min || iv > f->max) return false;
    write_int(base, f, iv);
    return true;
}

bool umb_settings_set_kv(const umb_settings_t *in, const char *key, const char *value, umb_settings_t *out)
{
    const field_t *f = find_field(key);
    if (!f || !value) return false;
    umb_settings_t t = *in;
    if (!store_value(&t, f, value)) return false;
    /* Cross-field rules: sanitize must be a no-op, otherwise reject. */
    umb_settings_t c = umb_settings_sanitize(t);
    if (!umb_settings_equal(&c, &t)) return false;
    *out = t;
    return true;
}

bool umb_settings_get_kv(const umb_settings_t *s, const char *key, char *buf, size_t cap)
{
    const field_t *f = find_field(key);
    if (!f || cap == 0) return false;
    const uint8_t *base = (const uint8_t *)s;
    int n;
    size_t en;
    const char *const *names = enum_names(f->kind, &en);

    if (f->kind == K_STR) {
        n = f->secret ? snprintf(buf, cap, "***") : snprintf(buf, cap, "%s", (const char *)base + f->off);
    } else if (f->kind == K_IP) {
        uint32_t ip = (uint32_t)read_int(base, f);
        n = ip == 0 ? snprintf(buf, cap, "discover")
                    : snprintf(buf, cap, "%u.%u.%u.%u", (unsigned)(ip >> 24),
                               (unsigned)((ip >> 16) & 255), (unsigned)((ip >> 8) & 255),
                               (unsigned)(ip & 255));
    } else if (is_float(f->kind)) {
        float v;
        memcpy(&v, base + f->off, sizeof v);
        n = snprintf(buf, cap, "%.6g", (double)v);
    } else if (names) {
        long long v = read_int(base, f);
        n = (v >= 0 && (size_t)v < en) ? snprintf(buf, cap, "%s", names[v])
                                       : snprintf(buf, cap, "%lld", v);
    } else {
        n = snprintf(buf, cap, "%lld", read_int(base, f));
    }
    return n >= 0 && (size_t)n < cap;
}
