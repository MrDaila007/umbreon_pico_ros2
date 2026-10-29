/* umb_console_parse.c - console line parser (pure C11) */
#include "umb_console_parse.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TOKENS     5      /* longest grammar: wifi set <ssid> <pass> */
#define TOKEN_BUF      128    /* tokens longer than this are rejected */
#define DEFAULT_MS     1000u
#define MAX_MS         3000u
#define MAX_SPEED_MPS  5.0f
#define KEY_MAX        31
#define VALUE_MAX      64
#define SSID_MAX       32
#define PASS_MIN       8
#define PASS_MAX       63

typedef struct {
    int count;
    char tok[MAX_TOKENS][TOKEN_BUF];
} tokens_t;

static bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

/* Splits line into tokens; "double quoted" tokens may contain whitespace.
 * Fails on too many tokens, over-long tokens or malformed quotes. */
static bool tokenize(const char *p, tokens_t *t) {
    t->count = 0;
    for (;;) {
        while (is_ws(*p)) p++;
        if (*p == '\0') return true;
        if (t->count >= MAX_TOKENS) return false;
        char *dst = t->tok[t->count];
        size_t n = 0;
        if (*p == '"') {
            p++;
            while (*p != '"') {
                if (*p == '\0' || n >= TOKEN_BUF - 1) return false;
                dst[n++] = *p++;
            }
            p++;
            if (*p != '\0' && !is_ws(*p)) return false;
        } else {
            while (*p != '\0' && !is_ws(*p)) {
                if (n >= TOKEN_BUF - 1) return false;
                dst[n++] = *p++;
            }
        }
        dst[n] = '\0';
        t->count++;
    }
}

static bool parse_float(const char *s, float lo, float hi, float *out) {
    char *end;
    if (*s == '\0') return false;
    float v = strtof(s, &end);
    if (*end != '\0' || !isfinite(v) || v < lo || v > hi) return false;
    *out = v;
    return true;
}

/* Decimal digits only; saturates instead of overflowing. */
static bool parse_uint(const char *s, unsigned long *out) {
    unsigned long v = 0;
    if (*s == '\0') return false;
    for (; *s != '\0'; s++) {
        if (*s < '0' || *s > '9') return false;
        if (v < 100000000ul) v = v * 10ul + (unsigned long)(*s - '0');
    }
    *out = v;
    return true;
}

static bool parse_duration(const tokens_t *t, int idx, uint16_t *ms) {
    if (idx >= t->count) { *ms = (uint16_t)DEFAULT_MS; return true; }
    unsigned long v;
    if (!parse_uint(t->tok[idx], &v) || v < 1ul) return false;
    *ms = (uint16_t)(v > MAX_MS ? MAX_MS : v);
    return true;
}

static bool parse_drive(const tokens_t *t, umb_console_cmd_t *o, float value_limit) {
    if (t->count < 3 || t->count > 4) return false;
    return parse_float(t->tok[1], -1.0f, 1.0f, &o->steer) &&
           parse_float(t->tok[2], -value_limit, value_limit, &o->value) &&
           parse_duration(t, 3, &o->duration_ms);
}

static bool valid_ipv4(const char *s) {
    int parts = 0;
    while (parts < 4) {
        int digits = 0;
        unsigned v = 0;
        while (*s >= '0' && *s <= '9') {
            if (++digits > 3) return false;
            v = v * 10u + (unsigned)(*s++ - '0');
        }
        if (digits == 0 || v > 255u) return false;
        parts++;
        if (parts < 4) {
            if (*s++ != '.') return false;
        }
    }
    return *s == '\0';
}

static bool parse_wifi(const tokens_t *t, umb_console_cmd_t *o) {
    if (t->count != 4 || strcmp(t->tok[1], "set") != 0) return false;
    size_t ssid = strlen(t->tok[2]), pass = strlen(t->tok[3]);
    if (ssid < 1 || ssid > SSID_MAX) return false;
    if (pass != 0 && (pass < PASS_MIN || pass > PASS_MAX)) return false;
    memcpy(o->arg1, t->tok[2], ssid + 1);
    memcpy(o->arg2, t->tok[3], pass + 1);
    return true;
}

static bool parse_agent(const tokens_t *t, umb_console_cmd_t *o) {
    unsigned long port;
    if (t->count != 4 || strcmp(t->tok[1], "set") != 0) return false;
    if (strcmp(t->tok[2], "discover") != 0 && !valid_ipv4(t->tok[2])) return false;
    if (!parse_uint(t->tok[3], &port) || port < 1ul || port > 65535ul) return false;
    memcpy(o->arg1, t->tok[2], strlen(t->tok[2]) + 1);   /* <= 15 or 8 chars */
    memcpy(o->arg2, t->tok[3], strlen(t->tok[3]) + 1);   /* digits, <= 65 */
    return true;
}

static bool parse_mode(const tokens_t *t, umb_console_cmd_t *o) {
    static const struct { const char *name; umb_arm_mode_t mode; } modes[] = {
        { "none", UMB_ARM_NONE }, { "encoder", UMB_ARM_ENCODER },
        { "ros", UMB_ARM_ROS },   { "both", UMB_ARM_BOTH } };
    if (t->count != 2) return false;
    for (size_t i = 0; i < sizeof modes / sizeof modes[0]; i++) {
        if (strcmp(t->tok[1], modes[i].name) == 0) { o->mode = modes[i].mode; return true; }
    }
    return false;
}

static bool copy_key(const char *s, umb_console_cmd_t *o) {
    size_t n = strlen(s);
    if (n < 1 || n > KEY_MAX) return false;
    memcpy(o->key, s, n + 1);
    return true;
}

static bool parse_set(const tokens_t *t, umb_console_cmd_t *o) {
    size_t n = strlen(t->tok[2]);
    if (t->count != 3 || n < 1 || n > VALUE_MAX || !copy_key(t->tok[1], o)) return false;
    memcpy(o->arg1, t->tok[2], n + 1);
    return true;
}

/* Commands taking no arguments. */
static bool simple_kind(const char *w, umb_cc_kind_t *k) {
    static const struct { const char *name; umb_cc_kind_t kind; } tab[] = {
        { "help", UMB_CC_HELP }, { "status", UMB_CC_STATUS }, { "stop", UMB_CC_STOP },
        { "disarm", UMB_CC_DISARM }, { "save", UMB_CC_SAVE },
        { "defaults", UMB_CC_DEFAULTS }, { "reboot", UMB_CC_REBOOT } };
    for (size_t i = 0; i < sizeof tab / sizeof tab[0]; i++) {
        if (strcmp(w, tab[i].name) == 0) { *k = tab[i].kind; return true; }
    }
    return false;
}

static bool dispatch(const tokens_t *t, umb_console_cmd_t *o) {
    const char *w = t->tok[0];
    umb_cc_kind_t k;
    if (t->count == 0) return false;
    if (simple_kind(w, &k)) { o->kind = k; return t->count == 1; }
    if (strcmp(w, "raw") == 0)   { o->kind = UMB_CC_RAW;   return parse_drive(t, o, 1.0f); }
    if (strcmp(w, "speed") == 0) { o->kind = UMB_CC_SPEED; return parse_drive(t, o, MAX_SPEED_MPS); }
    if (strcmp(w, "auto") == 0)  { o->kind = UMB_CC_AUTO;  return parse_drive(t, o, MAX_SPEED_MPS); }
    if (strcmp(w, "arm") == 0) {
        if (t->count != 2) return false;
        if (strcmp(t->tok[1], "car") == 0) { o->kind = UMB_CC_ARM_CAR; return true; }
        if (strcmp(t->tok[1], "ros") == 0) { o->kind = UMB_CC_ARM_ROS; return true; }
        return false;
    }
    if (strcmp(w, "mode") == 0)  { o->kind = UMB_CC_MODE; return parse_mode(t, o); }
    if (strcmp(w, "get") == 0)   { o->kind = UMB_CC_GET;  return t->count == 2 && copy_key(t->tok[1], o); }
    if (strcmp(w, "set") == 0)   { o->kind = UMB_CC_SET;  return parse_set(t, o); }
    if (strcmp(w, "wifi") == 0)  { o->kind = UMB_CC_WIFI_SET;  return parse_wifi(t, o); }
    if (strcmp(w, "agent") == 0) { o->kind = UMB_CC_AGENT_SET; return parse_agent(t, o); }
    return false;
}

bool umb_console_parse(const char *line, umb_console_cmd_t *out) {
    tokens_t t;
    if (out == NULL) return false;
    memset(out, 0, sizeof *out);
    if (line == NULL || !tokenize(line, &t) || !dispatch(&t, out)) {
        memset(out, 0, sizeof *out);
        out->kind = UMB_CC_INVALID;
        return false;
    }
    return true;
}
