/* console_cmds.c - see console_cmds.h. Parsing is core/umb_console_parse;
 * every decision about motion is core/umb_control on core 1. */
#include "console_cmds.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"
#include "hal_flash_store.h"
#include "shared_state.h"
#include "umb_console_parse.h"
#include "umb_gear.h"
#include "umb_reflex.h"
#include "umb_safety.h"
#include "umb_settings.h"

#define ARM_SETTLE_MS 60u      /* > one control period: let core 1 apply the request */
#define REBOOT_FLUSH_MS 50u

static const char *s_reset_reason = "unknown";

static const char *const ARM_MODE_NAMES[] = { "none", "encoder", "ros", "both" };
static const char *const GEAR_NAMES[] = { "neutral", "forward", "wait_stop", "brake", "gap", "reverse" };
static const char *const STOP_NAMES[] = { "cmd_timeout", "agent", "wifi", "reflex", "core0", "no_cmd" };

void console_cmds_init(const char *reset_reason) {
    if (reset_reason) s_reset_reason = reset_reason;
}

static const char *yes_no(bool v) { return v ? "yes" : "no"; }

static const char *arm_mode_name(unsigned m) {
    return m < 4u ? ARM_MODE_NAMES[m] : "?";
}

static void print_stop_bits(uint32_t mask) {
    printf("stop:");
    if (mask == 0u) printf(" none");
    for (unsigned i = 0; i < sizeof STOP_NAMES / sizeof STOP_NAMES[0]; i++) {
        if (mask & (1u << i)) printf(" %s", STOP_NAMES[i]);
    }
    printf("\n");
}

static void print_reflex_mask(const char *label, uint32_t mask) {
    printf(" %s", label);
    if (mask == 0u) printf(" none");
    for (int i = 0; i < UMB_REFLEX_COUNT; i++) {
        if (mask & (1u << i)) printf(" %s", umb_reflex_name((umb_reflex_id_t)i));
    }
}

static void print_status(void) {
    umb_outputs_t o;
    umb_arming_t a;
    umb_settings_t s;
    bool wifi, agent;
    shared_get_outputs(&o);
    shared_get_arming(&a);
    shared_get_settings(&s);
    shared_get_link(&wifi, &agent);

    if (s.arming_mode == UMB_ARM_NONE)
        printf("WARNING: NO ARMING (mode none): any non-neutral command moves the car\n");
    printf("outputs: servo %u us, esc %u us, motion %s, speed %.2f m/s\n",
           (unsigned)o.servo_us, (unsigned)o.esc_us, yes_no(o.motion_allowed), (double)o.speed_signed);
    printf("armed: %s (mode %s, car key %s, ros key %s, neutral seen %s)\n", yes_no(o.armed),
           arm_mode_name(a.mode), yes_no(a.car_key), yes_no(a.ros_key), yes_no(a.neutral_seen));
    print_stop_bits(o.stop_mask);
    printf("gear: %s, drive mode: %s\n", o.gear <= UMB_GEAR_REVERSE ? GEAR_NAMES[o.gear] : "?",
           o.mode == UMB_DRIVE_AUTO ? "auto" : "manual");
    printf("reflex:");
    print_reflex_mask("detected", o.reflex.detected_mask);
    printf(",");
    print_reflex_mask("intervening", o.reflex.intervene_mask);
    printf("\n");
    if (hal_settings_slot() < 0) printf("settings: defaults\n");
    else printf("settings: slot %d (seq %lu)\n", hal_settings_slot(), (unsigned long)hal_settings_seq());
    printf("link: wifi %s, agent %s\n", wifi ? "up" : "down", agent ? "up" : "down");
    printf("reset: %s\n", s_reset_reason);
}

static void print_help(void) {
    printf("commands:\n"
           "  status | help\n"
           "  raw <steer -1..1> <throttle -1..1> [ms]   manual, open loop (repeats every 20 ms)\n"
           "  speed <steer> <m/s> [ms]                   manual, closed loop\n"
           "  auto <steer> <m/s> [ms]                    autonomous source\n"
           "  stop                                       neutral now, cancels repetition\n"
           "  arm car | arm ros | disarm\n"
           "  mode none|encoder|ros|both\n"
           "  get <key> | set <key> <value> | defaults | save\n"
           "  wifi set <ssid> <pass> | agent set <ip|discover> <port>\n"
           "  reboot\n"
           "settings edits and save are refused while armed; [ms] defaults to 1000, max 3000\n");
}

static umb_cmd_t neutral_cmd(uint32_t now_ms) {
    umb_cmd_t c = { UMB_CMD_RAW, UMB_SRC_CONSOLE, UMB_DRIVE_MANUAL, 0.0f, 0.0f, now_ms };
    return c;
}

static void start_drive(const umb_console_cmd_t *cc, console_repeat_t *rep, uint32_t now_ms) {
    umb_cmd_t c = neutral_cmd(now_ms);
    c.kind = (cc->kind == UMB_CC_RAW) ? UMB_CMD_RAW : UMB_CMD_SPEED;
    c.mode = (cc->kind == UMB_CC_AUTO) ? UMB_DRIVE_AUTO : UMB_DRIVE_MANUAL;
    c.steer = cc->steer;
    c.value = cc->value;
    shared_post_cmd(&c);
    rep->cmd = c;
    rep->active = true;
    rep->end_ms = now_ms + cc->duration_ms;
    rep->next_ms = now_ms + CONSOLE_REPEAT_MS;
    umb_outputs_t o;
    shared_get_outputs(&o);
    printf("ok: %u ms\n", (unsigned)cc->duration_ms);
    if (!o.armed) printf("note: disarmed, outputs stay neutral\n");
}

static void stop_now(console_repeat_t *rep, uint32_t now_ms) {
    umb_cmd_t c = neutral_cmd(now_ms);
    rep->active = false;
    shared_post_cmd(&c);
}

static void arm_request(int kind) {
    shared_post_arm_request(kind);
    vTaskDelay(pdMS_TO_TICKS(ARM_SETTLE_MS));
    umb_outputs_t o;
    umb_arming_t a;
    shared_get_outputs(&o);
    shared_get_arming(&a);
    printf("armed: %s (car key %s, ros key %s; motion needs a neutral command first)\n",
           yes_no(o.armed), yes_no(a.car_key), yes_no(a.ros_key));
}

/* Applies up to two key=value edits to a copy of the settings and publishes it. */
static void edit_settings(const char *k1, const char *v1, const char *k2, const char *v2) {
    if (!shared_settings_write_allowed()) { printf("refused: armed\n"); return; }
    umb_settings_t s;
    shared_get_settings(&s);
    if (!umb_settings_set_kv(&s, k1, v1, &s) || (k2 && !umb_settings_set_kv(&s, k2, v2, &s))) {
        printf("error: invalid key or value\n");
        return;
    }
    shared_set_settings(&s);
    printf("ok (not saved; run save)\n");
}

static void get_setting(const char *key) {
    umb_settings_t s;
    char buf[80];
    shared_get_settings(&s);
    if (umb_settings_get_kv(&s, key, buf, sizeof buf)) printf("%s = %s\n", key, buf);
    else printf("error: unknown key\n");
}

static void restore_defaults(void) {
    if (!shared_settings_write_allowed()) { printf("refused: armed\n"); return; }
    umb_settings_t d = umb_settings_defaults();
    shared_set_settings(&d);
    printf("ok: defaults loaded (not saved; run save)\n");
}

static void save_settings(void) {
    if (!shared_settings_write_allowed()) { printf("refused: armed\n"); return; }
    umb_settings_t s;
    shared_get_settings(&s);
    if (hal_settings_save(&s))
        printf("ok: saved to slot %d (seq %lu)\n", hal_settings_slot(), (unsigned long)hal_settings_seq());
    else printf("error: save failed\n");
}

static void reboot_now(void) {
    printf("rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(REBOOT_FLUSH_MS));
    watchdog_reboot(0, 0, 0);
    for (;;) vTaskDelay(portMAX_DELAY);
}

void console_execute(const char *line, console_repeat_t *rep, uint32_t now_ms) {
    umb_console_cmd_t cc;
    if (!umb_console_parse(line, &cc)) { printf("error: invalid command (try help)\n"); return; }
    switch (cc.kind) {
    case UMB_CC_HELP:      print_help(); break;
    case UMB_CC_STATUS:    print_status(); break;
    case UMB_CC_RAW:
    case UMB_CC_SPEED:
    case UMB_CC_AUTO:      start_drive(&cc, rep, now_ms); break;
    case UMB_CC_STOP:      stop_now(rep, now_ms); printf("ok\n"); break;
    case UMB_CC_ARM_CAR:   arm_request(SHARED_ARM_CAR); break;
    case UMB_CC_ARM_ROS:   arm_request(SHARED_ARM_ROS); break;
    case UMB_CC_DISARM:    rep->active = false; arm_request(SHARED_ARM_DISARM); break;
    case UMB_CC_MODE:      edit_settings("arming_mode", arm_mode_name(cc.mode), NULL, NULL); break;
    case UMB_CC_GET:       get_setting(cc.key); break;
    case UMB_CC_SET:       edit_settings(cc.key, cc.arg1, NULL, NULL); break;
    case UMB_CC_SAVE:      save_settings(); break;
    case UMB_CC_DEFAULTS:  restore_defaults(); break;
    case UMB_CC_REBOOT:    reboot_now(); break;
    case UMB_CC_WIFI_SET:  edit_settings("wifi_ssid", cc.arg1, "wifi_pass", cc.arg2); break;
    case UMB_CC_AGENT_SET: edit_settings("agent_ip", cc.arg1, "agent_port", cc.arg2); break;
    default:               printf("error: invalid command (try help)\n"); break;
    }
}
