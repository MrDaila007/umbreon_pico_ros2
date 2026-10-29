/* console_cmds.c - see console_cmds.h. Parsing is core/umb_console_parse;
 * every decision about motion is core/umb_control on core 1. */
#include "console_cmds.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"
#include "pico/stdio_usb.h"
#include "hal_flash_store.h"
#include "shared_state.h"
#include "settings_apply.h"
#include "umb_console_parse.h"
#include "umb_gear.h"
#include "umb_reflex.h"
#include "umb_safety.h"
#include "umb_settings.h"
#include "umb_tof_array.h"

#define ARM_SETTLE_MS 60u      /* > one control period: let core 1 apply the request */
#define REBOOT_FLUSH_MS 50u
#define CAL_MAX_SPEED_MPS 0.05f   /* `cal imu` only while the car is (nearly) still */

static const char *s_reset_reason = "unknown";

static const char *const ARM_MODE_NAMES[] = { "none", "encoder", "ros", "both" };
static const char *const GEAR_NAMES[] = { "neutral", "forward", "wait_stop", "brake", "gap", "reverse" };
static const char *const TOF_LABELS[UMB_TOF_COUNT] = { "HR", "FR", "R", "L", "FL", "HL" };
static const char *const ENCODER_STATES[] = { "starting", "ok", "FAILED (keys disabled)" };
static const char *const STACK_TASKS[] = { "control", "console", "imu", "tof", "ui" };
static const char *const STOP_NAMES[] = { "cmd_timeout", "agent", "wifi", "reflex", "core0", "no_cmd", "maintenance" };

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
    int enc = shared_encoder_state();
    printf("encoder: %s\n", (enc >= 0 && enc <= SHARED_ENC_FAILED) ? ENCODER_STATES[enc] : "?");
    printf("reset: %s\n", s_reset_reason);
}

/* Minimum free stack ever seen per task, in words (uxTaskGetStackHighWaterMark). */
static void print_stack_free(void) {
    printf("stack free (words):");
    for (size_t i = 0; i < sizeof STACK_TASKS / sizeof STACK_TASKS[0]; i++) {
        TaskHandle_t h = xTaskGetHandle(STACK_TASKS[i]);
        if (h) printf(" %s %lu", STACK_TASKS[i], (unsigned long)uxTaskGetStackHighWaterMark(h));
        else printf(" %s ?", STACK_TASKS[i]);
    }
    printf("\n");
}

static void print_sensors(void) {
    shared_sensors_t sn;
    shared_get_sensors(&sn);
    printf("tof mm:");
    for (int i = 0; i < UMB_TOF_COUNT; i++) printf(" %s %d", TOF_LABELS[i], (int)sn.tof_mm[i]);
    printf("\ntof: online %u/%d, recovering %s, restarts %lu\n", (unsigned)sn.tof_online, UMB_TOF_COUNT,
           yes_no(sn.tof_recovering), (unsigned long)sn.tof_restarts);
    printf("accel g: %.3f %.3f %.3f\n", (double)sn.accel_g[0], (double)sn.accel_g[1], (double)sn.accel_g[2]);
    printf("gyro dps: %.2f %.2f %.2f\n", (double)sn.gyro_dps[0], (double)sn.gyro_dps[1], (double)sn.gyro_dps[2]);
    printf("yaw rate: %.2f dps, heading: %.1f deg\n", (double)sn.yaw_rate_dps, (double)sn.heading_deg);
    printf("imu: ok %s, calibrated %s\n", yes_no(sn.imu_ok), yes_no(sn.imu_calibrated));
    if (sn.bat_have) printf("battery: %.2f V (min %.2f V)\n", (double)sn.bat_v, (double)sn.bat_v_min);
    else printf("battery: no sample\n");
    print_stack_free();
}

static void request_imu_cal(void) {
    umb_outputs_t o;
    shared_get_outputs(&o);
    if (o.armed || !shared_settings_write_allowed()) { printf("refused: armed\n"); return; }
    if (!(fabsf(o.speed_signed) < CAL_MAX_SPEED_MPS)) { printf("refused: moving\n"); return; }
    shared_request_imu_cal();
    printf("ok: calibrating, keep still\n");
}

static void print_help(void) {
    printf("commands:\n"
           "  status | help | sensors\n"
           "  cal imu                                    gyro bias calibration (disarmed, still)\n"
           "  raw <steer -1..1> <throttle -1..1> [ms]   manual, open loop (repeats every 20 ms)\n"
           "  speed <steer> <m/s> [ms]                   manual, closed loop\n"
           "  auto <steer> <m/s> [ms]                    autonomous source\n"
           "  stop                                       neutral now, cancels repetition\n"
           "  arm car | arm ros | disarm\n"
           "  mode none|encoder|ros|both\n"
           "  get <key> | set <key> <value> | defaults | save\n"
           "  wifi set <ssid> <pass> | agent set <ip|discover> <port>\n"
           "  reboot\n"
           "set/defaults/wifi/agent/save require disarmed and idle;\n"
           "save is also refused while armed; mode is always allowed (it disarms)\n"
           "[ms] defaults to 1000, max 3000\n");
}

static umb_cmd_t neutral_cmd(uint32_t now_ms) {
    umb_cmd_t c = { UMB_CMD_RAW, UMB_SRC_CONSOLE, UMB_DRIVE_MANUAL, 0.0f, 0.0f, now_ms };
    return c;
}

static void start_drive(const umb_console_cmd_t *cc, console_origin_t origin,
                        console_repeat_t *rep, uint32_t now_ms) {
    /* Never start a repeat nobody can stop: a USB line racing a disconnect. */
    if (origin == CONSOLE_ORIGIN_USB && !stdio_usb_connected()) {
        printf("refused: usb disconnected\n");
        return;
    }
    umb_cmd_t c = neutral_cmd(now_ms);
    c.kind = (cc->kind == UMB_CC_RAW) ? UMB_CMD_RAW : UMB_CMD_SPEED;
    c.mode = (cc->kind == UMB_CC_AUTO) ? UMB_DRIVE_AUTO : UMB_DRIVE_MANUAL;
    c.steer = cc->steer;
    c.value = cc->value;
    if (!shared_post_cmd(&c)) { printf("refused: busy\n"); return; }
    rep->cmd = c;
    rep->origin = origin;
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
    if (!shared_post_arm_request(kind)) { printf("refused: busy\n"); return; }
    vTaskDelay(pdMS_TO_TICKS(ARM_SETTLE_MS));
    umb_outputs_t o;
    umb_arming_t a;
    shared_get_outputs(&o);
    shared_get_arming(&a);
    printf("armed: %s (car key %s, ros key %s; motion needs a neutral command first)\n",
           yes_no(o.armed), yes_no(a.car_key), yes_no(a.ros_key));
}

/* True while a fresh command stream is being obeyed (no stop bit, motion allowed). */
static bool driving(void) {
    umb_outputs_t o;
    shared_get_outputs(&o);
    return o.motion_allowed && o.stop_mask == 0u;
}

/* Applies up to two key=value edits to a copy of the settings and publishes it.
 * All ordinary edits require disarmed state; `mode` is a disarming-only transaction. */
static void edit_settings(bool gated, const char *k1, const char *v1, const char *k2, const char *v2) {
    if (gated && driving()) { printf("refused: driving\n"); return; }
    umb_settings_t s;
    uint32_t revision = shared_settings_snapshot(&s);
    if (!umb_settings_set_kv(&s, k1, v1, &s) || (k2 && !umb_settings_set_kv(&s, k2, v2, &s))) {
        printf("error: invalid key or value\n");
        return;
    }
    if (!shared_commit_settings(&s, revision, !gated)) {
        printf("refused: armed or busy\n"); return;
    }
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
    if (driving()) { printf("refused: driving\n"); return; }
    umb_settings_t current;
    uint32_t revision = shared_settings_snapshot(&current);
    umb_settings_t d = umb_settings_defaults();
    if (!shared_commit_settings(&d, revision, false)) {
        printf("refused: armed or busy\n"); return;
    }
    printf("ok: defaults loaded (not saved; run save)\n");
}

static void save_settings(void) {
    if (driving()) { printf("refused: driving\n"); return; }
    switch (settings_save_current()) {
    case SETTINGS_SAVE_BUSY: printf("refused: armed or busy\n"); break;
    case SETTINGS_SAVE_NEUTRAL_TIMEOUT: printf("error: neutral acknowledgement timeout\n"); break;
    case SETTINGS_SAVE_FLASH_FAILED: printf("error: save failed\n"); break;
    case SETTINGS_SAVE_OK:
        printf("ok: saved to slot %d (seq %lu)\n", hal_settings_slot(), (unsigned long)hal_settings_seq());
        break;
    }
}

static void reboot_now(void) {
    printf("rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(REBOOT_FLUSH_MS));
    watchdog_reboot(0, 0, 0);
    for (;;) vTaskDelay(portMAX_DELAY);
}

void console_execute(const char *line, console_origin_t origin, console_repeat_t *rep, uint32_t now_ms) {
    umb_console_cmd_t cc;
    if (!umb_console_parse(line, &cc)) { printf("error: invalid command (try help)\n"); return; }
    switch (cc.kind) {
    case UMB_CC_HELP:      print_help(); break;
    case UMB_CC_STATUS:    print_status(); break;
    case UMB_CC_SENSORS:   print_sensors(); break;
    case UMB_CC_CAL_IMU:   request_imu_cal(); break;
    case UMB_CC_RAW:
    case UMB_CC_SPEED:
    case UMB_CC_AUTO:      start_drive(&cc, origin, rep, now_ms); break;
    case UMB_CC_STOP:      stop_now(rep, now_ms); printf("ok\n"); break;
    case UMB_CC_ARM_CAR:   arm_request(SHARED_ARM_CAR); break;
    case UMB_CC_ARM_ROS:   arm_request(SHARED_ARM_ROS); break;
    case UMB_CC_DISARM:    rep->active = false; arm_request(SHARED_ARM_DISARM); break;
    case UMB_CC_MODE:      edit_settings(false, "arming_mode", arm_mode_name(cc.mode), NULL, NULL); break;
    case UMB_CC_GET:       get_setting(cc.key); break;
    case UMB_CC_SET:       edit_settings(true, cc.key, cc.arg1, NULL, NULL); break;
    case UMB_CC_SAVE:      save_settings(); break;
    case UMB_CC_DEFAULTS:  restore_defaults(); break;
    case UMB_CC_REBOOT:    reboot_now(); break;
    case UMB_CC_WIFI_SET:  edit_settings(true, "wifi_ssid", cc.arg1, "wifi_pass", cc.arg2); break;
    case UMB_CC_AGENT_SET: edit_settings(true, "agent_ip", cc.arg1, "agent_port", cc.arg2); break;
    default:               printf("error: invalid command (try help)\n"); break;
    }
}
