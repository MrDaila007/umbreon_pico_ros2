#include "test_runner.h"
#include "umb_settings.h"
#include "umb_crc32.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stddef.h>

TEST(test_crc32_known_vector) {
    ASSERT_EQ(umb_crc32((const uint8_t *)"123456789", 9), 0xCBF43926u);
}

TEST(test_defaults_match_table) {
    umb_settings_t d = umb_settings_defaults();
    ASSERT_EQ(d.cmd_timeout_ms, 250);
    ASSERT_EQ(d.arming_mode, UMB_ARM_BOTH);
    ASSERT_EQ(d.min_speed, 1540);
    ASSERT_EQ(d.neutral_point, 90);
    ASSERT_EQ(d.reverse_brake_ms, 650);
    ASSERT_EQ(d.agent_port, 8888);
    ASSERT_EQ(d.collision_sensor_mask, (1 << 1) | (1 << 4));
    ASSERT_FLOAT_EQ(d.pid_kp, 66.4f, 1e-4f);
}

TEST(test_sanitize_clamps_and_fixes_nan) {
    umb_settings_t s = umb_settings_defaults();
    s.cmd_timeout_ms = 5;
    s.pid_kp = NAN;
    s.max_point = 120;
    s.neutral_point = 150;
    s.pid_ki = INFINITY;
    s.max_speed_ms = 99.0f;
    s.wifi_ssid[32] = 'x';
    umb_settings_t r = umb_settings_sanitize(s);
    ASSERT_EQ(r.cmd_timeout_ms, 100);
    ASSERT_FLOAT_EQ(r.pid_kp, 66.4f, 1e-4f);
    ASSERT_FLOAT_EQ(r.pid_ki, 243.5f, 1e-3f);
    ASSERT_EQ(r.neutral_point, 120);
    ASSERT_FLOAT_EQ(r.max_speed_ms, 5.0f, 1e-6f);
    ASSERT_EQ(r.wifi_ssid[32], 0);
}

TEST(test_sanitize_point_order) {
    umb_settings_t s = umb_settings_defaults();
    s.min_point = 100;
    s.max_point = 50;
    s.neutral_point = 10;
    umb_settings_t r = umb_settings_sanitize(s);
    ASSERT_TRUE(r.min_point <= r.neutral_point);
    ASSERT_TRUE(r.neutral_point <= r.max_point);
}

TEST(test_sanitize_cap_near_below_far) {
    umb_settings_t s = umb_settings_defaults();
    s.cap_near_mm = 1200;
    s.cap_far_mm = 1000;
    umb_settings_t r = umb_settings_sanitize(s);
    ASSERT_TRUE(r.cap_near_mm < r.cap_far_mm);
}

TEST(test_set_kv_rejects_out_of_range) {
    umb_settings_t d = umb_settings_defaults(), o = d;
    ASSERT_FALSE(umb_settings_set_kv(&d, "cmd_timeout_ms", "2000", &o));
    ASSERT_TRUE(umb_settings_equal(&d, &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "cmd_timeout_ms", "300", &o));
    ASSERT_EQ(o.cmd_timeout_ms, 300);
    ASSERT_EQ(d.cmd_timeout_ms, 250);
    ASSERT_FALSE(umb_settings_set_kv(&d, "cmd_timeout_ms", "30x", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "cmd_timeout_ms", "", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "pid_kp", "nan", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "pid_kp", "-1", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "pid_kp", "70.5", &o));
    ASSERT_FLOAT_EQ(o.pid_kp, 70.5f, 1e-4f);
}

TEST(test_set_kv_cross_field_rejected) {
    umb_settings_t d = umb_settings_defaults(), o = d;
    ASSERT_FALSE(umb_settings_set_kv(&d, "neutral_point", "130", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "cap_near_mm", "1200", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "cap_near_mm", "500", &o));
}

TEST(test_set_kv_unknown_key_false) {
    umb_settings_t d = umb_settings_defaults(), o = d;
    ASSERT_FALSE(umb_settings_set_kv(&d, "no_such_key", "1", &o));
    char buf[32];
    ASSERT_FALSE(umb_settings_get_kv(&d, "no_such_key", buf, sizeof buf));
}

TEST(test_set_kv_types) {
    umb_settings_t d = umb_settings_defaults(), o = d, p;
    ASSERT_TRUE(umb_settings_set_kv(&d, "servo_reverse", "true", &o));
    ASSERT_TRUE(o.servo_reverse);
    p = o;
    ASSERT_TRUE(umb_settings_set_kv(&p, "servo_reverse", "0", &o));
    ASSERT_FALSE(o.servo_reverse);
    ASSERT_FALSE(umb_settings_set_kv(&d, "servo_reverse", "2", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "arming_mode", "ros", &o));
    ASSERT_EQ(o.arming_mode, UMB_ARM_ROS);
    ASSERT_TRUE(umb_settings_set_kv(&d, "arming_mode", "1", &o));
    ASSERT_EQ(o.arming_mode, UMB_ARM_ENCODER);
    ASSERT_FALSE(umb_settings_set_kv(&d, "arming_mode", "9", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "wifi_mode", "ap", &o));
    ASSERT_EQ(o.wifi_mode, UMB_WIFI_AP_ONLY);
    ASSERT_TRUE(umb_settings_set_kv(&d, "agent_ip", "192.168.4.2", &o));
    ASSERT_EQ(o.agent_ip, 0xC0A80402u);
    p = o;
    ASSERT_TRUE(umb_settings_set_kv(&p, "agent_ip", "discover", &o));
    ASSERT_EQ(o.agent_ip, 0);
    ASSERT_FALSE(umb_settings_set_kv(&d, "agent_ip", "1.2.3", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "agent_ip", "1.2.3.256", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "agent_ip", "1.2.3.4x", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "wifi_ssid", "MyNet", &o));
    ASSERT_TRUE(strcmp(o.wifi_ssid, "MyNet") == 0);
    ASSERT_FALSE(umb_settings_set_kv(&d, "wifi_ssid",
        "0123456789012345678901234567890123456789", &o));
    ASSERT_TRUE(umb_settings_set_kv(&d, "collision_sensor_mask", "63", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "collision_sensor_mask", "0", &o));
}

TEST(test_get_kv) {
    umb_settings_t d = umb_settings_defaults(), o;
    char buf[64];
    ASSERT_TRUE(umb_settings_get_kv(&d, "cmd_timeout_ms", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "250") == 0);
    ASSERT_TRUE(umb_settings_get_kv(&d, "servo_reverse", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "0") == 0);
    ASSERT_TRUE(umb_settings_get_kv(&d, "pid_kp", buf, sizeof buf));
    ASSERT_FLOAT_EQ(strtof(buf, NULL), 66.4f, 1e-3f);
    ASSERT_TRUE(umb_settings_get_kv(&d, "agent_ip", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "discover") == 0);
    ASSERT_TRUE(umb_settings_set_kv(&d, "agent_ip", "10.0.0.7", &o));
    ASSERT_TRUE(umb_settings_get_kv(&o, "agent_ip", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "10.0.0.7") == 0);
    ASSERT_TRUE(umb_settings_get_kv(&d, "arming_mode", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "both") == 0);
    ASSERT_TRUE(umb_settings_get_kv(&d, "reflex.battery.manual", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "pico") == 0);
    ASSERT_FALSE(umb_settings_get_kv(&d, "pid_kp", buf, 3));
    ASSERT_FALSE(umb_settings_get_kv(&d, "pid_kp", buf, 0));
}

TEST(test_get_kv_masks_password) {
    umb_settings_t d = umb_settings_defaults(), o;
    char buf[64];
    ASSERT_TRUE(umb_settings_set_kv(&d, "wifi_pass", "hunter2", &o));
    ASSERT_TRUE(umb_settings_get_kv(&o, "wifi_pass", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "***") == 0);
    ASSERT_TRUE(umb_settings_set_kv(&d, "ap_pass", "hunter22", &o));
    ASSERT_TRUE(umb_settings_get_kv(&o, "ap_pass", buf, sizeof buf));
    ASSERT_TRUE(strcmp(buf, "***") == 0);
}

TEST(test_ap_pass_must_be_wpa2_length_and_survives_defaults) {
    umb_settings_t d = umb_settings_defaults(), o;
    ASSERT_TRUE(umb_settings_set_kv(&d, "ap_pass", "synthetic-ap", &o));
    umb_settings_t before = o;
    ASSERT_FALSE(umb_settings_set_kv(&before, "ap_pass", "", &o));       /* cannot clear */
    ASSERT_FALSE(umb_settings_set_kv(&before, "ap_pass", "short77", &o)); /* < 8 */
    char long_pass[65]; memset(long_pass, 'A', 64); long_pass[64] = 0;
    ASSERT_FALSE(umb_settings_set_kv(&before, "ap_pass", long_pass, &o)); /* > 63 */
    ASSERT_TRUE(umb_settings_set_kv(&before, "wifi_pass", "", &o));      /* open STA still ok */
    before.agent_port = 9999;
    umb_settings_t reset = umb_settings_defaults_keeping_ap_pass(&before);
    ASSERT_TRUE(strcmp(reset.ap_pass, "synthetic-ap") == 0);
    ASSERT_EQ(reset.agent_port, umb_settings_defaults().agent_port);
}

TEST(test_reflex_defaults) {
    umb_settings_t d = umb_settings_defaults();
    ASSERT_EQ(d.reflex_manual[UMB_REFLEX_BATTERY], UMB_RX_PICO);
    ASSERT_EQ(d.reflex_manual[UMB_REFLEX_COLLISION], UMB_RX_WARN);
    for (int i = 0; i < UMB_REFLEX_COUNT; i++) {
        ASSERT_EQ(d.reflex_auto[i], UMB_RX_PICO);
        if (i != UMB_REFLEX_BATTERY) ASSERT_EQ(d.reflex_manual[i], UMB_RX_WARN);
    }
}

TEST(test_set_kv_reflex_action) {
    umb_settings_t d = umb_settings_defaults(), o = d;
    ASSERT_TRUE(umb_settings_set_kv(&d, "reflex.collision.manual", "ros", &o));
    ASSERT_EQ(o.reflex_manual[UMB_REFLEX_COLLISION], UMB_RX_ROS);
    ASSERT_TRUE(umb_settings_set_kv(&d, "reflex.sensor_fault.auto", "off", &o));
    ASSERT_EQ(o.reflex_auto[UMB_REFLEX_SENSOR_FAULT], UMB_RX_OFF);
    ASSERT_TRUE(umb_settings_set_kv(&d, "reflex.speed_cap.auto", "warn", &o));
    ASSERT_EQ(o.reflex_auto[UMB_REFLEX_SPEED_CAP], UMB_RX_WARN);
    ASSERT_FALSE(umb_settings_set_kv(&d, "reflex.collision.manual", "maybe", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "reflex.bogus.manual", "pico", &o));
    ASSERT_FALSE(umb_settings_set_kv(&d, "reflex.collision.sideways", "pico", &o));
}

TEST(test_sanitize_idempotent_on_defaults) {
    umb_settings_t d = umb_settings_defaults();
    umb_settings_t r = umb_settings_sanitize(d);
    ASSERT_TRUE(umb_settings_equal(&d, &r));
}

TEST(test_encode_decode_roundtrip) {
    umb_settings_t d = umb_settings_defaults(), o;
    uint8_t buf[UMB_SETTINGS_SLOT_SIZE];
    uint32_t seq = 0;
    size_t n = umb_settings_encode(&d, 7, buf, sizeof buf);
    ASSERT_TRUE(n > 0 && n <= UMB_SETTINGS_SLOT_SIZE);
    ASSERT_TRUE(umb_settings_decode(buf, n, &o, &seq));
    ASSERT_EQ(seq, 7);
    ASSERT_TRUE(umb_settings_equal(&d, &o));
    ASSERT_EQ(umb_settings_encode(&d, 7, buf, n - 1), 0);
    ASSERT_FALSE(umb_settings_decode(buf, n - 1, &o, &seq));
    ASSERT_FALSE(umb_settings_decode(buf, 4, &o, &seq));
}

static size_t enc(uint32_t seq, uint8_t *buf) {
    umb_settings_t d = umb_settings_defaults();
    memset(buf, 0xFF, UMB_SETTINGS_SLOT_SIZE);
    return umb_settings_encode(&d, seq, buf, UMB_SETTINGS_SLOT_SIZE);
}

TEST(test_decode_rejects_bad_crc) {
    uint8_t b[UMB_SETTINGS_SLOT_SIZE]; umb_settings_t o; uint32_t s;
    size_t n = enc(1, b);
    b[20] ^= 0x01;
    ASSERT_FALSE(umb_settings_decode(b, n, &o, &s));
}

TEST(test_decode_rejects_bad_magic) {
    uint8_t b[UMB_SETTINGS_SLOT_SIZE]; umb_settings_t o; uint32_t s;
    size_t n = enc(1, b);
    b[0] ^= 0x01;
    ASSERT_FALSE(umb_settings_decode(b, n, &o, &s));
}

static void fix_crc(uint8_t *b, size_t n) {
    uint32_t c = umb_crc32(b, n - 4);
    b[n - 4] = (uint8_t)c; b[n - 3] = (uint8_t)(c >> 8);
    b[n - 2] = (uint8_t)(c >> 16); b[n - 1] = (uint8_t)(c >> 24);
}

TEST(test_decode_rejects_other_version) {
    uint8_t b[UMB_SETTINGS_SLOT_SIZE]; umb_settings_t o; uint32_t s;
    size_t n = enc(1, b);
    b[4] = 2;
    fix_crc(b, n);   /* valid CRC, wrong version */
    ASSERT_FALSE(umb_settings_decode(b, n, &o, &s));
}

TEST(test_decode_rejects_wrong_payload_len) {
    uint8_t b[UMB_SETTINGS_SLOT_SIZE]; umb_settings_t o; uint32_t s;
    size_t n = enc(1, b);
    b[6] ^= 0x01;
    fix_crc(b, n);
    ASSERT_FALSE(umb_settings_decode(b, n, &o, &s));
}

TEST(test_decode_sanitizes_out_of_range) {
    umb_settings_t d = umb_settings_defaults(), o;
    uint8_t b[UMB_SETTINGS_SLOT_SIZE]; uint32_t s;
    d.cmd_timeout_ms = 5;
    size_t n = umb_settings_encode(&d, 1, b, sizeof b);
    ASSERT_TRUE(umb_settings_decode(b, n, &o, &s));
    ASSERT_EQ(o.cmd_timeout_ms, 100);
}

TEST(test_pick_slot_newest_valid_wins) {
    uint8_t a[UMB_SETTINGS_SLOT_SIZE], b[UMB_SETTINGS_SLOT_SIZE];
    umb_settings_t o; uint32_t s = 0;
    enc(3, a); enc(4, b);
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), 1);
    ASSERT_EQ(s, 4);
    ASSERT_EQ(umb_settings_pick_slot(b, a, sizeof a, &o, &s), 0);
}

TEST(test_pick_slot_falls_back_when_newest_corrupt) {
    uint8_t a[UMB_SETTINGS_SLOT_SIZE], b[UMB_SETTINGS_SLOT_SIZE];
    umb_settings_t o; uint32_t s = 0;
    enc(3, a); enc(4, b);
    b[30] ^= 0x10;
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), 0);
    ASSERT_EQ(s, 3);
    b[30] ^= 0x10; a[30] ^= 0x10;
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), 1);
}

TEST(test_pick_slot_erased_flash_gives_defaults) {
    uint8_t a[UMB_SETTINGS_SLOT_SIZE], b[UMB_SETTINGS_SLOT_SIZE];
    umb_settings_t o, d = umb_settings_defaults(); uint32_t s = 99;
    memset(a, 0xFF, sizeof a); memset(b, 0xFF, sizeof b);
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), -1);
    ASSERT_TRUE(umb_settings_equal(&o, &d));
    ASSERT_EQ(s, 0);
}

TEST(test_pick_slot_seq_wrap) {
    uint8_t a[UMB_SETTINGS_SLOT_SIZE], b[UMB_SETTINGS_SLOT_SIZE];
    umb_settings_t o; uint32_t s = 0;
    enc(0xFFFFFFFFu, a); enc(0, b);
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), 1);
    ASSERT_EQ(s, 0);
}

TEST(test_pick_slot_equal_seq_prefers_slot0) {
    uint8_t a[UMB_SETTINGS_SLOT_SIZE], b[UMB_SETTINGS_SLOT_SIZE];
    umb_settings_t o; uint32_t s = 0;
    enc(5, a); enc(5, b);
    ASSERT_EQ(umb_settings_pick_slot(a, b, sizeof a, &o, &s), 0);
}

TEST(test_padding_garbage_does_not_matter) {
    umb_settings_t d = umb_settings_defaults(), g, o;
    umb_settings_t c = umb_settings_canonical(&d);
    memset(&g, 0xAA, sizeof g);
    uint8_t *gp = (uint8_t *)&g;
    const uint8_t *cp = (const uint8_t *)&c;
    for (size_t i = 0; i < sizeof g; i++) {
        if (i == offsetof(umb_settings_t, arming_mode) + 1 ||
            (i >= offsetof(umb_settings_t, servo_reverse) + 1 &&
             i < offsetof(umb_settings_t, pid_kp))) continue;   /* leave poison in padding */
        gp[i] = cp[i];
    }
    ASSERT_TRUE(umb_settings_equal(&d, &g));
    ASSERT_TRUE(umb_settings_set_kv(&g, "cmd_timeout_ms", "250", &o));
    uint8_t b1[UMB_SETTINGS_SLOT_SIZE], b2[UMB_SETTINGS_SLOT_SIZE];
    size_t n1 = umb_settings_encode(&d, 1, b1, sizeof b1);
    size_t n2 = umb_settings_encode(&g, 1, b2, sizeof b2);
    ASSERT_EQ(n1, n2);
    ASSERT_TRUE(memcmp(b1, b2, n1) == 0);
}

int main(void) {
    RUN_TEST(test_padding_garbage_does_not_matter);
    RUN_TEST(test_crc32_known_vector);
    RUN_TEST(test_defaults_match_table);
    RUN_TEST(test_sanitize_clamps_and_fixes_nan);
    RUN_TEST(test_sanitize_point_order);
    RUN_TEST(test_sanitize_cap_near_below_far);
    RUN_TEST(test_set_kv_rejects_out_of_range);
    RUN_TEST(test_set_kv_cross_field_rejected);
    RUN_TEST(test_set_kv_unknown_key_false);
    RUN_TEST(test_set_kv_types);
    RUN_TEST(test_get_kv);
    RUN_TEST(test_get_kv_masks_password);
    RUN_TEST(test_ap_pass_must_be_wpa2_length_and_survives_defaults);
    RUN_TEST(test_reflex_defaults);
    RUN_TEST(test_set_kv_reflex_action);
    RUN_TEST(test_sanitize_idempotent_on_defaults);
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_decode_rejects_bad_crc);
    RUN_TEST(test_decode_rejects_bad_magic);
    RUN_TEST(test_decode_rejects_other_version);
    RUN_TEST(test_decode_rejects_wrong_payload_len);
    RUN_TEST(test_decode_sanitizes_out_of_range);
    RUN_TEST(test_pick_slot_newest_valid_wins);
    RUN_TEST(test_pick_slot_falls_back_when_newest_corrupt);
    RUN_TEST(test_pick_slot_erased_flash_gives_defaults);
    RUN_TEST(test_pick_slot_seq_wrap);
    RUN_TEST(test_pick_slot_equal_seq_prefers_slot0);
    TEST_SUMMARY();
}
