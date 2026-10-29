#include "test_runner.h"
#include "umb_console_parse.h"
#include <stdio.h>
#include <string.h>

static bool parse(const char *line, umb_console_cmd_t *c) { return umb_console_parse(line, c); }

static bool is_invalid(const char *line) {
    umb_console_cmd_t c;
    memset(&c, 0xA5, sizeof c);
    bool ok = parse(line, &c);
    umb_console_cmd_t z;
    memset(&z, 0, sizeof z);
    return !ok && c.kind == UMB_CC_INVALID && memcmp(&c, &z, sizeof c) == 0;
}

TEST(test_parse_raw_default_duration) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("raw 0.2 0.3", &c));
    ASSERT_EQ(c.kind, UMB_CC_RAW);
    ASSERT_FLOAT_EQ(c.steer, 0.2f, 1e-6f);
    ASSERT_FLOAT_EQ(c.value, 0.3f, 1e-6f);
    ASSERT_EQ(c.duration_ms, 1000);
}

TEST(test_parse_raw_duration_capped) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("raw 0 0.1 9000", &c));
    ASSERT_EQ(c.duration_ms, 3000);
    ASSERT_TRUE(parse("raw 0 0.1 99999999999999999999", &c));
    ASSERT_EQ(c.duration_ms, 3000);
    ASSERT_TRUE(parse("raw 0 0.1 3000", &c));
    ASSERT_EQ(c.duration_ms, 3000);
    ASSERT_TRUE(parse("raw 0 0.1 1", &c));
    ASSERT_EQ(c.duration_ms, 1);
    ASSERT_TRUE(parse("raw 0 0.1 250", &c));
    ASSERT_EQ(c.duration_ms, 250);
}

TEST(test_parse_duration_invalid) {
    ASSERT_TRUE(is_invalid("raw 0 0.1 0"));
    ASSERT_TRUE(is_invalid("raw 0 0.1 -5"));
    ASSERT_TRUE(is_invalid("raw 0 0.1 abc"));
    ASSERT_TRUE(is_invalid("raw 0 0.1 1.5"));
    ASSERT_TRUE(is_invalid("raw 0 0.1 100 extra"));
}

TEST(test_parse_raw_rejects_out_of_range) {
    umb_console_cmd_t c;
    ASSERT_FALSE(parse("raw 0 1.5", &c));
    ASSERT_EQ(c.kind, UMB_CC_INVALID);
    ASSERT_TRUE(is_invalid("raw 1.5 0"));
    ASSERT_TRUE(is_invalid("raw -1.01 0"));
    ASSERT_TRUE(is_invalid("raw 0 -1.01"));
    ASSERT_TRUE(parse("raw -1 1", &c));
}

TEST(test_parse_numbers_strict) {
    ASSERT_TRUE(is_invalid("raw 0"));
    ASSERT_TRUE(is_invalid("raw 0 0.1x"));
    ASSERT_TRUE(is_invalid("raw nan 0"));
    ASSERT_TRUE(is_invalid("raw 0 inf"));
    ASSERT_TRUE(is_invalid("raw 0 0 1000 1"));
    ASSERT_TRUE(is_invalid("raw abc 0"));
}

TEST(test_parse_speed_command) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("speed -0.5 2.5 500", &c));
    ASSERT_EQ(c.kind, UMB_CC_SPEED);
    ASSERT_FLOAT_EQ(c.steer, -0.5f, 1e-6f);
    ASSERT_FLOAT_EQ(c.value, 2.5f, 1e-6f);
    ASSERT_EQ(c.duration_ms, 500);
    ASSERT_TRUE(parse("speed 0 -5", &c));
    ASSERT_FLOAT_EQ(c.value, -5.0f, 1e-6f);
    ASSERT_TRUE(is_invalid("speed 0 5.1"));
    ASSERT_TRUE(is_invalid("speed 0 -5.1"));
    ASSERT_TRUE(is_invalid("speed 2 1"));
}

TEST(test_parse_auto_command) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("auto 0 0.4", &c));
    ASSERT_EQ(c.kind, UMB_CC_AUTO);
    ASSERT_FLOAT_EQ(c.value, 0.4f, 1e-6f);
    ASSERT_EQ(c.duration_ms, 1000);
    ASSERT_TRUE(is_invalid("auto 0 6"));
}

TEST(test_parse_simple_keywords) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("help", &c));     ASSERT_EQ(c.kind, UMB_CC_HELP);
    ASSERT_TRUE(parse("status", &c));   ASSERT_EQ(c.kind, UMB_CC_STATUS);
    ASSERT_TRUE(parse("stop", &c));     ASSERT_EQ(c.kind, UMB_CC_STOP);
    ASSERT_TRUE(parse("disarm", &c));   ASSERT_EQ(c.kind, UMB_CC_DISARM);
    ASSERT_TRUE(parse("save", &c));     ASSERT_EQ(c.kind, UMB_CC_SAVE);
    ASSERT_TRUE(parse("defaults", &c)); ASSERT_EQ(c.kind, UMB_CC_DEFAULTS);
    ASSERT_TRUE(parse("reboot", &c));   ASSERT_EQ(c.kind, UMB_CC_REBOOT);
    ASSERT_TRUE(is_invalid("stop now"));
    ASSERT_TRUE(is_invalid("reboot 1"));
}

TEST(test_parse_arm) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("arm car", &c)); ASSERT_EQ(c.kind, UMB_CC_ARM_CAR);
    ASSERT_TRUE(parse("arm ros", &c)); ASSERT_EQ(c.kind, UMB_CC_ARM_ROS);
    ASSERT_TRUE(is_invalid("arm"));
    ASSERT_TRUE(is_invalid("arm both"));
    ASSERT_TRUE(is_invalid("arm car x"));
}

TEST(test_parse_mode_values) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("mode none", &c));    ASSERT_EQ(c.kind, UMB_CC_MODE); ASSERT_EQ(c.mode, UMB_ARM_NONE);
    ASSERT_TRUE(parse("mode encoder", &c)); ASSERT_EQ(c.mode, UMB_ARM_ENCODER);
    ASSERT_TRUE(parse("mode ros", &c));     ASSERT_EQ(c.mode, UMB_ARM_ROS);
    ASSERT_TRUE(parse("mode both", &c));    ASSERT_EQ(c.mode, UMB_ARM_BOTH);
    ASSERT_TRUE(is_invalid("mode"));
    ASSERT_TRUE(is_invalid("mode fast"));
    ASSERT_TRUE(is_invalid("mode both x"));
}

TEST(test_parse_set_key_value) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("set cmd_timeout_ms 300", &c));
    ASSERT_EQ(c.kind, UMB_CC_SET);
    ASSERT_TRUE(strcmp(c.key, "cmd_timeout_ms") == 0);
    ASSERT_TRUE(strcmp(c.arg1, "300") == 0);
    ASSERT_TRUE(parse("get cmd_timeout_ms", &c));
    ASSERT_EQ(c.kind, UMB_CC_GET);
    ASSERT_TRUE(strcmp(c.key, "cmd_timeout_ms") == 0);
    ASSERT_TRUE(is_invalid("set a"));
    ASSERT_TRUE(is_invalid("get"));
    ASSERT_TRUE(is_invalid("get a b"));
    ASSERT_TRUE(is_invalid("set a b c"));
}

TEST(test_parse_set_length_limits) {
    char line[200];
    umb_console_cmd_t c;
    char k31[32], k32[33], v64[65], v65[66];
    memset(k31, 'k', 31); k31[31] = 0;
    memset(k32, 'k', 32); k32[32] = 0;
    memset(v64, 'v', 64); v64[64] = 0;
    memset(v65, 'v', 65); v65[65] = 0;
    snprintf(line, sizeof line, "set %s %s", k31, v64);
    ASSERT_TRUE(parse(line, &c));
    ASSERT_TRUE(strcmp(c.key, k31) == 0);
    ASSERT_TRUE(strcmp(c.arg1, v64) == 0);
    snprintf(line, sizeof line, "set %s x", k32);
    ASSERT_TRUE(is_invalid(line));
    snprintf(line, sizeof line, "set k %s", v65);
    ASSERT_TRUE(is_invalid(line));
    snprintf(line, sizeof line, "get %s", k32);
    ASSERT_TRUE(is_invalid(line));
    snprintf(line, sizeof line, "get %s", k31);
    ASSERT_TRUE(parse(line, &c));
}

TEST(test_parse_wifi_set_lengths) {
    umb_console_cmd_t c;
    char line[300], s32[33], s33[34], p63[64], p64[65];
    memset(s32, 's', 32); s32[32] = 0;
    memset(s33, 's', 33); s33[33] = 0;
    memset(p63, 'p', 63); p63[63] = 0;
    memset(p64, 'p', 64); p64[64] = 0;
    ASSERT_TRUE(parse("wifi set MyNet password1", &c));
    ASSERT_EQ(c.kind, UMB_CC_WIFI_SET);
    ASSERT_TRUE(strcmp(c.arg1, "MyNet") == 0);
    ASSERT_TRUE(strcmp(c.arg2, "password1") == 0);
    snprintf(line, sizeof line, "wifi set %s %s", s32, p63);
    ASSERT_TRUE(parse(line, &c));
    snprintf(line, sizeof line, "wifi set %s password1", s33);
    ASSERT_TRUE(is_invalid(line));
    snprintf(line, sizeof line, "wifi set net %s", p64);
    ASSERT_TRUE(is_invalid(line));
    ASSERT_TRUE(is_invalid("wifi set net short"));       /* 5 chars < 8 */
    ASSERT_TRUE(is_invalid("wifi set net 1234567"));     /* 7 chars */
    ASSERT_TRUE(parse("wifi set net 12345678", &c));     /* 8 chars */
    ASSERT_TRUE(is_invalid("wifi set \"\" password1"));  /* empty ssid */
    ASSERT_TRUE(is_invalid("wifi set net"));
    ASSERT_TRUE(is_invalid("wifi net password1"));
    ASSERT_TRUE(is_invalid("wifi set a b c"));
}

TEST(test_parse_wifi_quoted_and_open) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("wifi set \"My Net\" \"pa ss word\"", &c));
    ASSERT_TRUE(strcmp(c.arg1, "My Net") == 0);
    ASSERT_TRUE(strcmp(c.arg2, "pa ss word") == 0);
    ASSERT_TRUE(parse("wifi set \"Open Net\" \"\"", &c));
    ASSERT_TRUE(strcmp(c.arg1, "Open Net") == 0);
    ASSERT_EQ(c.arg2[0], 0);
    ASSERT_TRUE(parse("wifi set \"Mixed Net\" password1", &c));
    ASSERT_TRUE(strcmp(c.arg1, "Mixed Net") == 0);
    ASSERT_TRUE(is_invalid("wifi set \"unterminated password1"));
    ASSERT_TRUE(is_invalid("wifi set \"a\"b password1"));
}

TEST(test_parse_agent_discover) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("agent set discover 8888", &c));
    ASSERT_EQ(c.kind, UMB_CC_AGENT_SET);
    ASSERT_TRUE(strcmp(c.arg1, "discover") == 0);
    ASSERT_TRUE(strcmp(c.arg2, "8888") == 0);
}

TEST(test_parse_agent_ip_port) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("agent set 192.168.1.50 8888", &c));
    ASSERT_TRUE(strcmp(c.arg1, "192.168.1.50") == 0);
    ASSERT_TRUE(parse("agent set 0.0.0.0 1", &c));
    ASSERT_TRUE(parse("agent set 255.255.255.255 65535", &c));
    ASSERT_TRUE(is_invalid("agent set 256.1.1.1 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4.5 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.x 8888"));
    ASSERT_TRUE(is_invalid("agent set 1..3.4 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4. 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4444 8888"));
    ASSERT_TRUE(is_invalid("agent set host 8888"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4 0"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4 65536"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4 -1"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4 80x"));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4"));
    ASSERT_TRUE(is_invalid("agent 1.2.3.4 8888"));
    ASSERT_TRUE(is_invalid("agent set discover 8888 x"));
}

TEST(test_parse_garbage_invalid) {
    ASSERT_TRUE(is_invalid("frobnicate"));
    ASSERT_TRUE(is_invalid("RAW 0 0"));
    ASSERT_TRUE(is_invalid("HELP"));
    ASSERT_TRUE(is_invalid("%$#@!"));
}

TEST(test_parse_empty_line_invalid) {
    ASSERT_TRUE(is_invalid(""));
    ASSERT_TRUE(is_invalid("   \t "));
    ASSERT_TRUE(is_invalid("\r\n"));
    umb_console_cmd_t c;
    memset(&c, 0xA5, sizeof c);
    ASSERT_FALSE(umb_console_parse(NULL, &c));
    ASSERT_EQ(c.kind, UMB_CC_INVALID);
    ASSERT_FALSE(umb_console_parse("help", NULL));
}

TEST(test_parse_trailing_newline_ok) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("status\r\n", &c));
    ASSERT_EQ(c.kind, UMB_CC_STATUS);
    ASSERT_TRUE(parse("  raw \t0.1   0.2  500 \n", &c));
    ASSERT_EQ(c.kind, UMB_CC_RAW);
    ASSERT_EQ(c.duration_ms, 500);
    ASSERT_TRUE(parse("set a b\n", &c));
    ASSERT_TRUE(strcmp(c.arg1, "b") == 0);
}

TEST(test_parse_out_fully_initialised) {
    umb_console_cmd_t c;
    memset(&c, 0xA5, sizeof c);
    ASSERT_TRUE(parse("help", &c));
    umb_console_cmd_t z;
    memset(&z, 0, sizeof z);
    z.kind = UMB_CC_HELP;
    ASSERT_TRUE(memcmp(&c, &z, sizeof c) == 0);
}

TEST(test_parse_very_long_token_invalid) {
    char line[1000];
    memset(line, 'x', sizeof line);
    memcpy(line, "set k ", 6);
    line[999] = 0;
    ASSERT_TRUE(is_invalid(line));
    memcpy(line, "help ", 5);
    ASSERT_TRUE(is_invalid(line));
}

TEST(test_parse_missing_args_invalid) {
    ASSERT_TRUE(is_invalid("set"));
    ASSERT_TRUE(is_invalid("set a"));
    ASSERT_TRUE(is_invalid("get"));
    ASSERT_TRUE(is_invalid("agent"));
    ASSERT_TRUE(is_invalid("agent set"));
    ASSERT_TRUE(is_invalid("agent set discover"));
    ASSERT_TRUE(is_invalid("wifi"));
    ASSERT_TRUE(is_invalid("wifi set"));
    ASSERT_TRUE(is_invalid("mode"));
    ASSERT_TRUE(is_invalid("arm"));
    ASSERT_TRUE(is_invalid("raw"));
}

TEST(test_parse_agent_port_overflow_and_canonical) {
    char line[300];
    umb_console_cmd_t c;
    memset(line, 0, sizeof line);
    strcpy(line, "agent set 1.2.3.4 ");
    memset(line + strlen(line), '0', 100);
    strcat(line, "80");
    ASSERT_TRUE(is_invalid(line));
    ASSERT_TRUE(is_invalid("agent set 1.2.3.4 000080"));
    ASSERT_TRUE(parse("agent set 1.2.3.4 08888", &c));   /* canonicalised */
    ASSERT_TRUE(strcmp(c.arg2, "8888") == 0);
}

TEST(test_parse_sensors_and_cal_imu) {
    umb_console_cmd_t c;
    ASSERT_TRUE(parse("sensors", &c));
    ASSERT_EQ(c.kind, UMB_CC_SENSORS);
    ASSERT_TRUE(parse("  sensors\r\n", &c));
    ASSERT_EQ(c.kind, UMB_CC_SENSORS);
    ASSERT_TRUE(parse("cal imu", &c));
    ASSERT_EQ(c.kind, UMB_CC_CAL_IMU);
    ASSERT_TRUE(is_invalid("sensors now"));
    ASSERT_TRUE(is_invalid("cal"));
    ASSERT_TRUE(is_invalid("cal tof"));
    ASSERT_TRUE(is_invalid("cal imu now"));
    ASSERT_TRUE(is_invalid("sensor"));
}

int main(void) {
    RUN_TEST(test_parse_missing_args_invalid);
    RUN_TEST(test_parse_agent_port_overflow_and_canonical);
    RUN_TEST(test_parse_raw_default_duration);
    RUN_TEST(test_parse_raw_duration_capped);
    RUN_TEST(test_parse_duration_invalid);
    RUN_TEST(test_parse_raw_rejects_out_of_range);
    RUN_TEST(test_parse_numbers_strict);
    RUN_TEST(test_parse_speed_command);
    RUN_TEST(test_parse_auto_command);
    RUN_TEST(test_parse_simple_keywords);
    RUN_TEST(test_parse_arm);
    RUN_TEST(test_parse_mode_values);
    RUN_TEST(test_parse_set_key_value);
    RUN_TEST(test_parse_set_length_limits);
    RUN_TEST(test_parse_wifi_set_lengths);
    RUN_TEST(test_parse_wifi_quoted_and_open);
    RUN_TEST(test_parse_agent_discover);
    RUN_TEST(test_parse_agent_ip_port);
    RUN_TEST(test_parse_garbage_invalid);
    RUN_TEST(test_parse_empty_line_invalid);
    RUN_TEST(test_parse_trailing_newline_ok);
    RUN_TEST(test_parse_out_fully_initialised);
    RUN_TEST(test_parse_very_long_token_invalid);
    RUN_TEST(test_parse_sensors_and_cal_imu);
    TEST_SUMMARY();
}
