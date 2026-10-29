#include "test_runner.h"
#include "umb_safety.h"
#include <stdint.h>

static umb_safety_in_t base_in(void) {
    umb_safety_in_t in = { 1000u, true, 1000u, UMB_SRC_CONSOLE, true, true, 1000u };
    return in;
}

TEST(test_safety_no_cmd_stops) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.have_cmd = false;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_NO_CMD);
}

TEST(test_safety_all_clear) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    ASSERT_EQ(umb_safety_evaluate(&in, &c), 0u);
}

TEST(test_safety_timeout_boundary) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.now_ms = in.last_cmd_ms + 249u;
    in.core0_beat_ms = in.now_ms;
    ASSERT_EQ(umb_safety_evaluate(&in, &c) & UMB_STOP_CMD_TIMEOUT, 0u);
    in.now_ms = in.last_cmd_ms + 250u;
    in.core0_beat_ms = in.now_ms;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_CMD_TIMEOUT);
}

TEST(test_safety_timeout_across_wrap) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.last_cmd_ms = UINT32_MAX - 10u;
    in.now_ms = 300u;
    in.core0_beat_ms = in.now_ms;
    ASSERT_TRUE(umb_safety_evaluate(&in, &c) & UMB_STOP_CMD_TIMEOUT);
    in.now_ms = 200u;
    in.core0_beat_ms = in.now_ms;
    ASSERT_EQ(umb_safety_evaluate(&in, &c) & UMB_STOP_CMD_TIMEOUT, 0u);
}

/* Core 0 may stamp a command / beat 1 ms after core 1 read its clock. */
TEST(test_safety_future_cmd_stamp_is_fresh) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.last_cmd_ms = in.now_ms + 1u;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), 0u);
}

TEST(test_safety_future_core0_beat_is_fresh) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.core0_beat_ms = in.now_ms + 1u;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), 0u);
}

TEST(test_safety_link_only_for_ros_source) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.wifi_up = false;
    in.agent_up = false;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), 0u);
    in.source = UMB_SRC_ROS;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_WIFI | UMB_STOP_AGENT);
    in.wifi_up = true;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_AGENT);
    in.wifi_up = false;
    in.agent_up = true;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_WIFI);
}

TEST(test_safety_core0_stale) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.now_ms = 2000u;
    in.last_cmd_ms = 2000u;
    in.core0_beat_ms = 2000u - 500u;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), 0u);
    in.core0_beat_ms = 2000u - 501u;
    ASSERT_EQ(umb_safety_evaluate(&in, &c), UMB_STOP_CORE0);
}

TEST(test_safety_disarm_mask_excludes_timeout) {
    ASSERT_EQ(UMB_STOP_CMD_TIMEOUT & UMB_STOP_DISARM_MASK, 0u);
    ASSERT_EQ(UMB_STOP_NO_CMD & UMB_STOP_DISARM_MASK, 0u);
    ASSERT_EQ(UMB_STOP_REFLEX & UMB_STOP_DISARM_MASK, 0u);
    ASSERT_TRUE(UMB_STOP_CORE0 & UMB_STOP_DISARM_MASK);
}

TEST(test_safety_never_sets_reflex) {
    umb_settings_t c = umb_settings_defaults();
    umb_safety_in_t in = base_in();
    in.source = UMB_SRC_ROS;
    in.wifi_up = in.agent_up = false;
    in.have_cmd = false;
    in.core0_beat_ms = 0u;
    in.now_ms = 5000u;
    ASSERT_EQ(umb_safety_evaluate(&in, &c) & UMB_STOP_REFLEX, 0u);
}

int main(void) {
    RUN_TEST(test_safety_no_cmd_stops);
    RUN_TEST(test_safety_all_clear);
    RUN_TEST(test_safety_timeout_boundary);
    RUN_TEST(test_safety_timeout_across_wrap);
    RUN_TEST(test_safety_future_cmd_stamp_is_fresh);
    RUN_TEST(test_safety_future_core0_beat_is_fresh);
    RUN_TEST(test_safety_link_only_for_ros_source);
    RUN_TEST(test_safety_core0_stale);
    RUN_TEST(test_safety_disarm_mask_excludes_timeout);
    RUN_TEST(test_safety_never_sets_reflex);
    TEST_SUMMARY();
}
