#include "test_runner.h"
#include "umb_net_fsm.h"

TEST(test_wifi_station_fallback_and_ap_retry) {
    umb_wifi_state_t s = umb_wifi_init();
    umb_wifi_config_t c = {UMB_WIFI_AUTO, true, 15000};
    umb_wifi_in_t in = {.now_ms = 100}; umb_wifi_out_t out;
    s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_START_STA);
    in.now_ms = 15099; s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(out.actions, 0);
    in.now_ms = 15100; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_STOP_STA | UMB_NET_START_AP);
    in.ap_up = true; s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(s.stage, UMB_WIFI_ON_AP);
    in.now_ms += 60000; in.session_busy = true;
    s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(out.actions, 0);
    in.session_busy = false; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_STOP_AP | UMB_NET_START_STA);
}
TEST(test_wifi_preferences_and_missing_credentials) {
    umb_wifi_out_t out; umb_wifi_in_t in = {0};
    umb_wifi_config_t c = {UMB_WIFI_AUTO, false, 15000};
    umb_wifi_state_t s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_START_AP);
    c.mode = UMB_WIFI_STA_ONLY;
    s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    ASSERT_EQ(s.stage, UMB_WIFI_BACKOFF); ASSERT_EQ(out.actions, 0);
    in.now_ms = 100000;
    s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(out.actions, 0);
    c.mode = UMB_WIFI_AP_ONLY; c.credentials = true;
    s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_START_AP);
}
TEST(test_wifi_join_success_loss_and_wrap) {
    umb_wifi_config_t c = {UMB_WIFI_AUTO, true, 100};
    umb_wifi_in_t in = {.now_ms = UINT32_MAX - 50}; umb_wifi_out_t out;
    umb_wifi_state_t s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.now_ms = 48; s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(out.actions, 0);
    in.sta_up = true; s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(s.stage, UMB_WIFI_ON_STA);
    in.sta_up = false; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_EQ(out.actions, UMB_NET_STOP_STA | UMB_NET_START_STA);
}
TEST(test_wifi_mode_changes_wait_for_session) {
    umb_wifi_config_t c = {UMB_WIFI_AP_ONLY, false, 100};
    umb_wifi_in_t in = {0}; umb_wifi_out_t out;
    umb_wifi_state_t s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.ap_up = true; s = umb_wifi_step(s, &c, &in, &out);
    c.mode = UMB_WIFI_STA_ONLY; c.credentials = true; in.session_busy = true;
    s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(s.stage, UMB_WIFI_ON_AP);
    in.session_busy = false; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_EQ(out.actions, UMB_NET_STOP_AP | UMB_NET_START_STA);
}
static umb_session_t ready(umb_session_in_t *in) {
    umb_session_out_t out;
    *in = (umb_session_in_t){.wifi_up = true, .endpoint_valid = true};
    umb_session_t s = umb_session_step(umb_session_init(), in, &out);
    in->probe_done = in->probe_ok = true;
    s = umb_session_step(s, in, &out);
    in->probe_done = false; in->create_done = in->create_ok = true;
    return umb_session_step(s, in, &out);
}
TEST(test_session_ready_requires_entity_create_and_loss_precedes_teardown) {
    umb_session_in_t in = {.wifi_up = true, .endpoint_valid = true}; umb_session_out_t out;
    umb_session_t s = umb_session_step(umb_session_init(), &in, &out);
    ASSERT_EQ(out.action, UMB_SESSION_ACT_PROBE); ASSERT_FALSE(out.ready);
    in.probe_done = in.probe_ok = true;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_CREATE); ASSERT_FALSE(out.ready);
    in.probe_done = false; in.create_done = in.create_ok = true;
    s = umb_session_step(s, &in, &out); ASSERT_TRUE(out.ready);
    in.wifi_up = false;
    s = umb_session_step(s, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_FALSE(out.ready); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_NONE);
    in.destroy_done = true; s = umb_session_step(s, &in, &out); ASSERT_EQ(s.stage, UMB_SESSION_BACKOFF);
}
TEST(test_session_three_ping_failures_drop_and_success_resets) {
    umb_session_in_t in; umb_session_t s = ready(&in); umb_session_out_t out;
    in.create_done = false;
    for (int i = 0; i < 3; i++) {
        in.now_ms += 100; in.ping_done = false;
        s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_PING);
        in.ping_done = true; in.ping_ok = false; in.now_ms += 20;
        s = umb_session_step(s, &in, &out);
        if (i < 2) ASSERT_TRUE(out.ready);
    }
    ASSERT_TRUE(out.loss); ASSERT_FALSE(out.ready); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
    s = ready(&in); in.create_done = false; in.now_ms = 100;
    s = umb_session_step(s, &in, &out); in.ping_done = true; in.ping_ok = true;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(s.ping_failures, 0); ASSERT_TRUE(out.ready);
}
TEST(test_session_transport_failure_drops_ready_session_once) {
    umb_session_in_t in; umb_session_t s = ready(&in); umb_session_out_t out;
    in.create_done = false; in.now_ms = 10;
    s = umb_session_step(s, &in, &out); ASSERT_TRUE(out.ready); ASSERT_FALSE(out.loss);
    in.transport_failed = true;
    s = umb_session_step(s, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_FALSE(out.ready); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
    s = umb_session_step(s, &in, &out); ASSERT_FALSE(out.loss); ASSERT_EQ(out.action, UMB_SESSION_ACT_NONE);
    /* While pinging too. */
    s = ready(&in); in.create_done = false; in.now_ms = 100;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_PING);
    in.transport_failed = true; s = umb_session_step(s, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
}
TEST(test_session_partial_create_failure_still_destroys) {
    umb_session_in_t in = {.wifi_up = true, .endpoint_valid = true}; umb_session_out_t out;
    umb_session_t s = umb_session_step(umb_session_init(), &in, &out);
    in.probe_done = in.probe_ok = true; s = umb_session_step(s, &in, &out);
    in.probe_done = false; in.create_done = true; in.create_ok = false;
    s = umb_session_step(s, &in, &out);
    ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY); ASSERT_FALSE(out.ready);
}
TEST(test_session_discovery_and_operation_deadlines_are_bounded) {
    umb_session_in_t in = {.wifi_up = true}; umb_session_out_t out;
    umb_session_t s = umb_session_step(umb_session_init(), &in, &out);
    ASSERT_EQ(out.action, UMB_SESSION_ACT_FIND);
    in.now_ms = 1001; s = umb_session_step(s, &in, &out);
    ASSERT_EQ(s.stage, UMB_SESSION_BACKOFF);
    in.now_ms = 2001; in.endpoint_valid = true;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_PROBE);
    in.now_ms += 100; s = umb_session_step(s, &in, &out); ASSERT_EQ(s.stage, UMB_SESSION_BACKOFF);
}
TEST(test_wifi_failed_startups_backoff_without_mode_leak) {
    umb_wifi_config_t c = {UMB_WIFI_STA_ONLY, true, 100};
    umb_wifi_in_t in = {0}; umb_wifi_out_t out;
    umb_wifi_state_t s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.error = true;
    for (int i = 0; i < 8; i++) {
        s = umb_wifi_step(s, &c, &in, &out);
        ASSERT_EQ(out.actions, UMB_NET_STOP_STA);
        ASSERT_EQ(s.stage, UMB_WIFI_BACKOFF);
        in.error = false; in.now_ms = s.deadline_ms;
        s = umb_wifi_step(s, &c, &in, &out);
        ASSERT_EQ(out.actions, UMB_NET_START_STA);
        ASSERT_TRUE(s.backoff_ms <= 8000);
        in.error = true;
    }
    c.mode = UMB_WIFI_AP_ONLY; in.error = false;
    s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.error = true; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_STOP_AP);
    in.error = false; in.now_ms = s.deadline_ms;
    s = umb_wifi_step(s, &c, &in, &out); ASSERT_EQ(out.actions, UMB_NET_START_AP);
    in.ap_up = true; s = umb_wifi_step(s, &c, &in, &out);
    in.ap_up = false; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_TRUE(out.loss); ASSERT_EQ(out.actions, UMB_NET_STOP_AP | UMB_NET_START_AP);
}
TEST(test_session_endpoint_loss_and_timed_operations_release_before_retry) {
    umb_session_in_t in; umb_session_t s = ready(&in); umb_session_out_t out;
    in.create_done = false; in.endpoint_valid = false;
    s = umb_session_step(s, &in, &out); ASSERT_TRUE(out.loss);
    ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
    in.now_ms += 1000; in.endpoint_valid = true;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
    ASSERT_FALSE(out.ready);
    in.destroy_done = true; s = umb_session_step(s, &in, &out);
    in.destroy_done = false; in.now_ms += 200;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_PROBE);
    in.probe_done = true; in.probe_ok = true; s = umb_session_step(s, &in, &out);
    in.probe_done = false; in.now_ms += 5000;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
}
TEST(test_session_finding_success_and_ping_timeouts_across_wrap) {
    umb_session_in_t in = {.now_ms = UINT32_MAX - 200, .wifi_up = true};
    umb_session_out_t out;
    umb_session_t s = umb_session_step(umb_session_init(), &in, &out);
    in.find_done = true; in.endpoint_valid = true;
    s = umb_session_step(s, &in, &out); ASSERT_EQ(out.action, UMB_SESSION_ACT_PROBE);
    in.find_done = false; in.probe_done = in.probe_ok = true;
    s = umb_session_step(s, &in, &out);
    in.probe_done = false; in.create_done = in.create_ok = true;
    s = umb_session_step(s, &in, &out); in.create_done = false;
    for (int i = 0; i < 3; i++) {
        in.now_ms += 100; s = umb_session_step(s, &in, &out);
        ASSERT_EQ(out.action, UMB_SESSION_ACT_PING);
        in.now_ms += 20; s = umb_session_step(s, &in, &out);
    }
    ASSERT_TRUE(out.loss); ASSERT_EQ(out.action, UMB_SESSION_ACT_DESTROY);
}

TEST(test_wifi_auto_failed_ap_start_retries_station_first) {
    /* AUTO with credentials: a failed AP start (e.g. password not provisioned yet)
     * must not trap the radio in AP retries; the next attempt rejoins the station. */
    umb_wifi_config_t c = {UMB_WIFI_AUTO, true, 1000};
    umb_wifi_in_t in = {0}; umb_wifi_out_t out;
    umb_wifi_state_t s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.now_ms = 1000; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_STOP_STA | UMB_NET_START_AP);
    in.error = true; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(s.stage, UMB_WIFI_BACKOFF); ASSERT_FALSE(s.retry_ap);
    in.error = false; in.now_ms = s.deadline_ms; s = umb_wifi_step(s, &c, &in, &out);
    ASSERT_EQ(out.actions, UMB_NET_START_STA); ASSERT_EQ(s.stage, UMB_WIFI_JOINING);
    /* Without station credentials AUTO keeps retrying the AP. */
    c.credentials = false; s = umb_wifi_step(umb_wifi_init(), &c, &in, &out);
    in.error = true; s = umb_wifi_step(s, &c, &in, &out); ASSERT_TRUE(s.retry_ap);
}
int main(void) {
    RUN_TEST(test_wifi_failed_startups_backoff_without_mode_leak);
    RUN_TEST(test_session_endpoint_loss_and_timed_operations_release_before_retry);
    RUN_TEST(test_session_finding_success_and_ping_timeouts_across_wrap);
    RUN_TEST(test_wifi_station_fallback_and_ap_retry);
    RUN_TEST(test_wifi_auto_failed_ap_start_retries_station_first);
    RUN_TEST(test_wifi_preferences_and_missing_credentials);
    RUN_TEST(test_wifi_join_success_loss_and_wrap);
    RUN_TEST(test_wifi_mode_changes_wait_for_session);
    RUN_TEST(test_session_ready_requires_entity_create_and_loss_precedes_teardown);
    RUN_TEST(test_session_three_ping_failures_drop_and_success_resets);
    RUN_TEST(test_session_transport_failure_drops_ready_session_once);
    RUN_TEST(test_session_partial_create_failure_still_destroys);
    RUN_TEST(test_session_discovery_and_operation_deadlines_are_bounded);
    TEST_SUMMARY();
}
