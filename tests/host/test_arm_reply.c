/* Pure /umbreon/arm reply selection and bounded-wait decisions. */
#include "test_runner.h"
#include <string.h>
#include "umb_arm_reply.h"

static umb_arm_wait_view_t view(bool arm, umb_arm_ack_state_t ack, uint32_t elapsed) {
    umb_arm_wait_view_t v = {.arm = arm, .ack = ack, .result = UMB_ARM_OK,
                             .session_ready = true, .maintenance = false,
                             .cancel_attempted = false, .elapsed_ms = elapsed};
    return v;
}

TEST(test_arm_reply_messages_and_success_are_exact) {
    static const struct { umb_arm_reply_t r; const char *msg; bool ok; } table[] = {
        {UMB_ARM_REPLY_ARMED, "armed", true},
        {UMB_ARM_REPLY_WAITING_CAR, "waiting for car arm", true},
        {UMB_ARM_REPLY_WAITING_ROS, "waiting for ros arm", true},
        {UMB_ARM_REPLY_DISARMED, "disarmed", true},
        {UMB_ARM_REPLY_REFUSED_MODE, "refused: mode", false},
        {UMB_ARM_REPLY_REFUSED_SESSION, "refused: session", false},
        {UMB_ARM_REPLY_REFUSED_MAINTENANCE, "refused: maintenance", false},
        {UMB_ARM_REPLY_BUSY_TIMEOUT, "busy: timeout", false},
    };
    for (unsigned i = 0; i < sizeof table / sizeof table[0]; i++) {
        ASSERT_TRUE(strcmp(umb_arm_reply_message(table[i].r), table[i].msg) == 0);
        ASSERT_EQ(umb_arm_reply_success(table[i].r), table[i].ok);
    }
    ASSERT_TRUE(strcmp(umb_arm_reply_message((umb_arm_reply_t)99), "busy: timeout") == 0);
    ASSERT_FALSE(umb_arm_reply_success((umb_arm_reply_t)99));
}

TEST(test_arm_reply_maps_control_results) {
    ASSERT_EQ(umb_arm_reply_for_result(UMB_ARM_OK), UMB_ARM_REPLY_ARMED);
    ASSERT_EQ(umb_arm_reply_for_result(UMB_ARM_WAITING_CAR), UMB_ARM_REPLY_WAITING_CAR);
    ASSERT_EQ(umb_arm_reply_for_result(UMB_ARM_WAITING_ROS), UMB_ARM_REPLY_WAITING_ROS);
    ASSERT_EQ(umb_arm_reply_for_result(UMB_ARM_REJECTED_MODE), UMB_ARM_REPLY_REFUSED_MODE);
    ASSERT_EQ(umb_arm_reply_for_result((umb_arm_result_t)99), UMB_ARM_REPLY_BUSY_TIMEOUT);
}

TEST(test_arm_refusal_session_before_maintenance) {
    ASSERT_EQ(umb_arm_reply_refusal(false, true), UMB_ARM_REPLY_REFUSED_SESSION);
    ASSERT_EQ(umb_arm_reply_refusal(false, false), UMB_ARM_REPLY_REFUSED_SESSION);
    ASSERT_EQ(umb_arm_reply_refusal(true, true), UMB_ARM_REPLY_REFUSED_MAINTENANCE);
    ASSERT_EQ(umb_arm_reply_refusal(true, false), UMB_ARM_REPLY_BUSY_TIMEOUT);
}

TEST(test_arm_wait_polls_then_cancels_at_bound) {
    umb_arm_wait_view_t v = view(true, UMB_ARM_ACK_PENDING, 0);
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_POLL);
    v.elapsed_ms = UMB_ARM_WAIT_MS - 1u;
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_POLL);
    v.elapsed_ms = UMB_ARM_WAIT_MS;
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_CANCEL);
    /* Still pending after a cancel attempt cannot happen; never wait again. */
    v.cancel_attempted = true;
    umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_BUSY_TIMEOUT);
}

TEST(test_arm_wait_stale_session_or_maintenance_cancels_pending) {
    umb_arm_wait_view_t v = view(true, UMB_ARM_ACK_PENDING, 5);
    v.session_ready = false;
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_CANCEL);
    v.session_ready = true; v.maintenance = true;
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_CANCEL);
    v.cancel_attempted = true; v.maintenance = false; v.session_ready = false;
    umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_REFUSED_SESSION);
}

TEST(test_arm_wait_committed_ack_wins_over_timeout_and_session) {
    umb_arm_wait_view_t v = view(true, UMB_ARM_ACK_DONE, UMB_ARM_WAIT_MS + 5u);
    v.cancel_attempted = true; v.session_ready = false; v.result = UMB_ARM_WAITING_CAR;
    umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_WAITING_CAR);
}

TEST(test_arm_wait_cancelled_or_missing_classified) {
    umb_arm_wait_view_t v = view(true, UMB_ARM_ACK_CANCELLED, 10);
    ASSERT_EQ(umb_arm_wait_decide(&v).reply, UMB_ARM_REPLY_BUSY_TIMEOUT);
    v.maintenance = true;
    ASSERT_EQ(umb_arm_wait_decide(&v).reply, UMB_ARM_REPLY_REFUSED_MAINTENANCE);
    v.session_ready = false;
    ASSERT_EQ(umb_arm_wait_decide(&v).reply, UMB_ARM_REPLY_REFUSED_SESSION);
    v = view(true, UMB_ARM_ACK_MISSING, 0);
    umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_BUSY_TIMEOUT);
}

TEST(test_disarm_never_refused_or_cancelled) {
    static const umb_arm_ack_state_t states[] = {UMB_ARM_ACK_DONE, UMB_ARM_ACK_CANCELLED,
                                                 UMB_ARM_ACK_MISSING};
    for (unsigned i = 0; i < 3; i++) {
        umb_arm_wait_view_t v = view(false, states[i], 0);
        v.session_ready = false; v.maintenance = true; v.result = UMB_ARM_REJECTED_MODE;
        umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
        ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
        ASSERT_EQ(d.reply, UMB_ARM_REPLY_DISARMED);
    }
    umb_arm_wait_view_t v = view(false, UMB_ARM_ACK_PENDING, 0);
    v.session_ready = false; v.maintenance = true;
    ASSERT_EQ(umb_arm_wait_decide(&v).step, UMB_ARM_WAIT_POLL);
    v.elapsed_ms = UMB_ARM_WAIT_MS;
    umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY); /* sticky disarm: never cancelled */
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_DISARMED);
}

TEST(test_arm_wait_sleep_never_passes_bound) {
    ASSERT_EQ(umb_arm_wait_sleep_ms(0), UMB_ARM_POLL_MS);
    ASSERT_EQ(umb_arm_wait_sleep_ms(UMB_ARM_WAIT_MS - 1u), 1);
    ASSERT_EQ(umb_arm_wait_sleep_ms(UMB_ARM_WAIT_MS), 0);
    ASSERT_EQ(umb_arm_wait_sleep_ms(UMB_ARM_WAIT_MS + 100u), 0);
}

TEST(test_arm_wait_null_view_is_busy) {
    umb_arm_wait_decision_t d = umb_arm_wait_decide(NULL);
    ASSERT_EQ(d.step, UMB_ARM_WAIT_REPLY);
    ASSERT_EQ(d.reply, UMB_ARM_REPLY_BUSY_TIMEOUT);
}

int main(void) {
    RUN_TEST(test_arm_reply_messages_and_success_are_exact);
    RUN_TEST(test_arm_reply_maps_control_results);
    RUN_TEST(test_arm_refusal_session_before_maintenance);
    RUN_TEST(test_arm_wait_polls_then_cancels_at_bound);
    RUN_TEST(test_arm_wait_stale_session_or_maintenance_cancels_pending);
    RUN_TEST(test_arm_wait_committed_ack_wins_over_timeout_and_session);
    RUN_TEST(test_arm_wait_cancelled_or_missing_classified);
    RUN_TEST(test_disarm_never_refused_or_cancelled);
    RUN_TEST(test_arm_wait_sleep_never_passes_bound);
    RUN_TEST(test_arm_wait_null_view_is_busy);
    TEST_SUMMARY();
}
