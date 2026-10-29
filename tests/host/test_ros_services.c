/* /umbreon/arm handler against the real shared state, a fake clock and a
 * simulated core-1 control tick (take -> apply -> commit -> publish). */
#include "test_runner.h"
#include <string.h>
#include "ros_services.h"
#include "shared_state.h"

static uint32_t now;
uint32_t hal_now_ms(void) { return now; }

/* --- simulated control task, same order as control_task.c --- */
static umb_arming_t arming;
static shared_arm_request_t taken[SHARED_ARM_QUEUE_LEN];
static umb_arm_result_t results[SHARED_ARM_QUEUE_LEN];
static unsigned taken_count;
static umb_arming_t arming_before;

static umb_arming_t apply(umb_arming_t a, int kind, umb_arm_result_t *r) {
    switch (kind) {
    case SHARED_ARM_CAR:        return umb_arming_car_key(a, r);
    case SHARED_ARM_ROS:        return umb_arming_ros(a, true, r);
    case SHARED_ARM_ROS_DISARM: return umb_arming_ros(a, false, r);
    default:                    return umb_arming_disarm(a);
    }
}
static void control_take(void) {
    arming_before = arming;
    taken_count = 0;
    while (taken_count < SHARED_ARM_QUEUE_LEN && shared_take_arm_record(&taken[taken_count], now)) {
        results[taken_count] = UMB_ARM_OK;
        arming = apply(arming, taken[taken_count].kind, &results[taken_count]);
        int kind = taken[taken_count++].kind;
        if (kind == SHARED_ARM_DISARM || kind == SHARED_ARM_ROS_DISARM) break;
    }
}
static bool control_commit(void) {
    umb_outputs_t out = {0};
    out.armed = umb_arming_is_armed(arming);
    bool ok = shared_commit_arm_batch(taken, results, taken_count, &out, &arming, now);
    if (!ok) { arming = umb_arming_disarm(arming_before); out.armed = umb_arming_is_armed(arming); }
    shared_publish_outputs(&out);
    shared_publish_arming(&arming);
    taken_count = 0;
    return ok;
}
static void control_tick(void) { control_take(); control_commit(); }

/* --- fake clock: sleep advances time; hooks model core-1 running concurrently --- */
typedef enum { HOOK_NONE, HOOK_TICK, HOOK_TAKE, HOOK_LOSS } hook_t;
static hook_t sleep_hook, now_hook;
static uint32_t hook_at, sleeps, start, max_elapsed;
static void run_hook(hook_t h) {
    switch (h) {
    case HOOK_TICK: control_tick(); break;
    case HOOK_TAKE: control_take(); break;
    case HOOK_LOSS: shared_notify_network_loss(UMB_STOP_AGENT); shared_set_link(true, true); break;
    default: break;
    }
}
static uint32_t fake_now(void *ctx) {
    (void)ctx;
    /* hook fires on the time sample that first reaches hook_at */
    if (now_hook != HOOK_NONE && now >= hook_at) { hook_t h = now_hook; now_hook = HOOK_NONE; now -= 1u; run_hook(h); now += 1u; }
    if (now - start > max_elapsed) max_elapsed = now - start;
    return now;
}
static void fake_sleep(void *ctx, uint32_t ms) {
    (void)ctx;
    sleeps++;
    now += ms;
    if (sleep_hook != HOOK_NONE && now >= hook_at) { hook_t h = sleep_hook; sleep_hook = HOOK_NONE; run_hook(h); }
}
static const ros_arm_clock_t clock_fake = {fake_now, fake_sleep, NULL};

static uint32_t setup(umb_arm_mode_t mode) {
    now = 1000; start = now; sleeps = max_elapsed = 0; hook_at = 0;
    sleep_hook = now_hook = HOOK_NONE; taken_count = 0;
    umb_settings_t s = umb_settings_defaults();
    s.arming_mode = (uint8_t)mode;
    shared_init(&s);
    arming = umb_arming_init(mode);
    shared_set_link(true, true);
    return shared_session_generation();
}
static bool reply_is(ros_arm_reply_t r, const char *msg, bool success) {
    return r.message && strcmp(r.message, msg) == 0 && r.success == success;
}
/* No request ID consumed: the next local submit gets the first ID after init. */
static bool no_id_consumed(void) {
    uint32_t id = 0;
    return shared_submit_arm(SHARED_ARM_CAR, 0, now, &id) && id == 1u && shared_cancel_arm(id);
}
static bool nothing_queued(void) {
    shared_arm_request_t r;
    return !shared_take_arm_record(&r, now);
}

TEST(test_arm_ros_mode_reports_actual_control_ack) {
    uint32_t gen = setup(UMB_ARM_ROS);
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ros_arm_reply_t r = ros_services_arm(true, gen, &clock_fake);
    ASSERT_TRUE(reply_is(r, "armed", true));
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_TRUE(a.ros_key);
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_TRUE(o.armed);
    ASSERT_TRUE(max_elapsed <= ROS_ARM_WAIT_MS);
}

TEST(test_arm_both_without_car_key_waits_for_car) {
    uint32_t gen = setup(UMB_ARM_BOTH);
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "waiting for car arm", true));
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_FALSE(o.armed);
}

TEST(test_arm_both_after_car_key_is_armed) {
    uint32_t gen = setup(UMB_ARM_BOTH);
    ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR));
    control_tick();
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "armed", true));
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_TRUE(a.car_key && a.ros_key);
}

TEST(test_arm_wrong_mode_is_refused_by_control) {
    uint32_t gen = setup(UMB_ARM_ENCODER);
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "refused: mode", false));
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.ros_key);
}

TEST(test_arm_stale_or_dead_session_refused_without_submit) {
    uint32_t gen = setup(UMB_ARM_ROS);
    ASSERT_TRUE(reply_is(ros_services_arm(true, 0, &clock_fake), "refused: session", false));
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen + 1u, &clock_fake), "refused: session", false));
    ASSERT_TRUE(nothing_queued());
    ASSERT_TRUE(no_id_consumed());
    /* Dead session: agent lost (generation bumps, sticky disarm queued). */
    shared_set_link(true, false);
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "refused: session", false));
    ASSERT_TRUE(reply_is(ros_services_arm(true, shared_session_generation(), &clock_fake),
                         "refused: session", false));
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    ASSERT_EQ(r.kind, SHARED_ARM_DISARM); /* only the loss disarm, no arm request */
    ASSERT_EQ(sleeps, 0);
}

TEST(test_arm_during_maintenance_refused_without_submit) {
    uint32_t gen = setup(UMB_ARM_ROS);
    uint32_t token;
    ASSERT_TRUE(shared_begin_maintenance(&token));
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "refused: maintenance", false));
    ASSERT_TRUE(nothing_queued());
    ASSERT_EQ(sleeps, 0);
    shared_end_maintenance(token);
    ASSERT_TRUE(no_id_consumed());
}

TEST(test_arm_full_queue_is_busy_without_waiting) {
    uint32_t gen = setup(UMB_ARM_BOTH);
    for (unsigned i = 0; i < SHARED_ARM_QUEUE_LEN; i++) ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR));
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "busy: timeout", false));
    ASSERT_EQ(sleeps, 0);
}

TEST(test_arm_stalled_control_times_out_and_never_arms_later) {
    uint32_t gen = setup(UMB_ARM_ROS);
    ros_arm_reply_t r = ros_services_arm(true, gen, &clock_fake);
    ASSERT_TRUE(reply_is(r, "busy: timeout", false));
    ASSERT_EQ(max_elapsed, ROS_ARM_WAIT_MS);
    /* Core 1 sampled its clock just before the deadline but runs the tick only
     * after the handler replied: the cancelled request must not commit. */
    now = start + ROS_ARM_WAIT_MS - 1u;
    control_tick();
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.ros_key);
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_FALSE(o.armed);
}

TEST(test_arm_commit_just_before_timeout_reports_committed_ack) {
    uint32_t gen = setup(UMB_ARM_ROS);
    /* The handler reads PENDING, then core 1 commits at deadline-1 before the
     * handler samples the timeout and tries to cancel: cancel fails, ACK wins. */
    now_hook = HOOK_TICK; hook_at = start + ROS_ARM_WAIT_MS;
    ros_arm_reply_t r = ros_services_arm(true, gen, &clock_fake);
    ASSERT_TRUE(reply_is(r, "armed", true));
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_TRUE(o.armed);
    ASSERT_TRUE(max_elapsed <= ROS_ARM_WAIT_MS);
}

TEST(test_arm_taken_but_uncommitted_at_timeout_is_cancelled) {
    uint32_t gen = setup(UMB_ARM_ROS);
    /* Core 1 took the record (APPLYING) just before the timeout; the handler's
     * cancel wins, so the later commit discards the candidate. */
    now_hook = HOOK_TAKE; hook_at = start + ROS_ARM_WAIT_MS;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "busy: timeout", false));
    ASSERT_FALSE(control_commit());
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.ros_key);
    control_tick();
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_FALSE(o.armed);
}

TEST(test_arm_generation_change_while_waiting_refused_and_cancelled) {
    uint32_t gen = setup(UMB_ARM_ROS);
    sleep_hook = HOOK_LOSS; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "refused: session", false));
    ASSERT_TRUE(shared_session_generation() != gen);
    ASSERT_TRUE(max_elapsed < ROS_ARM_WAIT_MS);
    control_tick(); /* loss disarm */
    control_tick();
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.ros_key);
    ASSERT_TRUE(nothing_queued());
}

TEST(test_disarm_clears_both_keys_with_priority) {
    uint32_t gen = setup(UMB_ARM_BOTH);
    ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR));
    control_tick();
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "armed", true));
    ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR)); /* queued, must be dropped */
    start = now; sleep_hook = HOOK_TICK; hook_at = now + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(false, gen, &clock_fake), "disarmed", true));
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.car_key);
    ASSERT_FALSE(a.ros_key);
    umb_outputs_t o; shared_get_outputs(&o);
    ASSERT_FALSE(o.armed);
    ASSERT_TRUE(nothing_queued());
}

TEST(test_disarm_accepted_for_stale_session_and_maintenance) {
    uint32_t gen = setup(UMB_ARM_ROS);
    uint32_t token;
    ASSERT_TRUE(shared_begin_maintenance(&token));
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(false, gen + 7u, &clock_fake), "disarmed", true));
    ASSERT_TRUE(nothing_queued());
    start = now; sleep_hook = HOOK_TICK; hook_at = now + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(false, 0, &clock_fake), "disarmed", true));
    shared_end_maintenance(token);
}

TEST(test_disarm_with_stalled_control_stays_sticky) {
    uint32_t gen = setup(UMB_ARM_ROS);
    sleep_hook = HOOK_TICK; hook_at = start + 1u;
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, &clock_fake), "armed", true));
    start = now;
    ASSERT_TRUE(reply_is(ros_services_arm(false, gen, &clock_fake), "disarmed", true));
    ASSERT_TRUE(max_elapsed <= ROS_ARM_WAIT_MS);
    /* No ACK yet; the request is still queued and the next tick applies it. */
    control_tick();
    umb_arming_t a; shared_get_arming(&a);
    ASSERT_FALSE(a.ros_key);
}

TEST(test_arm_null_clock_refuses_safely) {
    uint32_t gen = setup(UMB_ARM_ROS);
    ASSERT_TRUE(reply_is(ros_services_arm(true, gen, NULL), "busy: timeout", false));
    ASSERT_TRUE(nothing_queued());
    /* Disarm still reaches control without a clock. */
    ASSERT_TRUE(reply_is(ros_services_arm(false, gen, NULL), "disarmed", true));
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    ASSERT_EQ(r.kind, SHARED_ARM_DISARM);
}

int main(void) {
    RUN_TEST(test_arm_ros_mode_reports_actual_control_ack);
    RUN_TEST(test_arm_both_without_car_key_waits_for_car);
    RUN_TEST(test_arm_both_after_car_key_is_armed);
    RUN_TEST(test_arm_wrong_mode_is_refused_by_control);
    RUN_TEST(test_arm_stale_or_dead_session_refused_without_submit);
    RUN_TEST(test_arm_during_maintenance_refused_without_submit);
    RUN_TEST(test_arm_full_queue_is_busy_without_waiting);
    RUN_TEST(test_arm_stalled_control_times_out_and_never_arms_later);
    RUN_TEST(test_arm_commit_just_before_timeout_reports_committed_ack);
    RUN_TEST(test_arm_taken_but_uncommitted_at_timeout_is_cancelled);
    RUN_TEST(test_arm_generation_change_while_waiting_refused_and_cancelled);
    RUN_TEST(test_disarm_clears_both_keys_with_priority);
    RUN_TEST(test_disarm_accepted_for_stale_session_and_maintenance);
    RUN_TEST(test_disarm_with_stalled_control_stays_sticky);
    RUN_TEST(test_arm_null_clock_refuses_safely);
    TEST_SUMMARY();
}
