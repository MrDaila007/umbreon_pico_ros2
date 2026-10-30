/* Session lifecycle against the fake rcl/rclc backend: readiness gating,
 * bounded steps, loss detection, reverse cleanup and fresh recovery. */
#include "test_runner.h"
#include "ros_session.h"
#include "fake_ros_backend.h"
#include <string.h>

#define GEN 41u
static fake_ros_t f;
static ros_backend_t b;
static ros_session_t s;
static ros_session_in_t in;
static ros_session_out_t out;
static unsigned teardowns_requested, ready_steps;

static void setup(void) {
    fake_ros_reset(&f); b = fake_ros_backend(&f); ros_session_init(&s);
    in = (ros_session_in_t){.now_ms = 1000, .link_up = true, .endpoint_selected = true,
                            .endpoint = {0x0a000005u, 8888, 3, 0x1234abcdu}, .generation = GEN,
                            .open_allowed = true};
    teardowns_requested = ready_steps = 0;
}
/* One owner iteration: loss is "latched" before teardown, like net_owner. */
static void step(void) {
    out = ros_session_step(&s, &in, &b);
    if (out.ready) ready_steps++;
    if (out.teardown) { ASSERT_FALSE(out.ready); teardowns_requested++; (void)ros_session_teardown(&s, &b); }
    in.now_ms += 10;
}
static void run(unsigned n) { for (unsigned i = 0; i < n; i++) step(); }
static bool reach_ready(unsigned max_steps) {
    for (unsigned i = 0; i < max_steps; i++) { step(); if (out.ready) return true; }
    return false;
}

TEST(test_ready_only_after_all_entities_and_executor) {
    setup();
    unsigned steps = 0;
    while (!out.ready && steps < 100) {
        unsigned before = f.creates;
        step(); steps++;
        ASSERT_TRUE(f.creates - before <= 1); /* one bounded creation per iteration */
        if (!out.ready) ASSERT_EQ(f.generation, 0); /* callbacks carry setup generation */
        if (!out.ready) ASSERT_EQ(f.spins, 0);
    }
    ASSERT_TRUE(out.ready); ASSERT_EQ(f.creates, ROS_ENT_COUNT);
    ASSERT_EQ(f.live, (1u << ROS_ENT_COUNT) - 1u);
    ASSERT_EQ(f.generation, GEN); ASSERT_EQ(s.generation, GEN); ASSERT_EQ(f.session_starts, 1);
    ASSERT_EQ(f.last_cfg.client_key, 0x1234abcdu); ASSERT_EQ(f.last_cfg.domain_id, 3);
    ASSERT_EQ(f.last_cfg.agent_ip, 0x0a000005u); ASSERT_EQ(f.last_cfg.agent_port, 8888);
    ASSERT_EQ(s.sessions, 1);
    /* No spin in the step that became ready: the owner publishes agent_up first,
     * so nothing delivered in the first spin is refused as "not ready". */
    ASSERT_EQ(f.spins, 0);
    ASSERT_EQ(f.service_reply_timeout_ms, 20); /* contract bound applied to /umbreon/arm */
    step(); ASSERT_EQ(f.spins, 1);
    run(50); ASSERT_TRUE(out.ready); ASSERT_EQ(teardowns_requested, 0); ASSERT_EQ(f.creates, ROS_ENT_COUNT);
    ASSERT_TRUE(f.pings >= 4);
}

TEST(test_failure_at_each_step_cleans_up_and_recovers_fresh) {
    for (int fail = 0; fail < ROS_ENT_COUNT; fail++) {
        setup(); f.fail_create_id = fail;
        unsigned guard = 0;
        while (teardowns_requested == 0 && guard++ < 100) step();
        ASSERT_EQ(teardowns_requested, 1); ASSERT_FALSE(out.ready); ASSERT_EQ(ready_steps, 0);
        ASSERT_EQ(f.live, 0); ASSERT_FALSE(f.transport_open); ASSERT_EQ(f.destroy_without_create, 0);
        ASSERT_EQ(f.destroys_before_teardown, 0); ASSERT_EQ(s.create_failures, 1);
        ASSERT_EQ(f.generation, 0); ASSERT_FALSE(ros_session_busy(&s));
        /* Recovery after backoff builds a complete fresh graph. */
        f.fail_create_id = -1;
        ASSERT_TRUE(reach_ready(200));
        ASSERT_EQ(f.live, (1u << ROS_ENT_COUNT) - 1u); ASSERT_EQ(f.creates_while_live_transport, 0);
        ASSERT_EQ(f.generation, GEN);
    }
}

TEST(test_ping_loss_drops_ready_before_teardown) {
    setup(); ASSERT_TRUE(reach_ready(100));
    f.agent_alive = false; f.spin_result = ROS_SPIN_OK;
    unsigned guard = 0;
    while (out.ready && guard++ < 100) step();
    ASSERT_TRUE(out.loss); ASSERT_EQ(teardowns_requested, 1);
    ASSERT_EQ(f.live, 0); ASSERT_FALSE(f.transport_open); ASSERT_EQ(f.generation, 0);
    ASSERT_EQ(s.losses, 1); ASSERT_FALSE(s.sync.synced);
    ASSERT_TRUE(guard <= 45); /* 3 x 100 ms pings, + spin errors, well under 500 ms */
}

TEST(test_executor_timeout_normal_repeated_errors_trigger_loss) {
    setup(); ASSERT_TRUE(reach_ready(100));
    f.spin_result = ROS_SPIN_ERROR; step(); step();
    f.spin_result = ROS_SPIN_OK; step(); /* a successful spin resets the error run */
    f.spin_result = ROS_SPIN_ERROR; step(); step();
    ASSERT_TRUE(out.ready); ASSERT_EQ(teardowns_requested, 0);
    step(); /* third consecutive actual error */
    step(); ASSERT_TRUE(out.loss); ASSERT_EQ(teardowns_requested, 1); ASSERT_EQ(s.transport_failures, 1);
    ASSERT_EQ(f.live, 0);
}

TEST(test_time_sync_retries_then_periodic_and_keeps_offset_on_resync_failure) {
    setup(); f.sync_ok = false;
    ASSERT_TRUE(reach_ready(100));
    run(30); ASSERT_FALSE(s.sync.synced);
    ASSERT_TRUE(f.syncs >= 2 && f.syncs <= 4); /* bounded retries every 100 ms, not every loop */
    f.sync_ok = true; f.mono_ms = 2000; f.epoch_ns = 1700000000000000000ll;
    run(12); ASSERT_TRUE(s.sync.synced);
    ASSERT_EQ(s.sync.offset_ns, 1700000000000000000ll - 2000ll * 1000000);
    unsigned syncs = f.syncs; run(100); ASSERT_EQ(f.syncs, syncs); /* next one after 30 s */
    f.sync_ok = false; in.now_ms += ROS_SYNC_PERIOD_MS; run(3);
    ASSERT_TRUE(s.sync.synced); ASSERT_TRUE(s.sync_failures > 0); /* previous offset kept */
    ASSERT_TRUE(out.ready);
    /* A new session invalidates the offset until it resynchronizes. */
    f.agent_alive = false; while (out.ready) step();
    ASSERT_FALSE(s.sync.synced);
    f.agent_alive = true; ASSERT_TRUE(reach_ready(200)); ASSERT_FALSE(s.sync.synced);
}

TEST(test_link_or_endpoint_change_drops_and_reconnects_with_new_endpoint) {
    setup(); ASSERT_TRUE(reach_ready(100));
    in.endpoint.agent_port = 7777; step();
    ASSERT_TRUE(out.loss); ASSERT_EQ(teardowns_requested, 1); ASSERT_EQ(f.live, 0);
    in.generation = GEN + 1;
    ASSERT_TRUE(reach_ready(200)); ASSERT_EQ(f.last_cfg.agent_port, 7777); ASSERT_EQ(f.generation, GEN + 1);
    ASSERT_EQ(f.session_starts, 2);
    in.endpoint.domain_id = 9; step(); ASSERT_TRUE(out.loss); /* domain change reconnects too */
    ASSERT_TRUE(reach_ready(200)); ASSERT_EQ(f.last_cfg.domain_id, 9);
    in.link_up = false; step(); ASSERT_TRUE(out.loss); ASSERT_EQ(f.live, 0);
    run(100); ASSERT_FALSE(out.ready); ASSERT_EQ(f.live, 0);
    in.link_up = true; ASSERT_TRUE(reach_ready(300));
    in.endpoint_selected = false; step(); ASSERT_TRUE(out.loss); ASSERT_EQ(f.live, 0);
}

TEST(test_unreleased_socket_blocks_recreate_until_released) {
    setup(); ASSERT_TRUE(reach_ready(100));
    f.release_ok = false; in.link_up = false; step();
    ASSERT_EQ(f.live, 0); ASSERT_TRUE(f.transport_open); ASSERT_TRUE(ros_session_busy(&s));
    in.link_up = true; unsigned creates = f.creates;
    run(150); ASSERT_EQ(f.creates, creates); ASSERT_FALSE(out.ready);
    ASSERT_TRUE(teardowns_requested >= 2); /* teardown re-requested after its deadline */
    f.release_ok = true; ASSERT_TRUE(reach_ready(400));
    ASSERT_EQ(f.creates_while_live_transport, 0); ASSERT_EQ(f.destroy_without_create, 0);
}

TEST(test_forced_teardown_is_idempotent_and_repeated_cycles_balanced) {
    setup();
    for (int cycle = 0; cycle < 100; cycle++) {
        ASSERT_TRUE(reach_ready(400));
        ASSERT_TRUE(ros_session_teardown(&s, &b)); /* owner-forced (link/reconfigure) */
        ASSERT_TRUE(ros_session_teardown(&s, &b));
        ASSERT_EQ(f.live, 0); ASSERT_FALSE(f.transport_open); ASSERT_EQ(f.generation, 0);
        ASSERT_FALSE(s.ready);
        step(); ASSERT_FALSE(out.ready);
    }
    ASSERT_EQ(f.destroy_without_create, 0); ASSERT_EQ(f.creates_while_live_transport, 0);
    ASSERT_EQ(s.sessions, 100);
}

TEST(test_session_open_never_attempted_while_armed) {
    setup(); in.open_allowed = false;
    run(300);
    ASSERT_FALSE(out.ready); ASSERT_EQ(f.live, 0);
    for (unsigned i = 0; i < f.len; i++) ASSERT_TRUE(!(f.op[i] == 'c' && f.id[i] == ROS_ENT_CONTEXT));
    /* Armed after the local init step but before the open: torn down, no open. */
    in.open_allowed = true;
    for (unsigned guard = 0; !(f.live & (1u << ROS_ENT_INIT_OPTIONS)) && guard < 400; guard++) step();
    ASSERT_TRUE(f.live & (1u << ROS_ENT_INIT_OPTIONS)); ASSERT_FALSE(f.live & (1u << ROS_ENT_CONTEXT));
    in.open_allowed = false; run(50);
    ASSERT_EQ(f.live, 0); ASSERT_FALSE(f.transport_open);
    for (unsigned i = 0; i < f.len; i++) ASSERT_TRUE(!(f.op[i] == 'c' && f.id[i] == ROS_ENT_CONTEXT));
    /* A live session is not dropped by arming: the gate only covers opening. */
    in.open_allowed = true; ASSERT_TRUE(reach_ready(300));
    in.open_allowed = false; run(50); ASSERT_TRUE(out.ready);
}
int main(void) {
    RUN_TEST(test_ready_only_after_all_entities_and_executor);
    RUN_TEST(test_failure_at_each_step_cleans_up_and_recovers_fresh);
    RUN_TEST(test_ping_loss_drops_ready_before_teardown);
    RUN_TEST(test_executor_timeout_normal_repeated_errors_trigger_loss);
    RUN_TEST(test_time_sync_retries_then_periodic_and_keeps_offset_on_resync_failure);
    RUN_TEST(test_link_or_endpoint_change_drops_and_reconnects_with_new_endpoint);
    RUN_TEST(test_unreleased_socket_blocks_recreate_until_released);
    RUN_TEST(test_forced_teardown_is_idempotent_and_repeated_cycles_balanced);
    RUN_TEST(test_session_open_never_attempted_while_armed);
    TEST_SUMMARY();
}
