/* Production ROS graph contract and partial-creation tracker under a fake
 * rcl/rclc backend: a failure is injected after every successful predecessor. */
#include "test_runner.h"
#include "ros_entities.h"
#include "fake_ros_backend.h"
#include <string.h>

static const ros_session_config_t CFG = {0x0a000005u, 8888, 7, 0x1234abcdu};
static fake_ros_t f;
static ros_backend_t b;

static void setup(void) { fake_ros_reset(&f); b = fake_ros_backend(&f); }
static const char *name(ros_entity_id_t id) { return ros_entity_desc(id)->name; }

TEST(test_graph_names_qos_and_counts_match_contract) {
    static const char *const ranges[6] = {"/umbreon/range/hr", "/umbreon/range/fr", "/umbreon/range/r",
                                          "/umbreon/range/l", "/umbreon/range/fl", "/umbreon/range/hl"};
    for (int i = 0; i < 6; i++) {
        const ros_entity_desc_t *d = ros_entity_desc((ros_entity_id_t)(ROS_ENT_PUB_RANGE_HR + i));
        ASSERT_TRUE(strcmp(d->name, ranges[i]) == 0); ASSERT_EQ(d->type, ROS_TYPE_RANGE); ASSERT_EQ(d->index, i);
    }
    ASSERT_TRUE(strcmp(name(ROS_ENT_PUB_IMU), "/umbreon/imu/data_raw") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_PUB_ODOM), "/umbreon/odom") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_PUB_BATTERY), "/umbreon/battery") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_PUB_REFLEX), "/umbreon/reflex") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_PUB_DIAGNOSTICS), "/umbreon/diagnostics") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_SUB_CMD_VEL), "/umbreon/cmd_vel") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_SUB_CMD_TELEOP), "/umbreon/cmd_teleop") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_SUB_CMD_RAW), "/umbreon/cmd_raw") == 0);
    ASSERT_TRUE(strcmp(name(ROS_ENT_SRV_ARM), "/umbreon/arm") == 0);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_PUB_IMU)->type, ROS_TYPE_IMU);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_PUB_ODOM)->type, ROS_TYPE_ODOMETRY);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_PUB_BATTERY)->type, ROS_TYPE_BATTERY);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_PUB_REFLEX)->type, ROS_TYPE_DIAG_STATUS);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_PUB_DIAGNOSTICS)->type, ROS_TYPE_DIAG_ARRAY);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_SRV_ARM)->type, ROS_TYPE_SET_BOOL);
    ASSERT_TRUE(strcmp(name(ROS_ENT_SRV_SAVE), "/umbreon/save_settings") == 0);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_SRV_SAVE)->type, ROS_TYPE_TRIGGER);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_EXEC_SAVE)->target, ROS_ENT_SRV_SAVE);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_SUB_CMD_VEL)->index, 0);   /* UMB_ROS_CMD_VEL */
    ASSERT_EQ(ros_entity_desc(ROS_ENT_SUB_CMD_TELEOP)->index, 1);
    ASSERT_EQ(ros_entity_desc(ROS_ENT_SUB_CMD_RAW)->index, 2);
    unsigned pubs = 0, subs = 0, srvs = 0, timers = 0, handles = 0, params = 0, weight = 0;
    for (int id = 0; id < ROS_ENT_COUNT; id++) {
        const ros_entity_desc_t *d = ros_entity_desc((ros_entity_id_t)id);
        ASSERT_TRUE(d != NULL);
        if (d->name) ASSERT_TRUE(strncmp(d->name, "/umbreon/", 9) == 0); /* absolute: no doubled namespace */
        switch (d->kind) {
        case ROS_KIND_PUBLISHER: pubs++; ASSERT_EQ(d->reliability, ROS_QOS_BEST_EFFORT); break;
        case ROS_KIND_SUBSCRIPTION: subs++; ASSERT_EQ(d->reliability, ROS_QOS_BEST_EFFORT);
            ASSERT_EQ(d->depth, 1); ASSERT_EQ(d->type, ROS_TYPE_TWIST); break;
        case ROS_KIND_SERVICE: srvs++; ASSERT_EQ(d->reliability, ROS_QOS_RELIABLE);
            /* Contract: reliable service responses confirm within <= 20 ms. */
            ASSERT_TRUE(d->reply_timeout_ms > 0 && d->reply_timeout_ms <= 20); break;
        case ROS_KIND_PARAM_SERVER: params++;
            /* rclc low-memory server: 6 services + event publisher, created in one bounded step. */
            ASSERT_EQ(d->reliability, ROS_QOS_RELIABLE);
            ASSERT_TRUE(d->reply_timeout_ms > 0 && d->reply_timeout_ms <= 20);
            ASSERT_TRUE(strcmp(d->name, "/umbreon/pico") == 0); break;
        case ROS_KIND_TIMER: timers++; break;
        case ROS_KIND_EXECUTOR_HANDLE: {
            handles++;
            weight += ros_entity_desc(d->target)->kind == ROS_KIND_PARAM_SERVER ? ROS_PARAM_SERVER_HANDLES : 1u;
            /* Dispatches an entity created earlier, after the executor itself. */
            ASSERT_TRUE(d->target < ROS_ENT_EXECUTOR && id > ROS_ENT_EXECUTOR);
            ros_entity_kind_t k = ros_entity_desc(d->target)->kind;
            ASSERT_TRUE(k == ROS_KIND_SUBSCRIPTION || k == ROS_KIND_SERVICE || k == ROS_KIND_TIMER ||
                      k == ROS_KIND_PARAM_SERVER);
            break;
        }
        default: ASSERT_TRUE(d->name == NULL); break;
        }
    }
    ASSERT_EQ(pubs, 11); ASSERT_EQ(subs, 3); ASSERT_EQ(srvs, 2); ASSERT_EQ(timers, 1);
    ASSERT_EQ(params, 1); ASSERT_EQ(handles, subs + srvs + params + timers);
    /* Executor capacity: 3 subs + arm + save + 6 parameter handles + timer = 12 of 12. */
    ASSERT_EQ(ROS_EXECUTOR_HANDLES, 12); ASSERT_EQ(weight, 12); ASSERT_TRUE(weight <= ROS_EXECUTOR_HANDLES);
    ASSERT_EQ(ROS_PARAM_SERVER_HANDLES, 6);
    ASSERT_TRUE(ROS_ENT_PARAM_SERVER > ROS_ENT_SRV_ARM && ROS_ENT_PARAM_SERVER < ROS_ENT_TIMER);
    ASSERT_TRUE(ROS_PARAM_CREATE_BUDGET_MS < 500u - 100u); /* inside one core-0 beat window */
    ASSERT_TRUE(ros_entity_desc(ROS_ENT_COUNT) == NULL);
    ASSERT_TRUE(strcmp(ROS_NODE_NAME, "pico") == 0 && strcmp(ROS_NODE_NAMESPACE, "/umbreon") == 0);
}

static void expect_reverse_cleanup(int created_upto /* exclusive */) {
    /* Log after creation: t (iff context exists), d for created ids high->low, r. */
    unsigned i = 0;
    while (i < f.len && f.op[i] == 'c') i++;
    if (created_upto > ROS_ENT_CONTEXT) { ASSERT_EQ(f.op[i], 't'); i++; }
    for (int id = created_upto - 1; id >= 0; id--, i++) {
        ASSERT_EQ(f.op[i], 'd'); ASSERT_EQ(f.id[i], id);
    }
    ASSERT_EQ(f.op[i], 'r'); ASSERT_EQ(i + 1, f.len);
}

TEST(test_full_create_then_reverse_teardown) {
    setup(); ros_entities_t e; ros_entities_init(&e);
    ASSERT_FALSE(ros_entities_busy(&e));
    for (int id = 0; id < ROS_ENT_COUNT - 1; id++) {
        ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_PROGRESS);
        ASSERT_EQ(f.creates, id + 1); /* exactly one entity per call */
        ASSERT_FALSE(ros_entities_complete(&e)); ASSERT_TRUE(ros_entities_busy(&e));
    }
    ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_DONE);
    ASSERT_TRUE(ros_entities_complete(&e));
    ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_DONE); /* no extra creates */
    ASSERT_EQ(f.creates, ROS_ENT_COUNT);
    ASSERT_EQ(f.last_cfg.client_key, CFG.client_key); ASSERT_EQ(f.last_cfg.domain_id, 7);
    ASSERT_TRUE(ros_entities_destroy(&e, &b));
    expect_reverse_cleanup(ROS_ENT_COUNT);
    ASSERT_EQ(f.live, 0); ASSERT_EQ(f.destroy_without_create, 0); ASSERT_EQ(f.destroys_before_teardown, 0);
    ASSERT_FALSE(ros_entities_busy(&e)); ASSERT_FALSE(f.transport_open);
}

TEST(test_failure_after_each_successful_step_finalizes_exactly_predecessors) {
    for (int fail = 0; fail < ROS_ENT_COUNT; fail++) {
        setup(); f.fail_create_id = fail;
        ros_entities_t e; ros_entities_init(&e);
        ros_create_result_t r = ROS_CREATE_PROGRESS;
        int calls = 0;
        while (r == ROS_CREATE_PROGRESS && calls < ROS_ENT_COUNT + 2) { r = ros_entities_create_next(&e, &b, &CFG); calls++; }
        ASSERT_EQ(r, ROS_CREATE_FAILED); ASSERT_EQ(calls, fail + 1);
        /* Failed tracker never creates again before teardown. */
        unsigned creates = f.creates;
        ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_FAILED); ASSERT_EQ(f.creates, creates);
        ASSERT_TRUE(ros_entities_busy(&e) || fail == 0);
        ASSERT_TRUE(ros_entities_destroy(&e, &b));
        expect_reverse_cleanup(fail);
        ASSERT_EQ(f.live, 0); ASSERT_EQ(f.destroy_without_create, 0);
        ASSERT_EQ(f.destroys_before_teardown, 0); ASSERT_EQ(f.teardown_after_destroy, 0);
        ASSERT_FALSE(f.transport_open); ASSERT_FALSE(ros_entities_busy(&e));
        /* Destroying again is a no-op for entities (only the idempotent release). */
        unsigned d = fake_ros_count(&f, 'd');
        ASSERT_TRUE(ros_entities_destroy(&e, &b)); ASSERT_EQ(fake_ros_count(&f, 'd'), d);
        /* The same tracker builds a complete fresh graph afterwards. */
        f.fail_create_id = -1; f.len = 0; f.teardown_begun = false;
        while ((r = ros_entities_create_next(&e, &b, &CFG)) == ROS_CREATE_PROGRESS) {}
        ASSERT_EQ(r, ROS_CREATE_DONE); ASSERT_EQ(f.live, (1u << ROS_ENT_COUNT) - 1u);
        ASSERT_EQ(f.creates_while_live_transport, 0);
        ASSERT_TRUE(ros_entities_destroy(&e, &b)); ASSERT_EQ(f.live, 0);
    }
}

TEST(test_unreleased_socket_keeps_tracker_busy_without_double_fini) {
    setup(); ros_entities_t e; ros_entities_init(&e);
    while (ros_entities_create_next(&e, &b, &CFG) == ROS_CREATE_PROGRESS) {}
    f.release_ok = false;
    ASSERT_FALSE(ros_entities_destroy(&e, &b));
    ASSERT_EQ(f.live, 0); ASSERT_TRUE(f.transport_open); ASSERT_TRUE(ros_entities_busy(&e));
    ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_FAILED); /* never over an owned socket */
    unsigned d = fake_ros_count(&f, 'd'), t = fake_ros_count(&f, 't');
    ASSERT_FALSE(ros_entities_destroy(&e, &b));
    f.release_ok = true;
    ASSERT_TRUE(ros_entities_destroy(&e, &b));
    ASSERT_EQ(fake_ros_count(&f, 'd'), d); ASSERT_EQ(fake_ros_count(&f, 't'), t);
    ASSERT_EQ(f.destroy_without_create, 0); ASSERT_FALSE(ros_entities_busy(&e)); ASSERT_FALSE(f.transport_open);
}

TEST(test_dead_agent_open_failure_still_releases_socket) {
    setup(); f.agent_alive = false; ros_entities_t e; ros_entities_init(&e);
    ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_PROGRESS); /* init options are local */
    ASSERT_EQ(ros_entities_create_next(&e, &b, &CFG), ROS_CREATE_FAILED);
    ASSERT_TRUE(f.transport_open);
    ASSERT_TRUE(ros_entities_destroy(&e, &b));
    expect_reverse_cleanup(ROS_ENT_CONTEXT);
    ASSERT_FALSE(f.transport_open);
}

TEST(test_repeated_cycles_stay_balanced) {
    setup(); ros_entities_t e; ros_entities_init(&e);
    for (int cycle = 0; cycle < 100; cycle++) {
        f.fail_create_id = cycle % 3 ? (cycle * 7) % ROS_ENT_COUNT : -1; f.teardown_begun = false;
        while (ros_entities_create_next(&e, &b, &CFG) == ROS_CREATE_PROGRESS) {}
        ASSERT_TRUE(ros_entities_destroy(&e, &b));
        ASSERT_EQ(f.live, 0); ASSERT_FALSE(f.transport_open);
    }
    ASSERT_EQ(f.destroy_without_create, 0); ASSERT_EQ(f.creates_while_live_transport, 0);
}

int main(void) {
    RUN_TEST(test_graph_names_qos_and_counts_match_contract);
    RUN_TEST(test_full_create_then_reverse_teardown);
    RUN_TEST(test_failure_after_each_successful_step_finalizes_exactly_predecessors);
    RUN_TEST(test_unreleased_socket_keeps_tracker_busy_without_double_fini);
    RUN_TEST(test_dead_agent_open_failure_still_releases_socket);
    RUN_TEST(test_repeated_cycles_stay_balanced);
    TEST_SUMMARY();
}
