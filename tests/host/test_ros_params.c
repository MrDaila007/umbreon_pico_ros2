/* ROS parameter model against the real shared settings: descriptor table,
 * typed validation, arming/maintenance gates, alias, producer conflicts and
 * bounded reconciliation with a fake parameter-server sink. */
#include "test_runner.h"
#include "ros_params.h"
#include "shared_state.h"
#include "hal_time.h"
#include <math.h>
#include <string.h>

uint32_t hal_now_ms(void) { return 1000; }

static ros_params_t P;
static void setup(void) {
    umb_settings_t s = umb_settings_defaults();
    shared_init(&s);
    ros_params_init(&P, ros_params_shared_io());
}
#define VI(v) (&(ros_param_value_t){.type = ROS_PARAM_INT, .i = (v)})
#define VD(v) (&(ros_param_value_t){.type = ROS_PARAM_DOUBLE, .d = (v)})
static umb_settings_t cur(void) { umb_settings_t s; shared_get_settings(&s); return s; }
static uint32_t rev(void) { umb_settings_t s; return shared_settings_snapshot(&s); }

TEST(test_table_is_complete_unique_and_consistent_with_core) {
    ASSERT_EQ(ros_params_count(), 55);
    for (size_t i = 0; i < ros_params_count(); i++) {
        const ros_param_desc_t *d = ros_params_desc(i);
        ASSERT_TRUE(strlen(d->name) <= 40);
        for (size_t j = 0; j < i; j++) ASSERT_TRUE(strcmp(ros_params_desc(j)->name, d->name) != 0);
        umb_settings_numeric_info_t info;
        ASSERT_TRUE(umb_settings_numeric_info(d->key, &info));
        ASSERT_EQ(info.is_float, d->type == ROS_PARAM_DOUBLE);
        double lo, hi;
        ASSERT_TRUE(ros_params_range(d, &lo, &hi));
        ASSERT_TRUE(lo <= hi);
        ASSERT_TRUE(ros_params_find(d->name) == d);
    }
    /* Not exposed: credentials, the calibration flag, the neutral_point alias target. */
    ASSERT_TRUE(ros_params_find("wifi_ssid") == NULL); ASSERT_TRUE(ros_params_find("wifi_pass") == NULL);
    ASSERT_TRUE(ros_params_find("ap_pass") == NULL); ASSERT_TRUE(ros_params_find("calibrated") == NULL);
    ASSERT_TRUE(ros_params_find("neutral_point") == NULL);
    ASSERT_TRUE(ros_params_find("steer_trim") != NULL);
    ASSERT_TRUE(ros_params_find(NULL) == NULL);
    /* Required groups including reverse limits and reflex settings. */
    static const char *const req[] = {"arming_mode", "cmd_timeout_ms", "max_speed_ms", "max_reverse_speed_ms",
        "max_throttle", "max_throttle_rev", "min_speed", "max_speed", "min_bspeed", "servo_reverse",
        "min_point", "max_point", "pid_kp", "pid_ki", "pid_kd", "spd_slew", "reverse_brake_cmd",
        "reverse_brake_ms", "bat_enabled", "bat_multiplier", "bat_low", "imu_rotate", "wheel_diam_m",
        "encoder_holes", "tach_glitch_filter_us", "collision_sensor_mask", "ttc_ms", "min_front_mm",
        "cap_far_mm", "cap_near_mm", "cap_min_ms", "stall_min_us", "stall_ms", "impact_g", "wifi_mode",
        "sta_timeout_s", "agent_port", "ros_domain_id", "agent_ip"};
    for (size_t i = 0; i < sizeof req / sizeof req[0]; i++) ASSERT_TRUE(ros_params_find(req[i]) != NULL);
    ASSERT_EQ(ros_params_find("servo_reverse")->type, ROS_PARAM_BOOL);
    ASSERT_EQ(ros_params_find("pid_kp")->type, ROS_PARAM_DOUBLE);
    ASSERT_EQ(ros_params_find("agent_ip")->type, ROS_PARAM_INT);
    double lo, hi;
    ASSERT_TRUE(ros_params_range(ros_params_find("agent_ip"), &lo, &hi)); ASSERT_TRUE(hi == 4294967295.0);
    ASSERT_TRUE(ros_params_range(ros_params_find("steer_trim"), &lo, &hi)); ASSERT_EQ(lo, -90); ASSERT_EQ(hi, 90);
    ASSERT_TRUE(ros_params_find("wifi_mode")->network && ros_params_find("agent_ip")->network);
    ASSERT_TRUE(ros_params_find("ros_domain_id")->network && ros_params_find("agent_port")->network);
    ASSERT_FALSE(ros_params_find("pid_kp")->network);
}

TEST(test_valid_scalar_edits_commit_and_read_back) {
    setup();
    uint32_t r0 = rev();
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_INT, false, 400, 0}), ROS_PARAM_OK);
    ASSERT_EQ(cur().cmd_timeout_ms, 400); ASSERT_EQ(rev(), r0 + 1);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 12.5}), ROS_PARAM_OK);
    ASSERT_FLOAT_EQ(cur().pid_kp, 12.5f, 0.0f);
    ASSERT_EQ(ros_params_set(&P, "servo_reverse", &(ros_param_value_t){ROS_PARAM_BOOL, true, 0, 0}), ROS_PARAM_OK);
    ASSERT_TRUE(cur().servo_reverse);
    ASSERT_EQ(ros_params_set(&P, "arming_mode", &(ros_param_value_t){ROS_PARAM_INT, false, 3, 0}), ROS_PARAM_OK);
    ASSERT_EQ(cur().arming_mode, 3);
    ros_param_value_t v;
    umb_settings_t s = cur();
    ASSERT_TRUE(ros_params_read(&s, ros_params_find("cmd_timeout_ms"), &v));
    ASSERT_EQ(v.type, ROS_PARAM_INT); ASSERT_EQ(v.i, 400);
    ASSERT_TRUE(ros_params_matches(&s, ros_params_find("cmd_timeout_ms"), &v));
    ASSERT_TRUE(ros_params_matches(&s, ros_params_find("pid_kp"), &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 12.5}));
    ASSERT_FALSE(ros_params_matches(&s, ros_params_find("pid_kp"), &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 12.6}));
    ASSERT_FALSE(ros_params_matches(&s, ros_params_find("pid_kp"), &(ros_param_value_t){ROS_PARAM_INT, false, 12, 0}));
    /* A double that only differs beyond float precision matches the stored float. */
    s.pid_kp = 1.1f;
    ASSERT_TRUE(ros_params_matches(&s, ros_params_find("pid_kp"), &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1.1}));
}

TEST(test_bad_type_range_nan_inf_overflow_leave_settings_untouched) {
    setup();
    umb_settings_t before = cur(); uint32_t r0 = rev();
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 400}), ROS_PARAM_BAD_TYPE);
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_BOOL, true, 0, 0}), ROS_PARAM_BAD_TYPE);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_INT, false, 3, 0}), ROS_PARAM_BAD_TYPE);
    ASSERT_EQ(ros_params_set(&P, "servo_reverse", &(ros_param_value_t){ROS_PARAM_INT, false, 1, 0}), ROS_PARAM_BAD_TYPE);
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_INT, false, 99, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_INT, false, 1001, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_INT, false, 4294967296ll + 400, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "cmd_timeout_ms", &(ros_param_value_t){ROS_PARAM_INT, false, INT64_MIN, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, NAN}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, INFINITY}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1e300}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, -1}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "arming_mode", &(ros_param_value_t){ROS_PARAM_INT, false, 4, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "agent_ip", &(ros_param_value_t){ROS_PARAM_INT, false, 4294967296ll, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "agent_ip", &(ros_param_value_t){ROS_PARAM_INT, false, -1, 0}), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "no_such_param", &(ros_param_value_t){ROS_PARAM_INT, false, 1, 0}), ROS_PARAM_UNKNOWN);
    ASSERT_EQ(ros_params_set(&P, "wifi_pass", &(ros_param_value_t){ROS_PARAM_INT, false, 1, 0}), ROS_PARAM_UNKNOWN);
    ASSERT_EQ(ros_params_set(&P, NULL, &(ros_param_value_t){ROS_PARAM_INT, false, 1, 0}), ROS_PARAM_UNKNOWN);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", NULL), ROS_PARAM_BAD_TYPE);
    /* Cross-field rule from the core: cap_near must stay below cap_far. */
    ASSERT_EQ(ros_params_set(&P, "cap_near_mm", &(ros_param_value_t){ROS_PARAM_INT, false, 1500, 0}), ROS_PARAM_BAD_VALUE);
    umb_settings_t now_s = cur();
    ASSERT_TRUE(umb_settings_equal(&before, &now_s));
    ASSERT_EQ(rev(), r0);
    ASSERT_EQ(P.set_rejected, 20);
}

TEST(test_agent_ip_int64_forms) {
    setup();
    ASSERT_EQ(ros_params_set(&P, "agent_ip", &(ros_param_value_t){ROS_PARAM_INT, false, 181015702, 0}), ROS_PARAM_OK);
    ASSERT_EQ(cur().agent_ip, (10u << 24) | (202u << 16) | (20u << 8) | 150u);
    ASSERT_EQ(ros_params_set(&P, "agent_ip", &(ros_param_value_t){ROS_PARAM_INT, false, 4294967295ll, 0}), ROS_PARAM_OK);
    ASSERT_EQ(cur().agent_ip, 0xffffffffu);
    ASSERT_EQ(ros_params_set(&P, "agent_ip", &(ros_param_value_t){ROS_PARAM_INT, false, 0, 0}), ROS_PARAM_OK);
    ASSERT_EQ(cur().agent_ip, 0);
    umb_settings_t s = cur(); s.agent_ip = 181015702u;
    ros_param_value_t v; ASSERT_TRUE(ros_params_read(&s, ros_params_find("agent_ip"), &v)); ASSERT_EQ(v.i, 181015702);
}

TEST(test_unknown_create_and_delete_refused_refresh_accepted) {
    setup();
    ros_param_value_t v = *VI(300);
    ASSERT_FALSE(ros_params_on_modification(&P, "brand_new", false, &v));          /* create */
    ASSERT_FALSE(ros_params_on_modification(&P, "cmd_timeout_ms", true, NULL));     /* delete */
    ASSERT_FALSE(ros_params_on_modification(&P, "nope", true, &v));
    ASSERT_FALSE(ros_params_on_modification(&P, "cmd_timeout_ms", true, &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 300}));
    ASSERT_EQ(cur().cmd_timeout_ms, umb_settings_defaults().cmd_timeout_ms);
    ASSERT_TRUE(ros_params_on_modification(&P, "cmd_timeout_ms", true, &v));
    ASSERT_EQ(cur().cmd_timeout_ms, 300);
}

TEST(test_armed_pending_arm_and_maintenance_refuse_every_edit) {
    setup();
    umb_settings_t before = cur();
    umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1}), ROS_PARAM_BUSY);
    ASSERT_EQ(ros_params_set(&P, "arming_mode", &(ros_param_value_t){ROS_PARAM_INT, false, 0, 0}), ROS_PARAM_BUSY);
    ASSERT_FALSE(ros_params_on_modification(&P, "cmd_timeout_ms", true, &(ros_param_value_t){ROS_PARAM_INT, false, 300, 0}));
    umb_settings_t now_s = cur();
    ASSERT_TRUE(umb_settings_equal(&before, &now_s));
    o.armed = false; shared_publish_outputs(&o);
    ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR)); /* pending arm */
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1}), ROS_PARAM_BUSY);
    setup();
    uint32_t token; ASSERT_TRUE(shared_begin_maintenance(&token));
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1}), ROS_PARAM_BUSY);
    shared_end_maintenance(token);
    ASSERT_EQ(ros_params_set(&P, "pid_kp", &(ros_param_value_t){ROS_PARAM_DOUBLE, false, 0, 1}), ROS_PARAM_OK);
}

TEST(test_steer_trim_alias_maps_to_neutral_point_both_ways) {
    setup();
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(5)), ROS_PARAM_OK);
    ASSERT_EQ(cur().neutral_point, 95);
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(-12)), ROS_PARAM_OK);
    ASSERT_EQ(cur().neutral_point, 78);
    umb_settings_t s = cur(); ros_param_value_t v;
    ASSERT_TRUE(ros_params_read(&s, ros_params_find("steer_trim"), &v)); ASSERT_EQ(v.i, -12);
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(91)), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(-91)), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(INT64_MAX)), ROS_PARAM_BAD_VALUE);
    ASSERT_EQ(cur().neutral_point, 78);
    /* Cross-field: neutral must stay inside min/max point. */
    ASSERT_EQ(ros_params_set(&P, "min_point", VI(80)), ROS_PARAM_BAD_VALUE); /* neutral 78 below min */
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(0)), ROS_PARAM_OK);
    ASSERT_EQ(ros_params_set(&P, "min_point", VI(60)), ROS_PARAM_OK);
    ASSERT_EQ(ros_params_set(&P, "steer_trim", VI(-40)), ROS_PARAM_BAD_VALUE); /* 50 < min_point 60 */
    ASSERT_TRUE(ros_params_matches(&s, ros_params_find("steer_trim"), VI(-12)));
    ASSERT_FALSE(ros_params_matches(&s, ros_params_find("steer_trim"), VI(78)));
}

TEST(test_all_twelve_reflex_policies_map_to_mode_and_index) {
    static const char *const names[UMB_REFLEX_COUNT] = {"battery", "collision", "speed_cap", "stall", "impact", "sensor_fault"};
    int found = 0;
    for (int id = 0; id < UMB_REFLEX_COUNT; id++) {
        for (int manual = 0; manual < 2; manual++) {
            char name[48];
            snprintf(name, sizeof name, "reflex.%s.%s", names[id], manual ? "manual" : "auto");
            setup();
            umb_settings_t base = cur();
            ASSERT_TRUE(ros_params_find(name) != NULL); found++;
            ASSERT_EQ(ros_params_find(name)->type, ROS_PARAM_INT);
            double lo, hi; ASSERT_TRUE(ros_params_range(ros_params_find(name), &lo, &hi));
            ASSERT_EQ(lo, 0); ASSERT_EQ(hi, 3);
            ASSERT_EQ(ros_params_set(&P, name, VI(0)), ROS_PARAM_OK);   /* off */
            ASSERT_EQ(ros_params_set(&P, name, VI(3)), ROS_PARAM_OK);   /* ros */
            ASSERT_EQ(ros_params_set(&P, name, VI(4)), ROS_PARAM_BAD_VALUE);
            ASSERT_EQ(ros_params_set(&P, name, VI(-1)), ROS_PARAM_BAD_VALUE);
            ASSERT_EQ(ros_params_set(&P, name, VI(2)), ROS_PARAM_OK);   /* pico */
            umb_settings_t s = cur();
            for (int j = 0; j < UMB_REFLEX_COUNT; j++) {
                ASSERT_EQ(s.reflex_manual[j], j == id && manual ? 2 : base.reflex_manual[j]);
                ASSERT_EQ(s.reflex_auto[j], j == id && !manual ? 2 : base.reflex_auto[j]);
            }
        }
    }
    ASSERT_EQ(found, 12);
}

/* --- conflicting producers --- */
static int conflicts_to_inject;
static uint32_t inject_snapshot(umb_settings_t *o) { return shared_settings_snapshot(o); }
static bool inject_commit(const umb_settings_t *s, uint32_t revision) {
    if (conflicts_to_inject > 0) { /* console edit lands between snapshot and commit */
        conflicts_to_inject--;
        umb_settings_t c; uint32_t r = shared_settings_snapshot(&c);
        c.max_speed_ms = 2.5f + 0.5f * (float)(conflicts_to_inject % 2);
        (void)shared_commit_settings(&c, r, false);
    }
    return shared_commit_settings(s, revision, false);
}
static bool inject_writable(void) { return shared_settings_write_allowed(); }
static const ros_params_io_t INJECT = {inject_snapshot, inject_commit, inject_writable};

TEST(test_concurrent_console_edit_is_never_overwritten) {
    setup(); P.io = &INJECT; conflicts_to_inject = 1;
    ASSERT_EQ(ros_params_set(&P, "pid_kp", VD(7)), ROS_PARAM_OK);       /* retried on the fresh snapshot */
    ASSERT_FLOAT_EQ(cur().pid_kp, 7.0f, 0.0f);
    ASSERT_FLOAT_EQ(cur().max_speed_ms, 2.5f, 0.0f);                    /* console edit survived */
    ASSERT_EQ(P.set_conflicts, 1);
    setup(); P.io = &INJECT; conflicts_to_inject = 100;
    float kp = cur().pid_kp;
    ASSERT_EQ(ros_params_set(&P, "pid_kp", VD(7)), ROS_PARAM_CONFLICT);  /* bounded retries, no write */
    ASSERT_FLOAT_EQ(cur().pid_kp, kp, 0.0f);
    ASSERT_EQ(conflicts_to_inject, 100 - (int)ROS_PARAMS_COMMIT_ATTEMPTS);
}

/* --- reconciliation with a fake parameter server --- */
typedef struct { ros_param_value_t v[64]; bool have[64]; unsigned puts, gets; bool fail_put; bool armed_checks; } fake_srv_t;
static fake_srv_t SRV;
static size_t idx(const ros_param_desc_t *d) { for (size_t i = 0; i < ros_params_count(); i++) if (ros_params_desc(i) == d) return i; return 99; }
static bool sink_get(void *c, const ros_param_desc_t *d, ros_param_value_t *o) {
    fake_srv_t *s = c; s->gets++; if (!s->have[idx(d)]) return false; *o = s->v[idx(d)]; return true;
}
static bool sink_put(void *c, const ros_param_desc_t *d, const ros_param_value_t *v) {
    fake_srv_t *s = c; s->puts++;
    if (s->fail_put) return false;
    /* rclc runs the modification callback for internal sets too. */
    if (!ros_params_on_modification(&P, d->name, s->have[idx(d)], v)) return false;
    s->v[idx(d)] = *v; s->have[idx(d)] = true; return true;
}
static const ros_param_sink_t SINK = {sink_get, sink_put, &SRV};
static void srv_reset(void) { memset(&SRV, 0, sizeof SRV); }

TEST(test_initial_population_then_idle_and_bounded_chunks) {
    setup(); srv_reset();
    unsigned calls = 0;
    while (!ros_params_reconcile(&P, &SINK, ROS_PARAMS_RECONCILE_BUDGET, 64)) {
        ASSERT_TRUE(SRV.gets <= (calls + 1) * ROS_PARAMS_RECONCILE_BUDGET); /* bounded work per call */
        ASSERT_TRUE(++calls < 20);
    }
    ASSERT_EQ(calls, (ros_params_count() - 1) / ROS_PARAMS_RECONCILE_BUDGET); /* 7 partial calls then done */
    ASSERT_EQ(SRV.puts, ros_params_count());
    unsigned puts = SRV.puts;
    ASSERT_TRUE(ros_params_reconcile(&P, &SINK, 1, 64)); /* up to date: no work, no writes */
    ASSERT_EQ(SRV.puts, puts);
    ASSERT_EQ(SRV.v[idx(ros_params_find("cmd_timeout_ms"))].i, 250);
}

TEST(test_console_update_is_read_on_next_reconcile_even_while_armed) {
    setup(); srv_reset();
    ASSERT_TRUE(ros_params_reconcile(&P, &SINK, 64, 64));
    umb_settings_t s; uint32_t r = shared_settings_snapshot(&s);
    s.cmd_timeout_ms = 420; s.neutral_point = 83; s.reflex_auto[UMB_REFLEX_STALL] = 3;
    ASSERT_TRUE(shared_commit_settings(&s, r, false));
    umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);   /* internal refresh is not an external write */
    unsigned puts = SRV.puts;
    ASSERT_TRUE(ros_params_reconcile(&P, &SINK, 64, 64));
    ASSERT_EQ(SRV.puts, puts + 3);                                   /* only the three that differ */
    ASSERT_EQ(SRV.v[idx(ros_params_find("cmd_timeout_ms"))].i, 420);
    ASSERT_EQ(SRV.v[idx(ros_params_find("steer_trim"))].i, -7);
    ASSERT_EQ(SRV.v[idx(ros_params_find("reflex.stall.auto"))].i, 3);
    ASSERT_FALSE(P.refreshing);
    /* But an external write is still refused while armed. */
    ASSERT_FALSE(ros_params_on_modification(&P, "cmd_timeout_ms", true, VI(300)));
}

static void console_edit_hook(void) {
    umb_settings_t s; uint32_t r = shared_settings_snapshot(&s);
    s.ttc_ms = 777; (void)shared_commit_settings(&s, r, false);
}
TEST(test_pass_restarts_when_settings_change_mid_pass) {
    setup(); srv_reset();
    ASSERT_FALSE(ros_params_reconcile(&P, &SINK, 8, 64));      /* pass in progress at revision R */
    console_edit_hook();                                    /* newer concurrent setting */
    unsigned guard = 0;
    while (!ros_params_reconcile(&P, &SINK, 8, 64)) ASSERT_TRUE(++guard < 30);
    ASSERT_EQ(P.refresh_restarts, 1);
    ASSERT_EQ(SRV.v[idx(ros_params_find("ttc_ms"))].i, 777); /* fresh value, never the stale copy */
    ASSERT_EQ(cur().ttc_ms, 777);
}

TEST(test_ros_edit_mid_pass_is_not_reverted_by_stale_copy) {
    setup(); srv_reset();
    ASSERT_TRUE(ros_params_reconcile(&P, &SINK, 64, 64));
    ASSERT_EQ(cur().cmd_timeout_ms, 250);
    ASSERT_TRUE(ros_params_on_modification(&P, "stall_ms", true, VI(900)));   /* external edit */
    SRV.v[idx(ros_params_find("stall_ms"))] = *VI(900);                          /* rclc stores it */
    ASSERT_FALSE(ros_params_reconcile(&P, &SINK, 2, 64));                          /* pass starts */
    ASSERT_TRUE(ros_params_on_modification(&P, "cmd_timeout_ms", true, VI(500)));
    SRV.v[idx(ros_params_find("cmd_timeout_ms"))] = *VI(500);
    unsigned guard = 0;
    while (!ros_params_reconcile(&P, &SINK, 2, 64)) ASSERT_TRUE(++guard < 60);
    ASSERT_EQ(SRV.v[idx(ros_params_find("cmd_timeout_ms"))].i, 500);
    ASSERT_EQ(SRV.v[idx(ros_params_find("stall_ms"))].i, 900);
}

TEST(test_put_failure_retries_and_new_server_instance_repopulates) {
    setup(); srv_reset();
    SRV.fail_put = true;
    ASSERT_FALSE(ros_params_reconcile(&P, &SINK, 64, 64));
    ASSERT_TRUE(P.refresh_failures >= 1); ASSERT_FALSE(P.refreshing);
    SRV.fail_put = false;
    unsigned guard = 0;
    while (!ros_params_reconcile(&P, &SINK, 64, 64)) ASSERT_TRUE(++guard < 5);
    ASSERT_TRUE(SRV.puts >= ros_params_count());
    ros_params_invalidate(&P); srv_reset();                 /* session recreated: server is empty */
    while (!ros_params_reconcile(&P, &SINK, 64, 64)) ASSERT_TRUE(++guard < 9);
    ASSERT_EQ(SRV.puts, ros_params_count());
}

TEST(test_server_writes_per_iteration_are_bounded) {
    setup(); srv_reset();
    ASSERT_TRUE(ros_params_reconcile(&P, &SINK, 64, 64));
    umb_settings_t s; uint32_t r = shared_settings_snapshot(&s);
    s.cmd_timeout_ms = 300; s.ttc_ms = 600; s.stall_ms = 700; s.min_front_mm = 90; s.cap_far_mm = 900;
    s.encoder_holes = 20; s.kick_ms = 40; s.bat_low = 6.5f; s.impact_g = 4;
    ASSERT_TRUE(shared_commit_settings(&s, r, false));
    unsigned calls = 0, before = SRV.puts;
    while (!ros_params_reconcile(&P, &SINK, ROS_PARAMS_RECONCILE_BUDGET, ROS_PARAMS_WRITE_BUDGET)) {
        ASSERT_TRUE(SRV.puts - before <= ROS_PARAMS_WRITE_BUDGET); before = SRV.puts;
        ASSERT_TRUE(++calls < 40);
    }
    ASSERT_EQ(SRV.v[idx(ros_params_find("impact_g"))].d, 4.0);
    ASSERT_EQ(SRV.v[idx(ros_params_find("bat_low"))].d, (double)6.5f);
    ASSERT_EQ(SRV.v[idx(ros_params_find("encoder_holes"))].i, 20);
    ASSERT_TRUE(calls >= 3);
}

/* Low-memory rclc hands the callback one parameter at a time, and a multi-entry
 * request is judged entry by entry: an accepted entry is not rolled back. */
TEST(test_batch_is_judged_per_parameter_and_not_atomic) {
    setup();
    struct { const char *name; ros_param_value_t v; } batch[] = {
        {"cmd_timeout_ms", {ROS_PARAM_INT, false, 500, 0}},
        {"pid_kp", {ROS_PARAM_DOUBLE, false, 0, NAN}},
        {"ttc_ms", {ROS_PARAM_INT, false, 800, 0}},
    };
    bool accepted[3];
    for (int i = 0; i < 3; i++) accepted[i] = ros_params_on_modification(&P, batch[i].name, true, &batch[i].v);
    ASSERT_TRUE(accepted[0]); ASSERT_FALSE(accepted[1]); ASSERT_TRUE(accepted[2]);
    ASSERT_EQ(cur().cmd_timeout_ms, 500); ASSERT_EQ(cur().ttc_ms, 800); /* first stays applied */
}

int main(void) {
    RUN_TEST(test_table_is_complete_unique_and_consistent_with_core);
    RUN_TEST(test_valid_scalar_edits_commit_and_read_back);
    RUN_TEST(test_bad_type_range_nan_inf_overflow_leave_settings_untouched);
    RUN_TEST(test_agent_ip_int64_forms);
    RUN_TEST(test_unknown_create_and_delete_refused_refresh_accepted);
    RUN_TEST(test_armed_pending_arm_and_maintenance_refuse_every_edit);
    RUN_TEST(test_steer_trim_alias_maps_to_neutral_point_both_ways);
    RUN_TEST(test_all_twelve_reflex_policies_map_to_mode_and_index);
    RUN_TEST(test_concurrent_console_edit_is_never_overwritten);
    RUN_TEST(test_initial_population_then_idle_and_bounded_chunks);
    RUN_TEST(test_console_update_is_read_on_next_reconcile_even_while_armed);
    RUN_TEST(test_pass_restarts_when_settings_change_mid_pass);
    RUN_TEST(test_ros_edit_mid_pass_is_not_reverted_by_stale_copy);
    RUN_TEST(test_batch_is_judged_per_parameter_and_not_atomic);
    RUN_TEST(test_server_writes_per_iteration_are_bounded);
    RUN_TEST(test_put_failure_retries_and_new_server_instance_repopulates);
    TEST_SUMMARY();
}
