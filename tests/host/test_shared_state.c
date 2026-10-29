#include "test_runner.h"
#include "shared_state.h"
#include "hal_time.h"
#include "umb_tof_array.h"

static uint32_t now;
uint32_t hal_now_ms(void) { return now; }

static void seed(uint32_t stamp) {
    now = stamp;
    umb_settings_t settings = umb_settings_defaults();
    shared_init(&settings);
    shared_sensors_t s = {0};
    s.tof_online = 6;
    s.tof_online_mask = 0x3f;
    s.imu_ok = true;
    s.imu_stamp_ms = stamp;
    for (int i = 0; i < 6; i++) {
        s.tof_mm[i] = 100 + i;
        s.tof_stamp_ms[i] = stamp;
    }
    shared_publish_sensors(&s);
}

TEST(test_snapshot_stalled_producers_expire) {
    seed(100);
    shared_sensors_t s;
    now = 200;
    shared_get_sensors(&s);
    ASSERT_TRUE(s.imu_ok);
    now = 201;
    shared_get_sensors(&s);
    ASSERT_FALSE(s.imu_ok);
    ASSERT_EQ(s.tof_online, 6);
    now = 1600;
    shared_get_sensors(&s);
    ASSERT_EQ(s.tof_online, 6);
    now = 1601;
    shared_get_sensors(&s);
    ASSERT_EQ(s.tof_online, 0);
    ASSERT_EQ(s.tof_online_mask, 0);
    for (int i = 0; i < 6; i++) ASSERT_EQ(s.tof_mm[i], UMB_TOF_INVALID_MM);
}

TEST(test_snapshot_oor_still_online_and_offline_not_counted) {
    seed(100);
    shared_sensors_t s;
    shared_get_sensors(&s);
    s.tof_mm[0] = UMB_TOF_INVALID_MM; /* fresh out-of-range is healthy */
    s.tof_online_mask &= (uint8_t)~(1u << 5); /* last device failed */
    s.tof_stamp_ms[1] = 0; /* this reading expires first */
    shared_publish_sensors(&s);
    now = 1501;
    shared_get_sensors(&s);
    ASSERT_EQ(s.tof_online, 4);
    ASSERT_EQ(s.tof_mm[0], UMB_TOF_INVALID_MM);
    ASSERT_EQ(s.tof_mm[1], UMB_TOF_INVALID_MM);
    ASSERT_EQ(s.tof_mm[2], 102);
    ASSERT_EQ(s.tof_mm[5], UMB_TOF_INVALID_MM);
}

TEST(test_snapshot_expiry_wrap_safe) {
    seed(UINT32_MAX - 50);
    shared_sensors_t s;
    now = 49; /* age 100 ms */
    shared_get_sensors(&s);
    ASSERT_TRUE(s.imu_ok);
    now = 50;
    shared_get_sensors(&s);
    ASSERT_FALSE(s.imu_ok);
    now = 1450; /* age 1501 ms */
    shared_get_sensors(&s);
    ASSERT_EQ(s.tof_online, 0);
}

TEST(test_disarm_bypasses_full_arm_queue) {
    seed(100);
    for (int i = 0; i < 8; i++) shared_post_arm_request(SHARED_ARM_CAR);
    shared_post_arm_request(SHARED_ARM_DISARM);
    shared_arm_request_t request;
    ASSERT_TRUE(shared_take_arm_record(&request, now));
    ASSERT_EQ(request.kind, SHARED_ARM_DISARM);
    ASSERT_FALSE(shared_take_arm_record(&request, now));
}

TEST(test_loss_is_sticky_and_old_session_is_rejected) {
    seed(100);
    shared_set_link(true, true);
    uint32_t old = shared_session_generation();
    umb_cmd_t cmd = {UMB_CMD_RAW, UMB_SRC_ROS, UMB_DRIVE_MANUAL, 0, 0.5f, now};
    ASSERT_TRUE(shared_post_ros_cmd(&cmd, old));
    shared_post_arm_request(SHARED_ARM_CAR);
    shared_set_link(false, false);
    shared_set_link(true, true); /* restore before the control task gets a turn */
    ASSERT_TRUE(shared_session_generation() != old);
    ASSERT_FALSE(shared_post_ros_cmd(&cmd, old));
    shared_control_mailbox_t box;
    shared_take_control_mailbox(&box);
    ASSERT_FALSE(box.cmd_pending);
    ASSERT_EQ(box.loss_mask, UMB_STOP_AGENT | UMB_STOP_WIFI);
    shared_take_control_mailbox(&box);
    ASSERT_EQ(box.loss_mask, 0);
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    ASSERT_EQ(r.kind, SHARED_ARM_DISARM);
    ASSERT_FALSE(shared_take_arm_record(&r, now));
    ASSERT_TRUE(shared_post_ros_cmd(&cmd, shared_session_generation()));
}

TEST(test_boot_offline_does_not_fabricate_loss) {
    seed(100); shared_set_link(false, false);
    shared_control_mailbox_t box; shared_take_control_mailbox(&box);
    ASSERT_EQ(box.loss_mask, 0);
    ASSERT_EQ(shared_session_generation(), 1);
    umb_cmd_t cmd = {UMB_CMD_RAW, UMB_SRC_CONSOLE, UMB_DRIVE_MANUAL, 0, 0, now};
    ASSERT_TRUE(shared_post_cmd(&cmd));
}

TEST(test_cancelled_and_expired_arm_candidates_cannot_commit) {
    seed(100);
    uint32_t id;
    ASSERT_TRUE(shared_submit_arm(SHARED_ARM_CAR, 0, now, &id));
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    ASSERT_TRUE(shared_cancel_arm(id)); /* cancellation after pop, before core commit */
    umb_settings_t cfg = umb_settings_defaults();
    umb_arming_t a = umb_arming_car_key(umb_arming_init(cfg.arming_mode), NULL);
    umb_outputs_t out = {0};
    umb_arm_result_t result = UMB_ARM_WAITING_ROS;
    ASSERT_FALSE(shared_commit_arm_batch(&r, &result, 1, &out, &a, now));
    shared_arm_ack_t ack;
    ASSERT_TRUE(shared_get_arm_ack(id, &ack));
    ASSERT_EQ(ack.state, SHARED_REQUEST_CANCELLED);
    shared_publish_outputs(&out);
    ASSERT_TRUE(shared_submit_arm(SHARED_ARM_CAR, 0, now, &id));
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    ASSERT_FALSE(shared_commit_arm_batch(&r, &result, 1, &out, &a, now + 60));
    shared_publish_outputs(&out);
    ASSERT_TRUE(shared_submit_arm(SHARED_ARM_CAR, 0, now, &id));
    ASSERT_FALSE(shared_take_arm_record(&r, now + 60));
    ASSERT_TRUE(shared_get_arm_ack(id, &ack));
    ASSERT_EQ(ack.state, SHARED_REQUEST_CANCELLED);
}

TEST(test_arm_ack_is_core_decision_and_old_generation_refused) {
    seed(100); shared_set_link(true, true);
    uint32_t id, generation = shared_session_generation();
    ASSERT_TRUE(shared_submit_arm(SHARED_ARM_ROS, generation, now, &id));
    shared_arm_ack_t ack;
    ASSERT_TRUE(shared_get_arm_ack(id, &ack));
    ASSERT_EQ(ack.state, SHARED_REQUEST_PENDING);
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    umb_arming_t a = umb_arming_ros(umb_arming_init(UMB_ARM_BOTH), true, NULL);
    umb_outputs_t out = {0}; umb_arm_result_t result = UMB_ARM_WAITING_CAR;
    ASSERT_TRUE(shared_commit_arm_batch(&r, &result, 1, &out, &a, now));
    ASSERT_FALSE(shared_cancel_arm(id)); /* cannot report timeout after commit won */
    ASSERT_TRUE(shared_get_arm_ack(id, &ack));
    ASSERT_EQ(ack.state, SHARED_REQUEST_DONE);
    ASSERT_TRUE(ack.ros_key);
    ASSERT_FALSE(ack.car_key);
    shared_set_link(false, false);
    shared_set_link(true, true);
    ASSERT_FALSE(shared_submit_arm(SHARED_ARM_ROS, generation, now, &id));
}

TEST(test_settings_revision_prevents_stale_writer_and_mode_exception_is_narrow) {
    seed(100);
    umb_settings_t first, stale;
    uint32_t revision = shared_settings_snapshot(&first);
    stale = first;
    first.max_speed_ms = 0.8f;
    ASSERT_TRUE(shared_commit_settings(&first, revision, false));
    stale.bat_low = 6.1f;
    ASSERT_FALSE(shared_commit_settings(&stale, revision, false));
    revision = shared_settings_snapshot(&first);
    umb_outputs_t out = {.armed = true}; shared_publish_outputs(&out);
    first.bat_low = 6.2f;
    ASSERT_FALSE(shared_commit_settings(&first, revision, false));
    ASSERT_FALSE(shared_commit_settings(&first, revision, true)); /* mode-only cannot smuggle edits */
    revision = shared_settings_snapshot(&first);
    first.arming_mode = UMB_ARM_ENCODER;
    ASSERT_TRUE(shared_commit_settings(&first, revision, true));
    shared_control_mailbox_t box; shared_take_control_mailbox(&box);
    ASSERT_FALSE(box.cmd_pending);
}

TEST(test_maintenance_blocks_all_writers_until_neutral_ack) {
    seed(100);
    uint32_t id, other;
    ASSERT_TRUE(shared_begin_maintenance(&id));
    ASSERT_FALSE(shared_begin_maintenance(&other));
    ASSERT_FALSE(shared_post_arm_request(SHARED_ARM_CAR));
    ASSERT_FALSE(shared_settings_write_allowed());
    umb_cmd_t cmd = {UMB_CMD_RAW, UMB_SRC_CONSOLE, UMB_DRIVE_MANUAL, 0, 0.5f, now};
    ASSERT_FALSE(shared_post_cmd(&cmd));
    umb_settings_t cfg;
    uint32_t revision = shared_settings_snapshot(&cfg);
    ASSERT_FALSE(shared_commit_settings(&cfg, revision, false));
    ASSERT_FALSE(shared_maintenance_acknowledged(id));
    shared_ack_maintenance(id + 1);
    ASSERT_FALSE(shared_maintenance_acknowledged(id));
    shared_ack_maintenance(id);
    ASSERT_TRUE(shared_maintenance_acknowledged(id));
    shared_end_maintenance(id + 1);
    ASSERT_TRUE(shared_maintenance_acknowledged(id));
    shared_end_maintenance(id);
    ASSERT_FALSE(shared_maintenance_acknowledged(id));
    ASSERT_TRUE(shared_post_cmd(&cmd));
    ASSERT_TRUE(shared_begin_maintenance(&other));
    ASSERT_TRUE(id != other);
    shared_end_maintenance(id); /* stale end cannot release the new owner */
    ASSERT_FALSE(shared_post_cmd(&cmd));
    shared_end_maintenance(other);
}

TEST(test_inflight_arming_excludes_settings_and_maintenance) {
    seed(100); shared_post_arm_request(SHARED_ARM_CAR);
    shared_arm_request_t r;
    ASSERT_TRUE(shared_take_arm_record(&r, now));
    uint32_t token;
    ASSERT_FALSE(shared_begin_maintenance(&token));
    ASSERT_FALSE(shared_settings_write_allowed());
    umb_outputs_t out = {0}; shared_publish_outputs(&out);
    ASSERT_TRUE(shared_begin_maintenance(&token));
    shared_end_maintenance(token);
}

TEST(test_arm_deadline_wrap_and_none_mode_maintenance) {
    seed(UINT32_MAX - 20u);
    uint32_t id;
    ASSERT_TRUE(shared_submit_arm(SHARED_ARM_CAR, 0, now, &id));
    shared_arm_request_t request;
    ASSERT_TRUE(shared_take_arm_record(&request, 20u));
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    umb_outputs_t out = {0}; umb_arm_result_t result = UMB_ARM_WAITING_ROS;
    ASSERT_FALSE(shared_commit_arm_batch(&request, &result, 1, &out, &a, 39u));
    umb_settings_t cfg = umb_settings_defaults(); cfg.arming_mode = UMB_ARM_NONE;
    shared_init(&cfg);
    ASSERT_FALSE(shared_begin_maintenance(&id));
    ASSERT_FALSE(shared_settings_write_allowed());
}

int main(void) {
    RUN_TEST(test_arm_deadline_wrap_and_none_mode_maintenance);
    RUN_TEST(test_loss_is_sticky_and_old_session_is_rejected);
    RUN_TEST(test_boot_offline_does_not_fabricate_loss);
    RUN_TEST(test_cancelled_and_expired_arm_candidates_cannot_commit);
    RUN_TEST(test_arm_ack_is_core_decision_and_old_generation_refused);
    RUN_TEST(test_settings_revision_prevents_stale_writer_and_mode_exception_is_narrow);
    RUN_TEST(test_maintenance_blocks_all_writers_until_neutral_ack);
    RUN_TEST(test_inflight_arming_excludes_settings_and_maintenance);
    RUN_TEST(test_disarm_bypasses_full_arm_queue);
    RUN_TEST(test_snapshot_stalled_producers_expire);
    RUN_TEST(test_snapshot_oor_still_online_and_offline_not_counted);
    RUN_TEST(test_snapshot_expiry_wrap_safe);
    TEST_SUMMARY();
}
