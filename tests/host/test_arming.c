#include "test_runner.h"
#include "umb_arming.h"
#include <math.h>
#include <stddef.h>

static const umb_arm_mode_t MODES[] = { UMB_ARM_NONE, UMB_ARM_ENCODER, UMB_ARM_ROS, UMB_ARM_BOTH };

static umb_cmd_t cmd_with(umb_cmd_kind_t kind, float value) {
    umb_cmd_t c = { kind, UMB_SRC_ROS, UMB_DRIVE_MANUAL, 0.0f, value, 0 };
    return c;
}

static umb_arming_t neutral(umb_arming_t a) {
    umb_cmd_t c = cmd_with(UMB_CMD_RAW, 0.0f);
    return umb_arming_on_command(a, &c);
}

TEST(test_arming_boot_disarmed_all_modes) {
    for (size_t i = 0; i < sizeof MODES / sizeof MODES[0]; i++) {
        umb_arming_t a = umb_arming_init(MODES[i]);
        if (MODES[i] != UMB_ARM_NONE) ASSERT_FALSE(umb_arming_is_armed(a));
        ASSERT_FALSE(umb_arming_motion_allowed(a));
    }
}

TEST(test_arming_none_needs_neutral) {
    umb_arming_t a = umb_arming_init(UMB_ARM_NONE);
    umb_cmd_t c = cmd_with(UMB_CMD_RAW, 0.5f);
    a = umb_arming_on_command(a, &c);
    ASSERT_FALSE(umb_arming_motion_allowed(a));
    c = cmd_with(UMB_CMD_RAW, 0.0f);
    a = umb_arming_on_command(a, &c);
    ASSERT_TRUE(umb_arming_motion_allowed(a));
}

TEST(test_arming_both_two_keys_any_order) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_WAITING_CAR);
    ASSERT_FALSE(umb_arming_is_armed(a));
    a = umb_arming_car_key(a, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));

    a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_car_key(a, &r);
    ASSERT_EQ(r, UMB_ARM_WAITING_ROS);
    ASSERT_FALSE(umb_arming_is_armed(a));
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));
}

TEST(test_arming_both_needs_neutral_after_keys) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_ros(a, true, &r);
    a = umb_arming_car_key(a, &r);
    ASSERT_TRUE(umb_arming_is_armed(a));
    ASSERT_FALSE(umb_arming_motion_allowed(a));
    a = neutral(a);
    ASSERT_TRUE(umb_arming_motion_allowed(a));
}

TEST(test_arming_disarm_clears_both) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_ros(a, true, &r);
    a = umb_arming_car_key(a, &r);
    a = neutral(a);
    ASSERT_TRUE(umb_arming_motion_allowed(a));
    a = umb_arming_disarm(a);
    ASSERT_FALSE(a.car_key);
    ASSERT_FALSE(a.ros_key);
    ASSERT_FALSE(a.neutral_seen);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_WAITING_CAR);
}

TEST(test_arming_ros_false_always_disarms) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_ros(a, true, &r);
    a = umb_arming_car_key(a, &r);
    a = neutral(a);
    a = umb_arming_ros(a, false, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_FALSE(umb_arming_is_armed(a));
    ASSERT_FALSE(a.neutral_seen);
    /* also in ENCODER mode */
    a = umb_arming_init(UMB_ARM_ENCODER);
    a = umb_arming_car_key(a, &r);
    a = umb_arming_ros(a, false, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_FALSE(umb_arming_is_armed(a));
}

TEST(test_arming_encoder_mode_rejects_ros_arm) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_ENCODER);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_REJECTED_MODE);
    ASSERT_FALSE(a.ros_key);
    a = umb_arming_car_key(a, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));
}

TEST(test_arming_ros_mode_rejects_car_key) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_ROS);
    a = umb_arming_car_key(a, &r);
    ASSERT_EQ(r, UMB_ARM_REJECTED_MODE);
    ASSERT_FALSE(a.car_key);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));
}

TEST(test_arming_mode_change_disarms) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_ros(a, true, &r);
    a = umb_arming_car_key(a, &r);
    ASSERT_TRUE(umb_arming_is_armed(a));
    a = umb_arming_set_mode(a, UMB_ARM_BOTH);
    ASSERT_FALSE(umb_arming_is_armed(a));
    a = umb_arming_set_mode(a, UMB_ARM_ENCODER);
    ASSERT_EQ(a.mode, UMB_ARM_ENCODER);
}

TEST(test_arming_nan_command_not_neutral) {
    umb_arming_t a = umb_arming_init(UMB_ARM_NONE);
    umb_cmd_t c = cmd_with(UMB_CMD_RAW, NAN);
    a = umb_arming_on_command(a, &c);
    ASSERT_FALSE(a.neutral_seen);
    c = cmd_with(UMB_CMD_SPEED, INFINITY);
    a = umb_arming_on_command(a, &c);
    ASSERT_FALSE(a.neutral_seen);
}

/* ruling 1 */
TEST(test_arming_neutral_before_keys_not_enough) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = neutral(a);
    a = umb_arming_ros(a, true, &r);
    a = umb_arming_car_key(a, &r);
    ASSERT_TRUE(umb_arming_is_armed(a));
    ASSERT_FALSE(umb_arming_motion_allowed(a));
    a = neutral(a);
    ASSERT_TRUE(umb_arming_motion_allowed(a));
    /* a repeated accepted key press re-latches */
    a = umb_arming_car_key(a, &r);
    ASSERT_FALSE(umb_arming_motion_allowed(a));
    a = neutral(a);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_FALSE(umb_arming_motion_allowed(a));
}

/* ruling 2 */
TEST(test_arming_none_rejects_keys) {
    umb_arm_result_t r;
    umb_arming_t a = neutral(umb_arming_init(UMB_ARM_NONE));
    ASSERT_TRUE(a.neutral_seen);
    a = umb_arming_car_key(a, &r);
    ASSERT_EQ(r, UMB_ARM_REJECTED_MODE);
    ASSERT_TRUE(a.neutral_seen);
    ASSERT_FALSE(a.car_key);
    a = umb_arming_ros(a, true, &r);
    ASSERT_EQ(r, UMB_ARM_REJECTED_MODE);
    ASSERT_TRUE(a.neutral_seen);
    ASSERT_FALSE(a.ros_key);
    a = umb_arming_ros(a, false, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_FALSE(a.neutral_seen);
    ASSERT_FALSE(umb_arming_motion_allowed(a));
}

/* ruling 3 */
TEST(test_arming_single_key_modes_ok) {
    umb_arm_result_t r;
    umb_arming_t a = umb_arming_car_key(umb_arming_init(UMB_ARM_ENCODER), &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));
    a = umb_arming_ros(umb_arming_init(UMB_ARM_ROS), true, &r);
    ASSERT_EQ(r, UMB_ARM_OK);
    ASSERT_TRUE(umb_arming_is_armed(a));
}

/* ruling 4 */
TEST(test_arming_null_res_tolerated) {
    umb_arming_t a = umb_arming_init(UMB_ARM_BOTH);
    a = umb_arming_car_key(a, NULL);
    ASSERT_TRUE(a.car_key);
    a = umb_arming_ros(a, true, NULL);
    ASSERT_TRUE(umb_arming_is_armed(a));
    a = umb_arming_ros(a, false, NULL);
    ASSERT_FALSE(umb_arming_is_armed(a));
    a = umb_arming_car_key(umb_arming_init(UMB_ARM_ROS), NULL);
    ASSERT_FALSE(a.car_key);
}

TEST(test_arming_kind_none_and_deadband_edges) {
    umb_arming_t a = umb_arming_init(UMB_ARM_NONE);
    umb_cmd_t c = cmd_with(UMB_CMD_NONE, 0.0f);
    a = umb_arming_on_command(a, &c);
    ASSERT_FALSE(a.neutral_seen);
    a = umb_arming_on_command(a, NULL);
    ASSERT_FALSE(a.neutral_seen);
    c = cmd_with(UMB_CMD_SPEED, 0.05f);
    a = umb_arming_on_command(a, &c);
    ASSERT_FALSE(a.neutral_seen);
    c = cmd_with(UMB_CMD_SPEED, -0.04f);
    a = umb_arming_on_command(a, &c);
    ASSERT_TRUE(a.neutral_seen);
    /* non-neutral command does not clear an already seen neutral */
    c = cmd_with(UMB_CMD_SPEED, 0.5f);
    a = umb_arming_on_command(a, &c);
    ASSERT_TRUE(a.neutral_seen);
}

int main(void) {
    RUN_TEST(test_arming_boot_disarmed_all_modes);
    RUN_TEST(test_arming_none_needs_neutral);
    RUN_TEST(test_arming_both_two_keys_any_order);
    RUN_TEST(test_arming_both_needs_neutral_after_keys);
    RUN_TEST(test_arming_disarm_clears_both);
    RUN_TEST(test_arming_ros_false_always_disarms);
    RUN_TEST(test_arming_encoder_mode_rejects_ros_arm);
    RUN_TEST(test_arming_ros_mode_rejects_car_key);
    RUN_TEST(test_arming_mode_change_disarms);
    RUN_TEST(test_arming_nan_command_not_neutral);
    RUN_TEST(test_arming_neutral_before_keys_not_enough);
    RUN_TEST(test_arming_none_rejects_keys);
    RUN_TEST(test_arming_single_key_modes_ok);
    RUN_TEST(test_arming_null_res_tolerated);
    RUN_TEST(test_arming_kind_none_and_deadband_edges);
    TEST_SUMMARY();
}
