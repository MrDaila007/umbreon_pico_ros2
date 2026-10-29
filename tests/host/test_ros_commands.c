#include "test_runner.h"
#include "ros_commands.h"
#include "shared_state.h"
#include <math.h>
static uint32_t now;
uint32_t hal_now_ms(void) { return now; }
static uint32_t ready(void) {
    now=10;umb_settings_t settings=umb_settings_defaults();shared_init(&settings);
    shared_set_link(true,true);return shared_session_generation();
}
TEST(test_ros_rejected_command_does_not_refresh_mailbox) {
    uint32_t gen=ready();umb_twist_fields_t t={.linear={0.5,0,0}};
    ASSERT_TRUE(ros_commands_post(UMB_ROS_CMD_VEL,&t,gen));now=50;t.linear[0]=NAN;
    ASSERT_FALSE(ros_commands_post(UMB_ROS_CMD_VEL,&t,gen));
    shared_control_mailbox_t box;shared_take_control_mailbox(&box);
    ASSERT_TRUE(box.cmd_pending);ASSERT_EQ(box.cmd.rx_ms,10);ASSERT_FLOAT_EQ(box.cmd.value,0.5,0.00001);
    shared_take_control_mailbox(&box);ASSERT_FALSE(box.cmd_pending);
}
TEST(test_ros_session_generation_and_maintenance_reject) {
    uint32_t gen=ready();umb_twist_fields_t t={0};
    shared_notify_network_loss(UMB_STOP_AGENT);shared_set_link(true,true);
    ASSERT_FALSE(ros_commands_post(UMB_ROS_CMD_VEL,&t,gen));
    /* Loss creates a sticky priority disarm; model its real control completion
     * before accepting commands for the replacement ready session. */
    ASSERT_FALSE(ros_commands_post(UMB_ROS_CMD_VEL,&t,shared_session_generation()));
    shared_arm_request_t request;ASSERT_TRUE(shared_take_arm_record(&request,now));
    shared_control_mailbox_t box;shared_take_control_mailbox(&box);
    umb_outputs_t out={0};umb_arming_t arming=umb_arming_init(UMB_ARM_BOTH);
    umb_arm_result_t result=UMB_ARM_OK;
    ASSERT_TRUE(shared_commit_arm_batch(&request,&result,1,&out,&arming,now));
    shared_publish_outputs(&out);shared_publish_arming(&arming);
    ASSERT_TRUE(ros_commands_post(UMB_ROS_CMD_VEL,&t,shared_session_generation()));
    uint32_t token;ASSERT_TRUE(shared_begin_maintenance(&token));
    ASSERT_FALSE(ros_commands_post(UMB_ROS_CMD_VEL,&t,shared_session_generation()));
    shared_end_maintenance(token);shared_set_link(false,false);
    ASSERT_FALSE(ros_commands_post(UMB_ROS_CMD_VEL,&t,shared_session_generation()));
}
TEST(test_ros_latest_callback_and_console_share_explicit_mailbox) {
    uint32_t gen=ready();umb_twist_fields_t t={.linear={0.5,0,0}};
    ASSERT_TRUE(ros_commands_post(UMB_ROS_CMD_VEL,&t,gen));now=20;t.linear[0]=-0.2;
    ASSERT_TRUE(ros_commands_post(UMB_ROS_CMD_RAW,&t,gen));
    shared_control_mailbox_t box;shared_take_control_mailbox(&box);
    ASSERT_EQ(box.cmd.kind,UMB_CMD_RAW);ASSERT_EQ(box.cmd.rx_ms,20);ASSERT_FLOAT_EQ(box.cmd.value,-0.2,0.00001);
    umb_cmd_t local={.kind=UMB_CMD_SPEED,.source=UMB_SRC_CONSOLE,.value=0,.rx_ms=21};
    ASSERT_TRUE(shared_post_cmd(&local));shared_take_control_mailbox(&box);ASSERT_EQ(box.cmd.source,UMB_SRC_CONSOLE);
    now=22;t.linear[0]=0;ASSERT_TRUE(ros_commands_post(UMB_ROS_CMD_TELEOP,&t,gen));
    shared_take_control_mailbox(&box);ASSERT_EQ(box.cmd.source,UMB_SRC_ROS);ASSERT_EQ(box.cmd.mode,UMB_DRIVE_MANUAL);
}
int main(void) {
    RUN_TEST(test_ros_rejected_command_does_not_refresh_mailbox);
    RUN_TEST(test_ros_session_generation_and_maintenance_reject);
    RUN_TEST(test_ros_latest_callback_and_console_share_explicit_mailbox);
    TEST_SUMMARY();
}
