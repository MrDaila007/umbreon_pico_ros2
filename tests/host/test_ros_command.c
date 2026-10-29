#include "test_runner.h"
#include "umb_ros_command.h"
#include <float.h>
#include <math.h>
#include <string.h>
TEST(test_ros_topic_mapping_signed_reverse_zero_and_steering) {
    for(unsigned topic=0;topic<3;topic++) {
        umb_twist_fields_t t={.linear={-0.75,0,0},.angular={0,0,0.5}}; umb_cmd_t out;
        ASSERT_TRUE(umb_ros_command((umb_ros_command_topic_t)topic,&t,UINT32_MAX,&out));
        ASSERT_EQ(out.kind,topic==2 ? UMB_CMD_RAW : UMB_CMD_SPEED);
        ASSERT_EQ(out.mode,topic==0 ? UMB_DRIVE_AUTO : UMB_DRIVE_MANUAL);
        ASSERT_EQ(out.source,UMB_SRC_ROS);ASSERT_EQ(out.rx_ms,UINT32_MAX);
        ASSERT_FLOAT_EQ(out.value,-0.75,0.00001);ASSERT_FLOAT_EQ(out.steer,0.5,0.00001);
        t.linear[0]=0;t.angular[2]=10;
        ASSERT_TRUE(umb_ros_command((umb_ros_command_topic_t)topic,&t,1,&out));
        ASSERT_EQ(out.value,0);ASSERT_EQ(out.steer,1);
        t.angular[2]=-10;ASSERT_TRUE(umb_ros_command((umb_ros_command_topic_t)topic,&t,2,&out));ASSERT_EQ(out.steer,-1);
    }
}
TEST(test_ros_invalid_fields_leave_output_untouched) {
    umb_cmd_t before;memset(&before,0x5a,sizeof before);umb_cmd_t out=before;
    const double invalid[]={NAN,INFINITY,-INFINITY};
    for(unsigned axis=0;axis<6;axis++) for(unsigned i=0;i<3;i++) {
        umb_twist_fields_t t={0};if(axis<3)t.linear[axis]=invalid[i];else t.angular[axis-3]=invalid[i];
        ASSERT_FALSE(umb_ros_command(UMB_ROS_CMD_VEL,&t,100,&out));ASSERT_EQ(memcmp(&out,&before,sizeof out),0);
    }
    for(unsigned axis=0;axis<4;axis++) {
        umb_twist_fields_t t={0};if(axis<2)t.linear[axis+1]=0.001;else t.angular[axis-2]=-0.001;
        ASSERT_FALSE(umb_ros_command(UMB_ROS_CMD_RAW,&t,101,&out));ASSERT_EQ(memcmp(&out,&before,sizeof out),0);
    }
    umb_twist_fields_t t={0};
    ASSERT_FALSE(umb_ros_command((umb_ros_command_topic_t)-1,&t,102,&out));
    ASSERT_FALSE(umb_ros_command((umb_ros_command_topic_t)3,&t,102,&out));
    ASSERT_FALSE(umb_ros_command(UMB_ROS_CMD_VEL,NULL,102,&out));
    ASSERT_FALSE(umb_ros_command(UMB_ROS_CMD_VEL,&t,102,NULL));ASSERT_EQ(memcmp(&out,&before,sizeof out),0);
}
TEST(test_ros_finite_double_overflow_cannot_create_float_infinity) {
    umb_twist_fields_t t={.linear={DBL_MAX,0,0},.angular={0,0,DBL_MAX}};umb_cmd_t out;
    ASSERT_TRUE(umb_ros_command(UMB_ROS_CMD_VEL,&t,0,&out));ASSERT_TRUE(isfinite(out.value));
    ASSERT_TRUE(out.value==FLT_MAX);ASSERT_EQ(out.steer,1);
    t.linear[0]=-DBL_MAX;t.angular[2]=-DBL_MAX;
    ASSERT_TRUE(umb_ros_command(UMB_ROS_CMD_RAW,&t,0,&out));ASSERT_TRUE(out.value==-FLT_MAX);ASSERT_EQ(out.steer,-1);
}
int main(void) {
    RUN_TEST(test_ros_topic_mapping_signed_reverse_zero_and_steering);
    RUN_TEST(test_ros_invalid_fields_leave_output_untouched);
    RUN_TEST(test_ros_finite_double_overflow_cannot_create_float_infinity);
    TEST_SUMMARY();
}
