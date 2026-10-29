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

int main(void) {
    RUN_TEST(test_snapshot_stalled_producers_expire);
    RUN_TEST(test_snapshot_oor_still_online_and_offline_not_counted);
    RUN_TEST(test_snapshot_expiry_wrap_safe);
    TEST_SUMMARY();
}
