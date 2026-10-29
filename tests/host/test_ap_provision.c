#include "test_runner.h"
#include "umb_ap_provision.h"
TEST(test_provisioned_device_never_due) {
    umb_ap_provision_t s = umb_ap_provision_init();
    ASSERT_FALSE(umb_ap_provision_due(s, true, 0));
    ASSERT_TRUE(umb_ap_provision_due(s, false, 0));
    s = umb_ap_provision_result(s, UMB_PROV_RESULT_OK, 5);
    ASSERT_EQ(s.stage, UMB_PROV_DONE); ASSERT_EQ(s.writes, 1); ASSERT_EQ(s.failures, 0);
    ASSERT_FALSE(umb_ap_provision_due(s, true, 10));
    /* Even if RAM loses the password later, a provisioned boot never regenerates. */
    ASSERT_FALSE(umb_ap_provision_due(s, false, 10));
    umb_ap_provision_t a = umb_ap_provision_result(umb_ap_provision_init(), UMB_PROV_RESULT_ALREADY, 0);
    ASSERT_FALSE(umb_ap_provision_due(a, false, 100000));
}
TEST(test_busy_retries_soon_without_counting_a_write) {
    umb_ap_provision_t s = umb_ap_provision_init();
    s = umb_ap_provision_result(s, UMB_PROV_RESULT_BUSY, 100);
    ASSERT_EQ(s.stage, UMB_PROV_BACKOFF); ASSERT_EQ(s.writes, 0); ASSERT_EQ(s.failures, 1);
    ASSERT_FALSE(umb_ap_provision_due(s, false, 100 + UMB_PROV_BUSY_RETRY_MS - 1));
    ASSERT_TRUE(umb_ap_provision_due(s, false, 100 + UMB_PROV_BUSY_RETRY_MS));
    for (int i = 0; i < 100; i++) s = umb_ap_provision_result(s, UMB_PROV_RESULT_BUSY, 0);
    ASSERT_EQ(s.stage, UMB_PROV_BACKOFF); ASSERT_EQ(s.writes, 0);
}
TEST(test_flash_failure_backs_off_and_gives_up_without_write_loop) {
    umb_ap_provision_t s = umb_ap_provision_init();
    uint32_t now = 0, expected = UMB_PROV_FIRST_BACKOFF_MS;
    unsigned writes = 0;
    for (uint32_t t = 0; t < 3600000u; t += 10) {
        if (!umb_ap_provision_due(s, false, t)) continue;
        writes++; now = t;
        s = umb_ap_provision_result(s, UMB_PROV_RESULT_FLASH_FAILED, t);
        if (s.stage == UMB_PROV_BACKOFF) {
            ASSERT_EQ(s.retry_ms - now, expected);
            expected *= 2; if (expected > UMB_PROV_MAX_BACKOFF_MS) expected = UMB_PROV_MAX_BACKOFF_MS;
        }
    }
    ASSERT_EQ(writes, UMB_PROV_MAX_WRITES);
    ASSERT_EQ(s.stage, UMB_PROV_GAVE_UP); ASSERT_EQ(s.last, UMB_PROV_RESULT_FLASH_FAILED);
    ASSERT_FALSE(umb_ap_provision_due(s, false, 0xffffffffu));
}
TEST(test_entropy_failure_and_wrapping_deadline) {
    umb_ap_provision_t s = umb_ap_provision_init();
    s = umb_ap_provision_result(s, UMB_PROV_RESULT_NO_ENTROPY, 0xfffffff0u);
    ASSERT_EQ(s.writes, 0); ASSERT_EQ(s.stage, UMB_PROV_BACKOFF);
    ASSERT_FALSE(umb_ap_provision_due(s, false, 5));
    ASSERT_TRUE(umb_ap_provision_due(s, false, 0xfffffff0u + UMB_PROV_FIRST_BACKOFF_MS));
    s = umb_ap_provision_result(s, UMB_PROV_RESULT_ALREADY, 7);
    ASSERT_EQ(s.stage, UMB_PROV_DONE); ASSERT_EQ(s.writes, 0);
    ASSERT_EQ(s.backoff_ms, UMB_PROV_FIRST_BACKOFF_MS);
    s = umb_ap_provision_result(s, (umb_prov_result_t)99, 8);
    ASSERT_EQ(s.stage, UMB_PROV_DONE);
}
int main(void) {
    RUN_TEST(test_provisioned_device_never_due);
    RUN_TEST(test_busy_retries_soon_without_counting_a_write);
    RUN_TEST(test_flash_failure_backs_off_and_gives_up_without_write_loop);
    RUN_TEST(test_entropy_failure_and_wrapping_deadline);
    TEST_SUMMARY();
}
