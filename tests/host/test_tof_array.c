#include "test_runner.h"
#include "umb_tof_array.h"
#include <stdint.h>

static umb_tof_array_t all_online(uint32_t now) {
    umb_tof_array_t a = umb_tof_init();
    for (int i = 0; i < UMB_TOF_COUNT; i++) a = umb_tof_on_init_result(a, i, true, now);
    return a;
}
static umb_tof_array_t poll_all(umb_tof_array_t a, uint16_t mm, uint32_t now) {
    for (int i = 0; i < UMB_TOF_COUNT; i++) a = umb_tof_on_poll(a, i, 1, mm, now);
    return a;
}

TEST(test_tof_addr_plan) {
    ASSERT_EQ(umb_tof_addr(0), 0x30);
    ASSERT_EQ(umb_tof_addr(5), 0x35);
}

TEST(test_tof_init_all_offline_invalid) {
    umb_tof_array_t a = umb_tof_init();
    int16_t d[6];
    umb_tof_distances(&a, 0, d);
    for (int i = 0; i < 6; i++) ASSERT_EQ(d[i], 9999);
    ASSERT_EQ(umb_tof_online(&a, 0), 0);
    ASSERT_FALSE(umb_tof_recovering(&a, 0));
}

TEST(test_tof_array_one_missing_rest_online) {
    umb_tof_array_t a = umb_tof_init();
    for (int i = 0; i < 5; i++) a = umb_tof_on_init_result(a, i, true, 100);
    a = umb_tof_on_init_result(a, 5, false, 100);
    a = poll_all(a, 300, 150);
    int16_t d[6];
    umb_tof_distances(&a, 200, d);
    ASSERT_EQ(umb_tof_online(&a, 200), 5);
    ASSERT_EQ(d[0], 300);
    ASSERT_EQ(d[5], 9999);
}

TEST(test_tof_out_of_range_reports_9999) {
    umb_tof_array_t a = all_online(0);
    a = umb_tof_on_poll(a, 0, 1, 8190, 100);
    a = umb_tof_on_poll(a, 1, 1, 0, 100);
    a = umb_tof_on_poll(a, 2, 1, 500, 100);
    int16_t d[6];
    umb_tof_distances(&a, 100, d);
    ASSERT_EQ(d[0], 9999);
    ASSERT_EQ(d[1], 9999);
    ASSERT_EQ(d[2], 500);
    ASSERT_EQ(a.s[0].errors, 0);
    ASSERT_EQ(a.s[0].last_ok_ms, 100);
    ASSERT_EQ(umb_tof_online(&a, 100), 6);
}

TEST(test_tof_no_reading_yet_is_invalid) {
    umb_tof_array_t a = all_online(0);
    int16_t d[6];
    umb_tof_distances(&a, 10, d);
    ASSERT_EQ(d[0], 9999);
    ASSERT_EQ(umb_tof_online(&a, 10), 6);
}

TEST(test_tof_not_ready_poll_changes_nothing) {
    umb_tof_array_t a = all_online(0);
    a = umb_tof_on_poll(a, 0, 0, 123, 500);
    ASSERT_EQ(a.s[0].last_ok_ms, 0);
    ASSERT_FALSE(a.s[0].have_reading);
    ASSERT_EQ(a.s[0].errors, 0);
}

TEST(test_tof_array_stale_then_restart_cooldown) {
    umb_tof_array_t a = all_online(0);
    a = poll_all(a, 400, 0);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 1500), UMB_TOF_ACT_NONE);   /* exactly 1500 not stale */
    ASSERT_EQ(umb_tof_online(&a, 1500), 6);
    /* stale at 1600, but the init at t=0 is within cooldown until 3000 */
    ASSERT_EQ(umb_tof_online(&a, 1600), 0);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 1600), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 3000), UMB_TOF_ACT_RESTART);
    int16_t d[6];
    umb_tof_distances(&a, 1600, d);
    ASSERT_EQ(d[0], 9999);
    /* failed restart at t=3000 -> cooldown again */
    a = umb_tof_on_init_result(a, 0, false, 3000);
    ASSERT_EQ(a.s[0].link, UMB_TOF_OFFLINE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 3001), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 5999), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 6000), UMB_TOF_ACT_RESTART);
}

TEST(test_tof_stale_with_old_restart_restarts_immediately) {
    umb_tof_array_t a = all_online(0);
    a = umb_tof_on_poll(a, 0, 1, 400, 10000);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 10000), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 11600), UMB_TOF_ACT_RESTART);
}

TEST(test_tof_first_restart_immediate) {
    umb_tof_array_t a = umb_tof_init();
    ASSERT_EQ(umb_tof_next_action(&a, 0, 0), UMB_TOF_ACT_RESTART);
    ASSERT_EQ(umb_tof_next_action(&a, 3, 7), UMB_TOF_ACT_RESTART);
    ASSERT_EQ(umb_tof_next_action(&a, 3, 0xFFFFFFF0u), UMB_TOF_ACT_RESTART);
    a = umb_tof_on_init_result(a, 3, false, 7);
    ASSERT_EQ(umb_tof_next_action(&a, 3, 8), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 2, 8), UMB_TOF_ACT_RESTART);
}

TEST(test_tof_online_slot_no_action) {
    umb_tof_array_t a = all_online(0);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 100), UMB_TOF_ACT_NONE);
}

TEST(test_tof_init_ok_gives_fresh_grace) {
    umb_tof_array_t a = umb_tof_init();
    a = umb_tof_on_init_result(a, 0, true, 5000);
    ASSERT_EQ(a.s[0].last_ok_ms, 5000);
    ASSERT_EQ(a.s[0].last_restart_ms, 5000);
    ASSERT_EQ(a.s[0].restarts, 1);
    ASSERT_FALSE(a.s[0].have_reading);
    ASSERT_EQ(umb_tof_online(&a, 6500), 1);
    ASSERT_EQ(umb_tof_online(&a, 6501), 0);
}

TEST(test_tof_boot_init_not_recovering) {
    umb_tof_array_t a = all_online(100);
    ASSERT_FALSE(umb_tof_recovering(&a, 100));
    ASSERT_FALSE(umb_tof_recovering(&a, 500));
    umb_tof_array_t f = umb_tof_init();
    f = umb_tof_on_init_result(f, 4, false, 200);     /* boot failure also not recovering */
    ASSERT_FALSE(umb_tof_recovering(&f, 300));
    f = umb_tof_on_init_result(f, 4, true, 3300);     /* its retry is a recovery */
    ASSERT_TRUE(umb_tof_recovering(&f, 3400));
}

TEST(test_tof_recovering_window) {
    umb_tof_array_t a = all_online(0);
    ASSERT_FALSE(umb_tof_recovering(&a, 4000));
    a = umb_tof_on_init_result(a, 2, true, 5000);     /* a real restart */
    ASSERT_EQ(a.s[2].restarts, 2);
    ASSERT_TRUE(umb_tof_recovering(&a, 5000));
    ASSERT_TRUE(umb_tof_recovering(&a, 7999));
    ASSERT_FALSE(umb_tof_recovering(&a, 8000));
}

TEST(test_tof_error_counts) {
    umb_tof_array_t a = all_online(0);
    a = umb_tof_on_poll(a, 1, 1, 250, 100);
    a = umb_tof_on_poll(a, 1, -1, 0, 200);
    a = umb_tof_on_poll(a, 1, -5, 0, 300);
    ASSERT_EQ(a.s[1].errors, 2);
    ASSERT_EQ(a.s[1].last_ok_ms, 100);
    ASSERT_EQ(a.s[1].link, UMB_TOF_ONLINE);
    ASSERT_EQ(a.s[1].mm, 250);
}

TEST(test_tof_consecutive_errors_go_offline) {
    umb_tof_array_t a = all_online(0);
    for (int k = 0; k < UMB_TOF_MAX_CONSEC_ERRORS - 1; k++)
        a = umb_tof_on_poll(a, 0, -1, 0, 10 + k);
    ASSERT_EQ(a.s[0].link, UMB_TOF_ONLINE);
    a = umb_tof_on_poll(a, 0, 1, 300, 50);            /* success resets the streak */
    for (int k = 0; k < UMB_TOF_MAX_CONSEC_ERRORS - 1; k++)
        a = umb_tof_on_poll(a, 0, -1, 0, 60 + k);
    ASSERT_EQ(a.s[0].link, UMB_TOF_ONLINE);
    a = umb_tof_on_poll(a, 0, -1, 0, 100);
    ASSERT_EQ(a.s[0].link, UMB_TOF_OFFLINE);
    ASSERT_EQ(umb_tof_online(&a, 100), 5);
    int16_t d[6];
    umb_tof_distances(&a, 100, d);
    ASSERT_EQ(d[0], 9999);
    ASSERT_EQ(umb_tof_next_action(&a, 0, 100), UMB_TOF_ACT_NONE);     /* cooldown from init at 0 */
    ASSERT_EQ(umb_tof_next_action(&a, 0, 3000), UMB_TOF_ACT_RESTART);
    /* restart clears the streak */
    a = umb_tof_on_init_result(a, 0, true, 3000);
    a = umb_tof_on_poll(a, 0, -1, 0, 3010);
    ASSERT_EQ(a.s[0].link, UMB_TOF_ONLINE);
}

TEST(test_tof_poll_on_offline_slot_ignored) {
    umb_tof_array_t a = umb_tof_init();
    a = umb_tof_on_poll(a, 0, 1, 300, 10);
    ASSERT_EQ(a.s[0].link, UMB_TOF_OFFLINE);
    ASSERT_FALSE(a.s[0].have_reading);
}

TEST(test_tof_bad_index_and_null) {
    umb_tof_array_t a = umb_tof_init();
    umb_tof_array_t b = umb_tof_on_init_result(a, -1, true, 5);
    b = umb_tof_on_init_result(b, 6, true, 5);
    b = umb_tof_on_poll(b, 9, 1, 100, 5);
    ASSERT_EQ(umb_tof_online(&b, 5), 0);
    ASSERT_EQ(umb_tof_addr(-1), 0);
    ASSERT_EQ(umb_tof_addr(6), 0);
    ASSERT_EQ(umb_tof_next_action(&a, 6, 0), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_online(NULL, 0), 0);
    ASSERT_FALSE(umb_tof_recovering(NULL, 0));
    umb_tof_distances(NULL, 0, NULL);
}

TEST(test_tof_wrap_safe) {
    uint32_t t0 = 0xFFFFFF00u;                         /* 256 ms before wrap */
    umb_tof_array_t a = all_online(t0);
    a = poll_all(a, 700, t0 + 100);
    int16_t d[6];
    umb_tof_distances(&a, t0 + 1000, d);               /* wrapped past zero */
    ASSERT_EQ(d[0], 700);
    ASSERT_EQ(umb_tof_online(&a, t0 + 100 + 1500), 6);
    ASSERT_EQ(umb_tof_online(&a, t0 + 100 + 1501), 0);
    ASSERT_EQ(umb_tof_next_action(&a, 0, t0 + 2999), UMB_TOF_ACT_NONE);
    ASSERT_EQ(umb_tof_next_action(&a, 0, t0 + 3000), UMB_TOF_ACT_RESTART);
    a = umb_tof_on_init_result(a, 0, true, t0 + 3000);
    ASSERT_TRUE(umb_tof_recovering(&a, t0 + 5999));
    ASSERT_FALSE(umb_tof_recovering(&a, t0 + 6000));
}

int main(void) {
    RUN_TEST(test_tof_addr_plan);
    RUN_TEST(test_tof_init_all_offline_invalid);
    RUN_TEST(test_tof_array_one_missing_rest_online);
    RUN_TEST(test_tof_out_of_range_reports_9999);
    RUN_TEST(test_tof_no_reading_yet_is_invalid);
    RUN_TEST(test_tof_not_ready_poll_changes_nothing);
    RUN_TEST(test_tof_array_stale_then_restart_cooldown);
    RUN_TEST(test_tof_stale_with_old_restart_restarts_immediately);
    RUN_TEST(test_tof_first_restart_immediate);
    RUN_TEST(test_tof_online_slot_no_action);
    RUN_TEST(test_tof_init_ok_gives_fresh_grace);
    RUN_TEST(test_tof_boot_init_not_recovering);
    RUN_TEST(test_tof_recovering_window);
    RUN_TEST(test_tof_error_counts);
    RUN_TEST(test_tof_consecutive_errors_go_offline);
    RUN_TEST(test_tof_poll_on_offline_slot_ignored);
    RUN_TEST(test_tof_bad_index_and_null);
    RUN_TEST(test_tof_wrap_safe);
    TEST_SUMMARY();
}
