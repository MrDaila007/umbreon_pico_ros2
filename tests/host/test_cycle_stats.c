/* Control-cycle lateness accounting: flash-induced pauses are counted apart from
 * ordinary scheduling jitter. */
#include "test_runner.h"
#include "umb_cycle_stats.h"

TEST(test_on_time_cycles_count_nothing_late) {
    umb_cycle_stats_t s = {0};
    s = umb_cycle_stats_record(s, 0, false);
    s = umb_cycle_stats_record(s, UMB_CYCLE_JITTER_TOL_MS, false);
    s = umb_cycle_stats_record(s, UMB_CYCLE_JITTER_TOL_MS, true);
    ASSERT_EQ(s.cycles, 3);
    ASSERT_EQ(s.jitter_late, 0);
    ASSERT_EQ(s.flash_pauses, 0);
}

TEST(test_late_cycles_split_by_maintenance) {
    umb_cycle_stats_t s = {0};
    s = umb_cycle_stats_record(s, 9, false);
    s = umb_cycle_stats_record(s, 4, false);   /* within tolerance */
    s = umb_cycle_stats_record(s, 6, false);
    ASSERT_EQ(s.jitter_late, 2);
    ASSERT_EQ(s.jitter_max_ms, 9);
    ASSERT_EQ(s.flash_pauses, 0);
    s = umb_cycle_stats_record(s, 31, true);
    s = umb_cycle_stats_record(s, 12, true);   /* catch-up after the stall: same event */
    ASSERT_EQ(s.flash_pauses, 1);
    s = umb_cycle_stats_record(s, 0, true);
    s = umb_cycle_stats_record(s, 12, true);   /* a new stall */
    ASSERT_EQ(s.flash_pauses, 2);
    ASSERT_EQ(s.flash_pause_max_ms, 31);
    ASSERT_EQ(s.jitter_late, 2);               /* unchanged */
    ASSERT_EQ(s.jitter_max_ms, 9);
    ASSERT_EQ(s.cycles, 7);
}

TEST(test_negative_lateness_is_not_late_and_counters_saturate) {
    umb_cycle_stats_t s = {0};
    s = umb_cycle_stats_record(s, -3, false);
    ASSERT_EQ(s.jitter_late, 0);
    s.cycles = 0xFFFFFFFFu; s.flash_pauses = 0xFFFFFFFFu;
    s = umb_cycle_stats_record(s, 50, true);
    ASSERT_EQ(s.cycles, 0xFFFFFFFFu);
    ASSERT_EQ(s.flash_pauses, 0xFFFFFFFFu);
}

int main(void) {
    RUN_TEST(test_on_time_cycles_count_nothing_late);
    RUN_TEST(test_late_cycles_split_by_maintenance);
    RUN_TEST(test_negative_lateness_is_not_late_and_counters_saturate);
    TEST_SUMMARY();
}
