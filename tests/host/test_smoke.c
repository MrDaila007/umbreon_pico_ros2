#include "test_runner.h"

TEST(test_harness_runs) {
    ASSERT_EQ(1 + 1, 2);
}

int main(void) {
    RUN_TEST(test_harness_runs);
    TEST_SUMMARY();
}
