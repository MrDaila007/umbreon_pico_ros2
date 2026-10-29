#include "test_runner.h"
#include "hal_microros_platform.h"
#include "hal_time.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"
#include "rcutils/allocator.h"
static rcutils_allocator_t allocator;
static bool fail_install, fail_allocate;
static unsigned alloc_calls, install_calls;
static uint64_t now_us, waited_us;
static TickType_t delayed;
static int scheduler;
void *pvPortMalloc(size_t size) { alloc_calls++; return fail_allocate ? NULL : malloc(size); }
void vPortFree(void *ptr) { free(ptr); }
bool rcutils_set_default_allocator(rcutils_allocator_t *a) {
    install_calls++; if (fail_install) return false; allocator = *a; return true;
}
uint64_t hal_now_us64(void) { return now_us; }
int xTaskGetSchedulerState(void) { return scheduler; }
void vTaskDelay(TickType_t ticks) { delayed = ticks; }
void busy_wait_us(uint64_t us) { waited_us = us; }
TEST(test_platform_install_failure_and_once_only_success) {
    fail_install = true; ASSERT_FALSE(hal_microros_platform_init());
    fail_install = false; ASSERT_TRUE(hal_microros_platform_init());
    ASSERT_TRUE(hal_microros_platform_init()); ASSERT_EQ(install_calls, 2);
}
TEST(test_allocator_alignment_resize_contents_and_stats) {
    uint8_t *p = allocator.allocate(17, NULL); ASSERT_TRUE(p != NULL);
    ASSERT_EQ((uintptr_t)p % _Alignof(max_align_t), 0);
    memset(p, 0x42, 17);
    p = allocator.reallocate(p, 32, NULL); ASSERT_TRUE(p != NULL);
    for (int i = 0; i < 17; i++) ASSERT_EQ(p[i], 0x42);
    p = allocator.reallocate(p, 8, NULL); ASSERT_TRUE(p != NULL);
    for (int i = 0; i < 8; i++) ASSERT_EQ(p[i], 0x42);
    ASSERT_EQ(hal_microros_allocator_stats().live_blocks, 1);
    ASSERT_EQ(hal_microros_allocator_stats().live_bytes, 8);
    ASSERT_TRUE(hal_microros_allocator_stats().peak_bytes >= 49);
    ASSERT_TRUE(allocator.reallocate(p, 0, NULL) == NULL);
    ASSERT_EQ(hal_microros_allocator_stats().live_blocks, 0);
}
TEST(test_allocator_failed_realloc_preserves_old_and_overflow_does_not_allocate) {
    uint8_t *p = allocator.allocate(4, NULL); ASSERT_TRUE(p != NULL); memset(p, 0x33, 4);
    fail_allocate = true;
    ASSERT_TRUE(allocator.reallocate(p, 128, NULL) == NULL);
    for (int i = 0; i < 4; i++) ASSERT_EQ(p[i], 0x33);
    ASSERT_EQ(hal_microros_allocator_stats().live_bytes, 4);
    fail_allocate = false; allocator.deallocate(p, NULL);
    unsigned before = alloc_calls;
    ASSERT_TRUE(allocator.allocate(SIZE_MAX, NULL) == NULL);
    ASSERT_TRUE(allocator.zero_allocate(SIZE_MAX, 2, NULL) == NULL);
    ASSERT_EQ(alloc_calls, before);
    ASSERT_EQ(hal_microros_allocator_stats().live_bytes, 0);
}
TEST(test_allocator_calloc_zero_and_null_cases) {
    uint8_t *p = allocator.zero_allocate(4, 9, NULL); ASSERT_TRUE(p != NULL);
    for (int i = 0; i < 36; i++) ASSERT_EQ(p[i], 0);
    ASSERT_TRUE(allocator.reallocate(p, 36, NULL) == p);
    allocator.deallocate(p, NULL); allocator.deallocate(NULL, NULL);
    ASSERT_TRUE(allocator.allocate(0, NULL) == NULL);
    ASSERT_TRUE(allocator.zero_allocate(0, SIZE_MAX, NULL) == NULL);
    p = allocator.reallocate(NULL, 8, NULL); ASSERT_TRUE(p != NULL); allocator.deallocate(p, NULL);
    ASSERT_EQ(hal_microros_allocator_stats().live_blocks, 0);
}
TEST(test_platform_monotonic_clock_and_yielding_sleep) {
    now_us = UINT64_C(4000000123456); struct timespec stamp;
    ASSERT_EQ(clock_gettime(CLOCK_MONOTONIC, &stamp), 0);
    ASSERT_EQ(stamp.tv_sec, 4000000); ASSERT_EQ(stamp.tv_nsec, 123456000);
    scheduler = taskSCHEDULER_RUNNING; delayed = 0; waited_us = 0;
    ASSERT_EQ(usleep(1001), 0); ASSERT_EQ(delayed, 2); ASSERT_EQ(waited_us, 0);
    ASSERT_EQ(usleep(50), 0); ASSERT_EQ(waited_us, 50);
    scheduler = 0; delayed = 0;
    ASSERT_EQ(usleep(2000), 0); ASSERT_EQ(delayed, 0); ASSERT_EQ(waited_us, 2000);
}
int main(void) {
    RUN_TEST(test_platform_install_failure_and_once_only_success);
    RUN_TEST(test_allocator_alignment_resize_contents_and_stats);
    RUN_TEST(test_allocator_failed_realloc_preserves_old_and_overflow_does_not_allocate);
    RUN_TEST(test_allocator_calloc_zero_and_null_cases);
    RUN_TEST(test_platform_monotonic_clock_and_yielding_sleep);
    TEST_SUMMARY();
}
