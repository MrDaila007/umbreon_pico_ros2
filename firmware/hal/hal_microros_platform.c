#include "hal_microros_platform.h"
#include "hal_time.h"
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/time.h"
#include <rcutils/allocator.h>

typedef union { max_align_t alignment; size_t size; } allocation_header_t;
_Static_assert(portBYTE_ALIGNMENT >= _Alignof(max_align_t), "FreeRTOS heap alignment must support ROS allocations");
static hal_microros_alloc_stats_t s_stats;
static bool s_installed;

static void *allocate(size_t size, void *ctx) {
    (void)ctx;
    if (!size) return NULL;
    if (size > SIZE_MAX - sizeof(allocation_header_t)) { s_stats.failures++; return NULL; }
    allocation_header_t *header = pvPortMalloc(sizeof *header + size);
    if (!header) { s_stats.failures++; return NULL; }
    header->size = size;
    s_stats.live_blocks++; s_stats.live_bytes += size;
    if (s_stats.live_bytes > s_stats.peak_bytes) s_stats.peak_bytes = s_stats.live_bytes;
    return header + 1;
}
static void deallocate(void *ptr, void *ctx) {
    (void)ctx;
    if (!ptr) return;
    allocation_header_t *header = (allocation_header_t *)ptr - 1;
    s_stats.live_blocks--; s_stats.live_bytes -= header->size;
    vPortFree(header);
}
static void *reallocate(void *ptr, size_t size, void *ctx) {
    if (!ptr) return allocate(size, ctx);
    if (!size) { deallocate(ptr, ctx); return NULL; }
    allocation_header_t *old = (allocation_header_t *)ptr - 1;
    if (old->size == size) return ptr;
    void *fresh = allocate(size, ctx);
    if (!fresh) return NULL; /* old pointer/data stay owned and intact */
    memcpy(fresh, ptr, old->size < size ? old->size : size);
    deallocate(ptr, ctx); return fresh;
}
static void *zero_allocate(size_t count, size_t size, void *ctx) {
    if (count && size > SIZE_MAX / count) { s_stats.failures++; return NULL; }
    size_t bytes = count * size;
    void *ptr = allocate(bytes, ctx);
    if (ptr) memset(ptr, 0, bytes);
    return ptr;
}
bool hal_microros_platform_init(void) {
    if (s_installed) return true;
    rcutils_allocator_t allocator = {.allocate = allocate, .deallocate = deallocate,
        .reallocate = reallocate, .zero_allocate = zero_allocate, .state = NULL};
    s_installed = rcutils_set_default_allocator(&allocator);
    return s_installed;
}
hal_microros_alloc_stats_t hal_microros_allocator_stats(void) { return s_stats; }

/* micro-ROS time sync supplies the epoch offset separately. Libc clocks here are
 * monotonic boot time; network/ROS time changes never affect command safety. */
int clock_gettime(clockid_t id, struct timespec *tp) {
    (void)id;
    uint64_t us = hal_now_us64();
    tp->tv_sec = (time_t)(us / 1000000u);
    tp->tv_nsec = (long)((us % 1000000u) * 1000u);
    return 0;
}
int usleep(useconds_t us) {
    if (us >= 1000u && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        TickType_t ticks = (TickType_t)(((uint64_t)us * configTICK_RATE_HZ + 999999u) / 1000000u);
        vTaskDelay(ticks ? ticks : 1);
    } else busy_wait_us(us);
    return 0;
}
