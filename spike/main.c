/* Phase 0 spike (throwaway): micro-ROS Jazzy over WiFi/UDP on Pico 2 W with FreeRTOS SMP.
 *
 * core 0: task "uros"   - WiFi, micro-ROS node umbreon_spike:
 *                         /spike/count (Int32, 50 Hz), /spike/echo_in -> /spike/echo_out
 * core 1: task "jitter" - toggles GP4 at 50 Hz, logs worst period jitter every 5 s
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/inet.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "udp_transport.h"

#define JITTER_GPIO          4
#define PERIOD_US            20000
#define JITTER_REPORT_TICKS  250          /* 250 * 20 ms = 5 s */
#define PING_PERIOD_MS       500
#define PING_TIMEOUT_MS      50
#define PING_FAILS_TO_DROP   6          /* ~3 s without a pong -> agent lost */
#define WIFI_CONNECT_TIMEOUT_MS 15000

#define LOG(fmt, ...) printf("[%8lu] " fmt "\n", (unsigned long)(time_us_64() / 1000), ##__VA_ARGS__)

/* ---------------- libc hooks needed by libmicroros ---------------- */

int clock_gettime(clockid_t clk, struct timespec *tp)
{
    (void)clk;
    uint64_t us = time_us_64();
    tp->tv_sec = (time_t)(us / 1000000u);
    tp->tv_nsec = (long)((us % 1000000u) * 1000u);
    return 0;
}

int usleep(useconds_t us)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && us >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(us / 1000));
    } else {
        busy_wait_us(us);
    }
    return 0;
}

/* ---------------- FreeRTOS hooks ---------------- */

void vApplicationMallocFailedHook(void)
{
    printf("FATAL: FreeRTOS malloc failed\n");
    for (;;) { tight_loop_contents(); }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    printf("FATAL: stack overflow in %s\n", name);
    for (;;) { tight_loop_contents(); }
}

/* ---------------- rcl allocator on the FreeRTOS heap ---------------- */
/* Each block carries an 8-byte header with its size so reallocate can copy. */

#define HDR 8u

static void *fr_alloc(size_t size, void *state)
{
    (void)state;
    uint8_t *p = pvPortMalloc(size + HDR);
    if (!p) { return NULL; }
    *(size_t *)p = size;
    return p + HDR;
}

static void fr_free(void *ptr, void *state)
{
    (void)state;
    if (ptr) { vPortFree((uint8_t *)ptr - HDR); }
}

static void *fr_realloc(void *ptr, size_t size, void *state)
{
    if (!ptr) { return fr_alloc(size, state); }
    size_t old = *(size_t *)((uint8_t *)ptr - HDR);
    void *n = fr_alloc(size, state);
    if (!n) { return NULL; }
    memcpy(n, ptr, old < size ? old : size);
    fr_free(ptr, state);
    return n;
}

static void *fr_calloc(size_t count, size_t size, void *state)
{
    void *p = fr_alloc(count * size, state);
    if (p) { memset(p, 0, count * size); }
    return p;
}

static void install_allocator(void)
{
    rcl_allocator_t a = rcutils_get_zero_initialized_allocator();
    a.allocate = fr_alloc;
    a.deallocate = fr_free;
    a.reallocate = fr_realloc;
    a.zero_allocate = fr_calloc;
    if (!rcutils_set_default_allocator(&a)) {
        LOG("rcutils_set_default_allocator failed");
    }
}

/* ---------------- core 1: GPIO toggle + jitter ---------------- */

static void jitter_task(void *arg)
{
    (void)arg;
    gpio_init(JITTER_GPIO);
    gpio_set_dir(JITTER_GPIO, GPIO_OUT);
    TickType_t wake = xTaskGetTickCount();
    uint64_t prev = time_us_64();
    int64_t worst_window = 0, worst_ever = 0;
    uint32_t n = 0;
    bool level = false;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_US / 1000));
        uint64_t now = time_us_64();
        level = !level;
        gpio_put(JITTER_GPIO, level);
        int64_t dev = (int64_t)(now - prev) - PERIOD_US;
        if (dev < 0) { dev = -dev; }
        prev = now;
        if (n > 0 && dev > worst_window) { worst_window = dev; }
        if (++n % JITTER_REPORT_TICKS == 0) {
            if (worst_window > worst_ever) { worst_ever = worst_window; }
            LOG("JITTER core=%u worst_5s=%lld us worst_ever=%lld us",
                get_core_num(), (long long)worst_window, (long long)worst_ever);
            worst_window = 0;
        }
    }
}

/* ---------------- core 0: WiFi + micro-ROS ---------------- */

static udp_transport g_transport = { .sock = -1 };

static rcl_node_t node;
static rclc_support_t support;
static rcl_publisher_t pub_count, pub_echo;
static rcl_subscription_t sub_echo;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__Int32 msg_count, msg_echo_in;
static uint32_t echo_count;

static bool wifi_up(void)
{
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

static void wifi_connect_blocking(void)
{
    while (!wifi_up()) {
        LOG("WIFI connecting");
        int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                    CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS);
        if (rc == 0 && wifi_up()) {
            break;
        }
        LOG("WIFI connect failed rc=%d status=%d", rc,
            cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    struct netif *nif = &cyw43_state.netif[CYW43_ITF_STA];
    LOG("WIFI up ip=%s", ip4addr_ntoa(netif_ip4_addr(nif)));
    LOG("WIFI mask=%s", ip4addr_ntoa(netif_ip4_netmask(nif))); /* ntoa uses one static buffer */
    LOG("WIFI gw=%s", ip4addr_ntoa(netif_ip4_gw(nif)));
    cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM); /* power save adds 100+ ms latency */
    int32_t rssi = 0;
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    LOG("WIFI rssi=%ld dBm", (long)rssi);
}

static void timer_cb(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (t == NULL) { return; }
    rcl_ret_t rc = rcl_publish(&pub_count, &msg_count, NULL);
    (void)rc;
    msg_count.data++;
}

static void echo_cb(const void *msgin)
{
    const std_msgs__msg__Int32 *in = (const std_msgs__msg__Int32 *)msgin;
    std_msgs__msg__Int32 out = { .data = in->data };
    rcl_ret_t rc = rcl_publish(&pub_echo, &out, NULL);
    (void)rc;
    echo_count++;
}

#define RCCHECK(x) do { rcl_ret_t _rc = (x); if (_rc != RCL_RET_OK) { \
    LOG("RCL fail %s rc=%d", #x, (int)_rc); rcl_reset_error(); return false; } } while (0)

static bool entities_create(void)
{
    rcl_allocator_t alloc = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &alloc));
    RCCHECK(rclc_node_init_default(&node, "umbreon_spike", "", &support));
    RCCHECK(rclc_publisher_init_best_effort(&pub_count, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/spike/count"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_echo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/spike/echo_out"));
    RCCHECK(rclc_subscription_init_best_effort(&sub_echo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/spike/echo_in"));
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PERIOD_US / 1000), timer_cb, true));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &alloc));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_echo, &msg_echo_in, echo_cb, ON_NEW_DATA));
    return true;
}

static void entities_destroy(void)
{
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    if (rmw_ctx) {
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    }
    rcl_ret_t rc;
    rc = rcl_publisher_fini(&pub_count, &node);
    rc = rcl_publisher_fini(&pub_echo, &node);
    rc = rcl_subscription_fini(&sub_echo, &node);
    rc = rcl_timer_fini(&timer);
    rc = rclc_executor_fini(&executor);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
    (void)rc; /* best effort teardown: the agent may already be gone */
    rcl_reset_error();
}

static void resolve_agent(void)
{
    if (strlen(AGENT_IP) > 0) {
        g_transport.remote_addr = ipaddr_addr(AGENT_IP);
        LOG("AGENT static %s:%d", AGENT_IP, AGENT_PORT);
        return;
    }
    /* Step 3: AGENT_IP empty -> try discovery. */
    struct netif *nif = &cyw43_state.netif[CYW43_ITF_STA];
    uint32_t ip = ip4_addr_get_u32(netif_ip4_addr(nif));
    uint32_t mask = ip4_addr_get_u32(netif_ip4_netmask(nif));
    const struct { uint32_t addr; uint16_t port; const char *what; } tries[] = {
        { (ip & mask) | ~mask, AGENT_PORT, "subnet broadcast" },
        { ipaddr_addr("255.255.255.255"), AGENT_PORT, "limited broadcast" },
        { ipaddr_addr("239.255.0.2"), 7400, "XRCE discovery multicast 239.255.0.2:7400" },
    };
    for (;;) {
        for (size_t i = 0; i < sizeof(tries) / sizeof(tries[0]); i++) {
            uint32_t found = 0;
            ip4_addr_t a = { .addr = tries[i].addr };
            LOG("DISCOVERY try %s (%s:%u)", tries[i].what, ip4addr_ntoa(&a), tries[i].port);
            if (udp_transport_discover(tries[i].addr, tries[i].port, 500, 3, &found)) {
                ip4_addr_t f = { .addr = found };
                LOG("DISCOVERY found agent at %s via %s", ip4addr_ntoa(&f), tries[i].what);
                g_transport.remote_addr = found;
                return;
            }
            LOG("DISCOVERY no reply via %s", tries[i].what);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

typedef enum { ST_WAIT_AGENT, ST_CONNECTED } link_state;

static void uros_task(void *arg)
{
    (void)arg;
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE) != 0) {
        LOG("cyw43_arch_init failed");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();
    wifi_connect_blocking();

    install_allocator();
    g_transport.remote_port = AGENT_PORT;
    resolve_agent();
    rmw_uros_set_custom_transport(false, &g_transport, udp_transport_open, udp_transport_close,
                                  udp_transport_write, udp_transport_read);

    link_state st = ST_WAIT_AGENT;
    uint64_t lost_at_ms = 0, last_ping_ms = 0, last_stat_ms = 0;
    int ping_fails = 0;
    uint32_t ping_fail_total = 0, ping_total = 0, ping_max_ms = 0;
    uint32_t sessions = 0;
    for (;;) {
        if (!wifi_up()) {
            LOG("WIFI lost (status=%d)", cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA));
            if (st == ST_CONNECTED) {
                entities_destroy();
                st = ST_WAIT_AGENT;
                lost_at_ms = time_us_64() / 1000;
            }
            wifi_connect_blocking();
        }
        uint64_t now_ms = time_us_64() / 1000;
        switch (st) {
        case ST_WAIT_AGENT:
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
                LOG("AGENT reachable, creating entities");
                if (entities_create()) {
                    sessions++;
                    st = ST_CONNECTED;
                    ping_fails = 0;
                    last_ping_ms = now_ms;
                    LOG("SESSION up #%lu (%lu ms after loss)", (unsigned long)sessions,
                        (unsigned long)(lost_at_ms ? now_ms - lost_at_ms : 0));
                } else {
                    entities_destroy();
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            break;
        case ST_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            if (now_ms - last_ping_ms >= PING_PERIOD_MS) {
                last_ping_ms = now_ms;
                ping_total++;
                uint64_t t0 = time_us_64();
                rmw_ret_t pr = rmw_uros_ping_agent(PING_TIMEOUT_MS, 1);
                uint32_t pms = (uint32_t)((time_us_64() - t0) / 1000);
                if (pms > ping_max_ms) { ping_max_ms = pms; }
                if (pr == RMW_RET_OK) {
                    ping_fails = 0;
                } else if (ping_fail_total++, ++ping_fails >= PING_FAILS_TO_DROP) {
                    LOG("SESSION lost (%d pings failed)", ping_fails);
                    entities_destroy();
                    st = ST_WAIT_AGENT;
                    lost_at_ms = now_ms;
                }
            }
            if (now_ms - last_stat_ms >= 5000) {
                last_stat_ms = now_ms;
                LOG("STAT count=%ld echoes=%lu pings=%lu ping_fail=%lu ping_max=%lums heap_free=%u core=%u",
                    (long)msg_count.data, (unsigned long)echo_count, (unsigned long)ping_total,
                    (unsigned long)ping_fail_total, (unsigned long)ping_max_ms,
                    (unsigned)xPortGetFreeHeapSize(), get_core_num());
                ping_max_ms = 0;
            }
            break;
        }
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000); /* give the host a moment to open the USB CDC port */
    printf("\numbreon_spike boot, AGENT_IP='%s' port=%d\n", AGENT_IP, AGENT_PORT);

    TaskHandle_t h_uros, h_jitter;
    xTaskCreate(uros_task, "uros", 8192, NULL, tskIDLE_PRIORITY + 2, &h_uros);
    vTaskCoreAffinitySet(h_uros, 1u << 0);
    xTaskCreate(jitter_task, "jitter", 1024, NULL, configMAX_PRIORITIES - 2, &h_jitter);
    vTaskCoreAffinitySet(h_jitter, 1u << 1);
    vTaskStartScheduler();
    return 0;
}
