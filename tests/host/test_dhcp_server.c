/* Actual RAW wrapper with chained pbuf/failure/core-lock fakes, no network I/O. */
#include "test_runner.h"
#include "hal_dhcp_server.h"
#include "hal_time.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include <string.h>
static int lock_depth, violations, live_pcbs, live_buffers, sent, copies;
static bool fail_new, fail_bind, fail_alloc, fail_take, fail_send;
static struct udp_pcb pcb;
static struct netif ap = {1};
static udp_recv_fn callback;
static void *callback_args;
static struct pbuf first, second, outgoing;
static uint8_t packet[600], reply[300];
uint32_t hal_now_ms(void) { return 100; }
void umb_test_lwip_begin(void) { lock_depth++; }
void umb_test_lwip_end(void) { lock_depth--; if (lock_depth < 0) violations++; }
static void locked(void) { if (lock_depth <= 0) violations++; }
struct udp_pcb *udp_new_ip_type(u8_t type) { (void)type; locked(); if (fail_new) return NULL; live_pcbs++; return &pcb; }
err_t udp_bind(struct udp_pcb *p, const ip_addr_t *ip, u16_t port) { (void)p; (void)ip; locked(); if (port != 67) violations++; return fail_bind ? ERR_MEM : ERR_OK; }
void udp_bind_netif(struct udp_pcb *p, const struct netif *n) { (void)p; locked(); if (n != &ap) violations++; }
void udp_recv(struct udp_pcb *p, udp_recv_fn cb, void *args) { (void)p; locked(); callback = cb; callback_args = args; }
void udp_remove(struct udp_pcb *p) { (void)p; locked(); live_pcbs--; }
struct pbuf *pbuf_alloc(int layer, u16_t n, int type) {
    (void)layer; (void)type; locked(); if (fail_alloc) return NULL;
    if (n > sizeof reply) { violations++; return NULL; }
    outgoing = (struct pbuf){NULL, reply, n, n}; live_buffers++; return &outgoing;
}
u16_t pbuf_copy_partial(const struct pbuf *p, void *out, u16_t n, u16_t offset) {
    locked(); copies++; unsigned copied = 0;
    while (p && copied < n) {
        if (offset >= p->len) { offset = (u16_t)(offset - p->len); p = p->next; continue; }
        unsigned take = p->len - offset;
        if (take > n - copied) take = n - copied;
        memcpy((uint8_t *)out + copied, (uint8_t *)p->payload + offset, take);
        copied += take; offset = 0; p = p->next;
    }
    return (u16_t)copied;
}
err_t pbuf_take(struct pbuf *p, const void *input, u16_t n) {
    locked(); if (fail_take) return ERR_MEM; memcpy(p->payload, input, n); return ERR_OK;
}
u8_t pbuf_free(struct pbuf *p) {
    locked(); u8_t count = 0;
    for (; p; p = p->next) { live_buffers--; count++; }
    return count;
}
err_t udp_sendto_if(struct udp_pcb *p, struct pbuf *buf, const ip_addr_t *ip, u16_t port, struct netif *netif) {
    (void)p; locked();
    if (buf->len != 300 || ip->addr != UINT32_MAX || port != 68 || netif != &ap || ((uint8_t *)buf->payload)[0] != 2) violations++;
    if (fail_send) return ERR_MEM;
    sent++; return ERR_OK;
}
static void setup(void) {
    hal_dhcp_server_stop();
    lock_depth = violations = live_pcbs = live_buffers = sent = copies = 0;
    fail_new = fail_bind = fail_alloc = fail_take = fail_send = false;
    callback = NULL;
}
static void deliver(uint8_t type, bool chain, u16_t port) {
    memset(packet, 0, sizeof packet);
    packet[0] = packet[1] = 1; packet[2] = 6; packet[28] = 2; packet[33] = 42;
    packet[236] = 0x63; packet[237] = 0x82; packet[238] = 0x53; packet[239] = 0x63;
    packet[240] = 53; packet[241] = 1; packet[242] = type;
    unsigned n = 243;
    if (type == 3) {
        packet[n++] = 50; packet[n++] = 4; packet[n++] = 192; packet[n++] = 168; packet[n++] = 4; packet[n++] = 2;
    }
    packet[n++] = 255;
    first = (struct pbuf){NULL, packet, (u16_t)n, (u16_t)n};
    if (chain) {
        first.len = 120; first.next = &second;
        second = (struct pbuf){NULL, packet + 120, (u16_t)(n - 120), (u16_t)(n - 120)};
    }
    live_buffers += chain ? 2 : 1;
    umb_test_lwip_begin(); callback(callback_args, &pcb, &first, NULL, port); umb_test_lwip_end();
}
TEST(test_dhcp_raw_chained_packets_and_only_acked_clients) {
    setup(); ASSERT_TRUE(hal_dhcp_server_start(&ap, 0xc0a80401u));
    deliver(1, true, 68); ASSERT_EQ(sent, 1); ASSERT_EQ(copies, 1); ASSERT_EQ(live_buffers, 0);
    hal_dhcp_client_t clients[8]; ASSERT_EQ(hal_dhcp_server_clients(clients), 0);
    deliver(3, true, 68); ASSERT_EQ(hal_dhcp_server_clients(clients), 1);
    ASSERT_EQ(clients[0].ip, 0xc0a80402u); ASSERT_EQ(clients[0].mac[5], 42);
    for (int i = 0; i < 100; i++) deliver(3, true, 68);
    ASSERT_EQ(live_buffers, 0); ASSERT_EQ(live_pcbs, 1); ASSERT_EQ(violations, 0); ASSERT_EQ(lock_depth, 0);
    hal_dhcp_server_stop(); hal_dhcp_server_stop(); ASSERT_EQ(live_pcbs, 0);
}
TEST(test_dhcp_raw_resource_failures_cleanup) {
    setup(); fail_new = true; ASSERT_FALSE(hal_dhcp_server_start(&ap, 0xc0a80401u));
    fail_new = false; fail_bind = true; ASSERT_FALSE(hal_dhcp_server_start(&ap, 0xc0a80401u)); ASSERT_EQ(live_pcbs, 0);
    fail_bind = false; ASSERT_TRUE(hal_dhcp_server_start(&ap, 0xc0a80401u));
    ASSERT_TRUE(hal_dhcp_server_start(&ap, 0xc0a80401u)); ASSERT_EQ(live_pcbs, 1);
    uint32_t errors = hal_dhcp_server_errors();
    fail_alloc = true; deliver(1, true, 68); ASSERT_EQ(live_buffers, 0);
    fail_alloc = false; fail_take = true; deliver(1, true, 68); ASSERT_EQ(live_buffers, 0);
    fail_take = false; fail_send = true; deliver(1, true, 68); ASSERT_EQ(live_buffers, 0);
    fail_send = false; deliver(1, true, 67); ASSERT_EQ(live_buffers, 0);
    ASSERT_EQ(hal_dhcp_server_errors(), errors + 4); ASSERT_EQ(violations, 0);
    hal_dhcp_server_stop(); ASSERT_EQ(live_pcbs, 0);
}
int main(void) {
    RUN_TEST(test_dhcp_raw_chained_packets_and_only_acked_clients);
    RUN_TEST(test_dhcp_raw_resource_failures_cleanup);
    TEST_SUMMARY();
}
