#include "hal_dhcp_server.h"
#include "hal_time.h"
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
static struct { struct udp_pcb *pcb; struct netif *ap; umb_dhcp_t table; uint32_t errors; } s_dhcp;

/* lwIP invokes this under the same core lock used by the public entry points. */
static void receive(void *ctx, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    (void)ctx; (void)addr;
    if (!p) return;
    uint8_t input[UMB_DHCP_PACKET_CAP], output[UMB_DHCP_REPLY_CAP];
    size_t size = p->tot_len;
    if (port != 68 || size > sizeof input || pbuf_copy_partial(p, input, (u16_t)size, 0) != size) {
        s_dhcp.errors++; pbuf_free(p); return;
    }
    pbuf_free(p);
    size_t reply_size = 0;
    int rc = umb_dhcp_process(&s_dhcp.table, input, size, hal_now_ms(), output, sizeof output, &reply_size);
    if (rc < 0) { s_dhcp.errors++; return; }
    if (rc == 0) return;
    struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, (u16_t)reply_size, PBUF_RAM);
    if (!reply) { s_dhcp.errors++; return; }
    ip_addr_t broadcast; IP_ADDR4(&broadcast, 255, 255, 255, 255);
    if (pbuf_take(reply, output, (u16_t)reply_size) != ERR_OK ||
        udp_sendto_if(pcb, reply, &broadcast, 68, s_dhcp.ap) != ERR_OK) s_dhcp.errors++;
    pbuf_free(reply);
}
bool hal_dhcp_server_start(struct netif *ap, uint32_t ip) {
    if (!ap || !ip) return false;
    cyw43_arch_lwip_begin();
    if (s_dhcp.pcb) {
        bool same = s_dhcp.ap == ap && s_dhcp.table.server_ip == ip;
        cyw43_arch_lwip_end(); return same;
    }
    s_dhcp.table = umb_dhcp_init(ip); s_dhcp.ap = ap;
    s_dhcp.pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (s_dhcp.pcb && udp_bind(s_dhcp.pcb, IP_ANY_TYPE, 67) == ERR_OK) {
        udp_bind_netif(s_dhcp.pcb, ap);
        udp_recv(s_dhcp.pcb, receive, NULL);
        cyw43_arch_lwip_end(); return true;
    }
    if (s_dhcp.pcb) udp_remove(s_dhcp.pcb);
    s_dhcp.pcb = NULL; s_dhcp.ap = NULL; s_dhcp.errors++;
    cyw43_arch_lwip_end(); return false;
}
void hal_dhcp_server_stop(void) {
    cyw43_arch_lwip_begin();
    if (s_dhcp.pcb) udp_remove(s_dhcp.pcb);
    s_dhcp.pcb = NULL; s_dhcp.ap = NULL;
    memset(&s_dhcp.table, 0, sizeof s_dhcp.table);
    cyw43_arch_lwip_end();
}
size_t hal_dhcp_server_clients(hal_dhcp_client_t out[UMB_DHCP_LEASES]) {
    if (!out) return 0;
    cyw43_arch_lwip_begin();
    size_t count = 0;
    uint32_t now = hal_now_ms(); umb_dhcp_expire(&s_dhcp.table, now);
    if (s_dhcp.pcb) for (unsigned i = 0; i < UMB_DHCP_LEASES; i++) {
        uint32_t ip = umb_dhcp_bound_ip(&s_dhcp.table, i, now);
        if (!ip) continue;
        out[count].ip = ip; memcpy(out[count].mac, s_dhcp.table.leases[i].mac, 6); count++;
    }
    cyw43_arch_lwip_end(); return count;
}
uint32_t hal_dhcp_server_errors(void) {
    cyw43_arch_lwip_begin(); uint32_t count = s_dhcp.errors; cyw43_arch_lwip_end(); return count;
}
