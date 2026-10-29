#pragma once
#include "lwip/raw_types.h"
struct netif;
struct pbuf;
struct udp_pcb { int marker; };
typedef void (*udp_recv_fn)(void *, struct udp_pcb *, struct pbuf *, const ip_addr_t *, u16_t);
struct udp_pcb *udp_new_ip_type(u8_t type);
err_t udp_bind(struct udp_pcb *pcb, const ip_addr_t *ip, u16_t port);
void udp_bind_netif(struct udp_pcb *pcb, const struct netif *netif);
void udp_recv(struct udp_pcb *pcb, udp_recv_fn callback, void *args);
void udp_remove(struct udp_pcb *pcb);
err_t udp_sendto_if(struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *ip, u16_t port, struct netif *netif);
