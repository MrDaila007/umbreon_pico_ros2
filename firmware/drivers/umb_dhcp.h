/* Small bounded DHCPv4 codec/lease table for the car's isolated /24 AP. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define UMB_DHCP_LEASES 8
#define UMB_DHCP_REPLY_CAP 300u
#define UMB_DHCP_PACKET_CAP 600u
typedef struct { uint8_t mac[6]; uint32_t expires_ms; bool used, bound; } umb_dhcp_lease_t;
typedef struct { uint32_t server_ip; umb_dhcp_lease_t leases[UMB_DHCP_LEASES]; } umb_dhcp_t;
umb_dhcp_t umb_dhcp_init(uint32_t server_ip);
void umb_dhcp_expire(umb_dhcp_t *s, uint32_t now_ms);
/* 1 reply, 0 ignore/release/no free lease, -1 malformed, -2 insufficient output.
 * Reply uses broadcast UDP68 on the AP interface. No DNS server is advertised. */
int umb_dhcp_process(umb_dhcp_t *s, const uint8_t *packet, size_t size, uint32_t now_ms,
                     uint8_t *reply, size_t capacity, size_t *reply_size);
/* Returns 0 if unused, offered-only, or expired; addresses begin at subnet .2. */
uint32_t umb_dhcp_bound_ip(const umb_dhcp_t *s, unsigned slot, uint32_t now_ms);
