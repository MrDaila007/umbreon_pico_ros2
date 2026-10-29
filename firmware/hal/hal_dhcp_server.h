/* One AP DHCP server. RAW API calls are made only under the CYW43/lwIP core lock. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "umb_dhcp.h"
struct netif;
typedef struct { uint32_t ip; uint8_t mac[6]; } hal_dhcp_client_t;
bool hal_dhcp_server_start(struct netif *ap, uint32_t server_ip);
void hal_dhcp_server_stop(void);
size_t hal_dhcp_server_clients(hal_dhcp_client_t out[UMB_DHCP_LEASES]);
uint32_t hal_dhcp_server_errors(void);
