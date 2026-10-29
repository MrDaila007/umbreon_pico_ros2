/* Bounded endpoint selection; no sockets, SDK, heap or secret data.
 * Owner reconciles observations BEFORE delivering a probe result each iteration. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define UMB_ENDPOINT_CLIENTS 8u
#define UMB_ENDPOINT_PROBE_MS 100u
#define UMB_ENDPOINT_RETRY_MS 100u
typedef enum { UMB_ENDPOINT_OFF, UMB_ENDPOINT_STA, UMB_ENDPOINT_AP } umb_endpoint_mode_t;
typedef enum { UMB_ENDPOINT_NONE, UMB_ENDPOINT_CONFIGURED,
               UMB_ENDPOINT_BROADCAST, UMB_ENDPOINT_DHCP } umb_endpoint_method_t;
typedef struct { uint32_t ip; uint8_t mac[6]; } umb_endpoint_client_t;
typedef struct {
    umb_endpoint_mode_t mode;
    uint32_t configured_ip, local_ip, mask;
    uint16_t port;
    umb_endpoint_client_t leases[UMB_ENDPOINT_CLIENTS];
    size_t lease_count;
    uint8_t associated[UMB_ENDPOINT_CLIENTS][6];
    size_t associated_count;
    bool associations_known;
} umb_endpoint_input_t;
typedef struct {
    uint32_t token, ip;
    uint16_t port, timeout_ms;
    bool broadcast;
} umb_endpoint_probe_t;
typedef struct {
    umb_endpoint_mode_t mode;
    umb_endpoint_method_t method, pending_method;
    uint32_t local_ip, mask, configured_ip, selected_ip;
    uint16_t port;
    uint8_t selected_mac[6], pending_mac[6];
    uint32_t serial, pending_token, pending_ip, deadline_ms, retry_ms;
    size_t cursor;
    bool initialized, selected, pending;
} umb_endpoint_t;
umb_endpoint_t umb_endpoint_init(void);
/* Returns true on loss of a previously selected endpoint. Cancels stale probes. */
bool umb_endpoint_reconcile(umb_endpoint_t *s, const umb_endpoint_input_t *in, uint32_t now_ms);
/* At most one outstanding probe; timeout also cancels its result token. */
bool umb_endpoint_next(umb_endpoint_t *s, const umb_endpoint_input_t *in, uint32_t now_ms,
                        umb_endpoint_probe_t *probe);
/* xrce_ok must come from the dedicated validated XRCE probe, never raw UDP receipt. */
bool umb_endpoint_result(umb_endpoint_t *s, uint32_t token, bool xrce_ok,
                          uint32_t reply_ip, uint32_t now_ms);
/* Session failure: rediscover, keeping AP round-robin cursor for fairness. */
void umb_endpoint_clear(umb_endpoint_t *s, uint32_t now_ms);
