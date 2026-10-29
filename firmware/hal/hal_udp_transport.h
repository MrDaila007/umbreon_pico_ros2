/* Production datagram callbacks. Addresses use settings host-order IPv4 values. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
typedef struct {
    int socket;
    uint32_t remote_ip;
    uint16_t remote_port;
    uint32_t active_ip;
    uint16_t active_port; /* immutable for one open socket */
    bool probe_replies; /* only the private discovery context enables this */
    uint32_t last_sender_ip;
    uint32_t sent, received, dropped, errors;
} hal_udp_transport_t;
/* Initialize once, or only after close succeeded; do not overwrite an owned fd. */
void hal_udp_transport_init(hal_udp_transport_t *state, uint32_t ip, uint16_t port);
bool hal_udp_transport_open(struct uxrCustomTransport *transport);
bool hal_udp_transport_close(struct uxrCustomTransport *transport);
size_t hal_udp_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *error);
size_t hal_udp_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout_ms, uint8_t *error);

/* Separate bounded XRCE ping; never changes the registered production transport.
 * broadcast=true accepts candidate source IPs, but only XRCE-valid ping selects one. */
bool hal_udp_probe(uint32_t destination_ip, uint16_t port, unsigned timeout_ms,
                   bool broadcast, uint32_t *found_ip);

/* The single network owner must close the private probe before changing WiFi mode. */
bool hal_udp_probe_close(void);
