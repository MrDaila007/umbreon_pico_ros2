/* Phase 0 spike: micro-ROS custom transport over lwIP UDP sockets. */
#ifndef SPIKE_UDP_TRANSPORT_H
#define SPIKE_UDP_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

typedef struct udp_transport {
    uint32_t remote_addr;   /* network byte order */
    uint16_t remote_port;   /* host byte order */
    int sock;
    uint32_t last_from_addr; /* source of the last datagram received (network order) */
} udp_transport;

bool udp_transport_open(struct uxrCustomTransport *transport);
bool udp_transport_close(struct uxrCustomTransport *transport);
size_t udp_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t udp_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

/* Send an XRCE GET_INFO (the same message rmw_uros_ping_agent sends) to dest_addr:port
 * (broadcast or multicast) and return the source address of the first INFO reply.
 * Stand-in for rmw_uros_discover_agent(), which this custom-transport build of
 * libmicroros does not contain. */
bool udp_transport_discover(uint32_t dest_addr, uint16_t port, int timeout_ms, uint8_t attempts,
                            uint32_t *found_addr);

#endif
