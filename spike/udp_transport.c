/* Phase 0 spike: micro-ROS custom transport (framing=false) over lwIP BSD sockets. */
#include "udp_transport.h"

#include <string.h>
#include "lwip/sockets.h"
#include <uxr/client/util/ping.h>

bool udp_transport_open(struct uxrCustomTransport *transport)
{
    udp_transport *t = (udp_transport *)transport->args;
    t->sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (t->sock < 0) {
        return false;
    }
    struct sockaddr_in local = {0};
    local.sin_family = AF_INET;
    local.sin_port = 0;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(t->sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        lwip_close(t->sock);
        t->sock = -1;
        return false;
    }
    return true;
}

bool udp_transport_close(struct uxrCustomTransport *transport)
{
    udp_transport *t = (udp_transport *)transport->args;
    if (t->sock >= 0) {
        lwip_close(t->sock);
        t->sock = -1;
    }
    return true;
}

size_t udp_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    udp_transport *t = (udp_transport *)transport->args;
    struct sockaddr_in to = {0};
    to.sin_family = AF_INET;
    to.sin_port = htons(t->remote_port);
    to.sin_addr.s_addr = t->remote_addr;
    int n = lwip_sendto(t->sock, buf, len, 0, (struct sockaddr *)&to, sizeof(to));
    if (n < 0) {
        *err = 1;
        return 0;
    }
    return (size_t)n;
}

size_t udp_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    udp_transport *t = (udp_transport *)transport->args;
    int flags = 0;
    if (timeout <= 0) {
        flags = MSG_DONTWAIT; /* SO_RCVTIMEO of 0 would mean "block forever" in lwIP */
    } else {
        struct timeval tv = { .tv_sec = timeout / 1000, .tv_usec = (timeout % 1000) * 1000 };
        lwip_setsockopt(t->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    int n = lwip_recvfrom(t->sock, buf, len, flags, (struct sockaddr *)&from, &from_len);
    if (n <= 0) {
        *err = 1;
        return 0;
    }
    t->last_from_addr = from.sin_addr.s_addr;
    return (size_t)n;
}

bool udp_transport_discover(uint32_t dest_addr, uint16_t port, int timeout_ms, uint8_t attempts,
                            uint32_t *found_addr)
{
    static uxrCustomTransport ct;
    static udp_transport t;
    memset(&t, 0, sizeof(t));
    t.remote_addr = dest_addr;
    t.remote_port = port;
    t.sock = -1;
    uxr_set_custom_transport_callbacks(&ct, false, udp_transport_open, udp_transport_close,
                                       udp_transport_write, udp_transport_read);
    if (!uxr_init_custom_transport(&ct, &t)) {
        return false;
    }
    bool ok = uxr_ping_agent_attempts(&ct.comm, timeout_ms, attempts);
    if (ok) {
        *found_addr = t.last_from_addr;
    }
    uxr_close_custom_transport(&ct);
    return ok;
}
