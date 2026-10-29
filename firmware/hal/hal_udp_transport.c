#include "hal_udp_transport.h"
#include "hal_time.h"
#include <string.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <uxr/client/config.h>
#define MAX_READ_PACKETS 16u

void hal_udp_transport_init(hal_udp_transport_t *t, uint32_t ip, uint16_t port) {
    memset(t, 0, sizeof *t);
    t->socket = -1; t->remote_ip = ip; t->remote_port = port;
}
bool hal_udp_transport_close(struct uxrCustomTransport *transport) {
    hal_udp_transport_t *t = transport->args;
    if (t->socket >= 0 && lwip_close(t->socket) < 0) {
        t->errors++;
        if (errno != EBADF) return false; /* retain ownership for retry, do not leak the fd */
    }
    t->socket = -1; t->active_ip = 0; t->active_port = 0;
    return true;
}
bool hal_udp_transport_open(struct uxrCustomTransport *transport) {
    hal_udp_transport_t *t = transport->args;
    if (!t->remote_ip || !t->remote_port) return false;
    if (t->socket >= 0) return t->active_ip == t->remote_ip && t->active_port == t->remote_port;
    t->socket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (t->socket < 0) { t->errors++; return false; }
    struct sockaddr_in local = {.sin_family = AF_INET, .sin_port = 0};
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(t->socket, (struct sockaddr *)&local, sizeof local) < 0) {
        t->errors++; (void)hal_udp_transport_close(transport); return false;
    }
    struct timeval send_timeout = {.tv_sec = 0, .tv_usec = 20000};
    if (lwip_setsockopt(t->socket, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof send_timeout) < 0) {
        t->errors++; (void)hal_udp_transport_close(transport); return false;
    }
    t->active_ip = t->remote_ip; t->active_port = t->remote_port;
    return true;
}
size_t hal_udp_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *error) {
    hal_udp_transport_t *t = transport->args;
    *error = 1;
    if (t->socket < 0 || t->remote_ip != t->active_ip || t->remote_port != t->active_port ||
        !buf || !len || len > UXR_CONFIG_CUSTOM_TRANSPORT_MTU) { t->errors++; return 0; }
    struct sockaddr_in peer = {.sin_family = AF_INET, .sin_port = htons(t->remote_port)};
    peer.sin_addr.s_addr = htonl(t->remote_ip);
    int n = lwip_sendto(t->socket, buf, len, 0, (struct sockaddr *)&peer, sizeof peer);
    if (n < 0 || (size_t)n != len) { t->errors++; return 0; }
    t->sent++; *error = 0; return len;
}
size_t hal_udp_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len,
                              int timeout_ms, uint8_t *error) {
    hal_udp_transport_t *t = transport->args;
    *error = 1;
    if (t->socket < 0 || t->remote_ip != t->active_ip || t->remote_port != t->active_port || !buf || !len) { t->errors++; return 0; }
    uint32_t start = hal_now_ms();
    for (unsigned attempt = 0; attempt < MAX_READ_PACKETS; attempt++) {
        int flags = timeout_ms <= 0 ? MSG_DONTWAIT : 0;
        if (timeout_ms > 0) {
            uint32_t elapsed = hal_now_ms() - start;
            if (elapsed >= (uint32_t)timeout_ms) return 0;
            uint32_t remaining = (uint32_t)timeout_ms - elapsed;
            struct timeval tv = {.tv_sec = (long)(remaining / 1000u),
                                 .tv_usec = (long)((remaining % 1000u) * 1000u)};
            if (lwip_setsockopt(t->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv) < 0) { t->errors++; return 0; }
        }
        struct sockaddr_in peer = {0};
        struct iovec vector = {.iov_base = buf, .iov_len = len};
        struct msghdr msg = {.msg_name = &peer, .msg_namelen = sizeof peer,
                             .msg_iov = &vector, .msg_iovlen = 1};
        ssize_t n = lwip_recvmsg(t->socket, &msg, flags);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno != EAGAIN && errno != EWOULDBLOCK) t->errors++;
            return 0;
        }
        if (!n || (size_t)n > len || (msg.msg_flags & MSG_TRUNC) ||
            peer.sin_family != AF_INET || peer.sin_port != htons(t->remote_port) ||
            (!t->probe_replies && peer.sin_addr.s_addr != htonl(t->active_ip))) {
            t->dropped++; continue;
        }
        t->last_sender_ip = ntohl(peer.sin_addr.s_addr);
        t->received++; *error = 0; return (size_t)n;
    }
    return 0; /* Flood bound also applies to nonblocking reads with no advancing clock. */
}
