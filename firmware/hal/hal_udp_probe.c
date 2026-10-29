#include "hal_udp_transport.h"
#include "lwip/sockets.h"
#include <uxr/client/util/ping.h>
/* Separate retained context: a failed close must remain owned for a later retry. */
static hal_udp_transport_t state = {.socket = -1};
static uxrCustomTransport probe;
bool hal_udp_probe_close(void) {
    return state.socket < 0 || hal_udp_transport_close(&probe);
}
bool hal_udp_probe(uint32_t destination, uint16_t port, unsigned timeout,
                   bool broadcast, uint32_t *found_ip) {
    if (!destination || !port || !timeout || timeout > 1000 || !found_ip) return false;
    if (!hal_udp_probe_close()) return false;
    hal_udp_transport_init(&state, destination, port); state.probe_replies = broadcast;
    uxr_set_custom_transport_callbacks(&probe, false, hal_udp_transport_open, hal_udp_transport_close,
                                       hal_udp_transport_write, hal_udp_transport_read);
    if (!uxr_init_custom_transport(&probe, &state)) return false;
    bool ok = true;
    if (broadcast) {
        int enable = 1;
        ok = lwip_setsockopt(state.socket, SOL_SOCKET, SO_BROADCAST, &enable, sizeof enable) == 0;
    }
    if (ok) ok = uxr_ping_agent_attempts(&probe.comm, (int)timeout, 1);
    bool closed = uxr_close_custom_transport(&probe);
    if (!ok || !closed || !state.last_sender_ip) return false;
    *found_ip = state.last_sender_ip;
    return true;
}
