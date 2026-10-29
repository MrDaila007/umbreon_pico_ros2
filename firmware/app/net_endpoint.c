#include "net_endpoint.h"
#include "hal_dhcp_server.h"
#include "hal_udp_transport.h"
#include "hal_time.h"
#include <string.h>
_Static_assert(UMB_ENDPOINT_CLIENTS == HAL_WIFI_AP_CLIENTS, "associated snapshot capacity");
_Static_assert(UMB_ENDPOINT_CLIENTS == UMB_DHCP_LEASES, "lease snapshot capacity");
void net_endpoint_init(net_endpoint_t *s) {
    memset(s, 0, sizeof *s); s->policy = umb_endpoint_init();
}
bool net_endpoint_refresh(net_endpoint_t *s, umb_endpoint_mode_t active_mode,
                           uint32_t configured_ip, uint16_t port) {
    s->requested_mode = active_mode; s->configured_ip = configured_ip; s->port = port;
    hal_wifi_status(&s->wifi);
    umb_endpoint_input_t *in = &s->input;
    memset(in, 0, sizeof *in); in->port = port; in->configured_ip = configured_ip;
    if (active_mode == UMB_ENDPOINT_STA && s->wifi.sta_up) {
        in->mode = UMB_ENDPOINT_STA; in->local_ip = s->wifi.sta_ip; in->mask = s->wifi.sta_mask;
    } else if (active_mode == UMB_ENDPOINT_AP && s->wifi.ap_up) {
        in->mode = UMB_ENDPOINT_AP; in->local_ip = s->wifi.ap_ip; in->mask = 0xffffff00u;
        hal_dhcp_client_t leases[UMB_DHCP_LEASES];
        size_t n = hal_dhcp_server_clients(leases);
        in->lease_count = n < UMB_ENDPOINT_CLIENTS ? n : UMB_ENDPOINT_CLIENTS;
        for (size_t i = 0; i < in->lease_count; i++) {
            in->leases[i].ip = leases[i].ip;
            memcpy(in->leases[i].mac, leases[i].mac, 6);
        }
        int associated = hal_wifi_ap_clients(in->associated);
        in->associations_known = associated >= 0 && associated <= (int)UMB_ENDPOINT_CLIENTS;
        in->associated_count = in->associations_known ? (size_t)associated : 0;
    }
    return umb_endpoint_reconcile(&s->policy, in, hal_now_ms());
}
bool net_endpoint_probe_once(net_endpoint_t *s) {
    umb_endpoint_probe_t probe;
    if (!umb_endpoint_next(&s->policy, &s->input, hal_now_ms(), &probe)) return false;
    uint32_t found = 0;
    s->probes++;
    bool ok = hal_udp_probe(probe.ip, probe.port, probe.timeout_ms, probe.broadcast, &found);
    /* A client may disconnect while the bounded socket call yields. Cancel that
     * token before consuming the result; a stale lease cannot create a session. */
    (void)net_endpoint_refresh(s, s->requested_mode, s->configured_ip, s->port);
    bool selected = umb_endpoint_result(&s->policy, probe.token, ok, found, hal_now_ms());
    if (!selected) s->failures++;
    return selected;
}
bool net_endpoint_close(net_endpoint_t *s) {
    if (!hal_udp_probe_close()) return false;
    umb_endpoint_clear(&s->policy, hal_now_ms());
    memset(&s->input, 0, sizeof s->input); s->requested_mode = UMB_ENDPOINT_OFF;
    return true;
}
