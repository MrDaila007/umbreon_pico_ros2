#include "umb_endpoint.h"
#include <string.h>
static bool due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static size_t bounded(size_t count) { return count < UMB_ENDPOINT_CLIENTS ? count : UMB_ENDPOINT_CLIENTS; }
static bool unicast(uint32_t ip) { return (ip >> 24) != 0 && (ip >> 24) != 127 && (ip >> 24) < 224; }
static bool subnet_valid(uint32_t local, uint32_t mask) {
    uint32_t hosts = ~mask;
    return unicast(local) && mask && hosts > 1 && ((hosts & (hosts + 1)) == 0) &&
        (local & hosts) != 0 && (local & hosts) != hosts;
}
static bool subnet_host(uint32_t ip, uint32_t local, uint32_t mask) {
    uint32_t hosts = ~mask;
    return subnet_valid(local, mask) && unicast(ip) && (ip & mask) == (local & mask) &&
        (ip & hosts) != 0 && (ip & hosts) != hosts && ip != local;
}
static bool eligible(const umb_endpoint_input_t *in, const umb_endpoint_client_t *c) {
    if (!in->associations_known || !subnet_host(c->ip, in->local_ip, in->mask) || (c->mac[0] & 1)) return false;
    uint8_t any = 0; for (size_t k = 0; k < 6; k++) any |= c->mac[k];
    if (!any) return false;
    for (size_t i = 0; i < bounded(in->associated_count); i++)
        if (!memcmp(c->mac, in->associated[i], 6)) return true;
    return false;
}
static bool member(const umb_endpoint_input_t *in, uint32_t ip, const uint8_t mac[6]) {
    for (size_t i = 0; i < bounded(in->lease_count); i++)
        if (in->leases[i].ip == ip && !memcmp(in->leases[i].mac, mac, 6) && eligible(in, &in->leases[i])) return true;
    return false;
}
umb_endpoint_t umb_endpoint_init(void) { return (umb_endpoint_t){0}; }
void umb_endpoint_clear(umb_endpoint_t *s, uint32_t now) {
    s->selected = false; s->selected_ip = 0; s->method = UMB_ENDPOINT_NONE;
    s->pending = false; s->pending_token = 0; s->retry_ms = now;
}
bool umb_endpoint_reconcile(umb_endpoint_t *s, const umb_endpoint_input_t *in, uint32_t now) {
    /* STA fixed IP is deliberately ignored in AP mode. */
    uint32_t configured = in->mode == UMB_ENDPOINT_STA ? in->configured_ip : 0;
    bool changed = !s->initialized || s->mode != in->mode || s->port != in->port ||
        s->local_ip != in->local_ip || s->mask != in->mask || s->configured_ip != configured;
    bool lost = false;
    if (changed) {
        lost = s->selected; umb_endpoint_clear(s, now); s->cursor = 0;
        s->initialized = true; s->mode = in->mode; s->port = in->port;
        s->local_ip = in->local_ip; s->mask = in->mask; s->configured_ip = configured;
    }
    if (s->mode == UMB_ENDPOINT_AP) {
        if (s->selected && !member(in, s->selected_ip, s->selected_mac)) {
            lost = true; umb_endpoint_clear(s, now);
        } else if (s->pending && !member(in, s->pending_ip, s->pending_mac)) {
            s->pending = false; s->pending_token = 0; s->retry_ms = now;
        }
    }
    return lost;
}
bool umb_endpoint_next(umb_endpoint_t *s, const umb_endpoint_input_t *in, uint32_t now, umb_endpoint_probe_t *probe) {
    (void)umb_endpoint_reconcile(s, in, now);
    if (s->pending && due(now, s->deadline_ms)) {
        s->pending = false; s->pending_token = 0; s->retry_ms = now + UMB_ENDPOINT_RETRY_MS;
    }
    if (s->selected || s->pending || !due(now, s->retry_ms) || !s->port ||
        !subnet_valid(s->local_ip, s->mask)) return false;
    uint32_t ip = 0; bool broadcast = false;
    umb_endpoint_method_t method = UMB_ENDPOINT_NONE;
    if (s->mode == UMB_ENDPOINT_STA) {
        if (s->configured_ip) {
            if (!unicast(s->configured_ip) || s->configured_ip == s->local_ip) return false;
            ip = s->configured_ip; method = UMB_ENDPOINT_CONFIGURED;
        } else {
            uint32_t candidate = (s->local_ip & s->mask) | ~s->mask;
            /* Require an ordinary subnet, never send to the limited broadcast. */
            if (candidate == UINT32_MAX) return false;
            ip = candidate; broadcast = true; method = UMB_ENDPOINT_BROADCAST;
        }
    } else if (s->mode == UMB_ENDPOINT_AP) {
        size_t count = bounded(in->lease_count);
        for (size_t n = 0; n < count; n++) {
            size_t i = (s->cursor + n) % count;
            if (!eligible(in, &in->leases[i])) continue;
            ip = in->leases[i].ip; memcpy(s->pending_mac, in->leases[i].mac, 6);
            s->cursor = (i + 1) % count; method = UMB_ENDPOINT_DHCP; break;
        }
        if (!ip) return false;
    } else return false;
    if (++s->serial == 0) ++s->serial;
    s->pending = true; s->pending_token = s->serial; s->pending_ip = ip;
    s->pending_method = method; s->deadline_ms = now + UMB_ENDPOINT_PROBE_MS;
    *probe = (umb_endpoint_probe_t){s->serial, ip, s->port, UMB_ENDPOINT_PROBE_MS, broadcast};
    return true;
}
bool umb_endpoint_result(umb_endpoint_t *s, uint32_t token, bool xrce_ok, uint32_t ip, uint32_t now) {
    if (!s->pending || token != s->pending_token) return false;
    s->pending = false; s->pending_token = 0; s->retry_ms = now + UMB_ENDPOINT_RETRY_MS;
    if (!xrce_ok || due(now, s->deadline_ms) || !unicast(ip)) return false;
    if (s->pending_method == UMB_ENDPOINT_BROADCAST) {
        if (!subnet_host(ip, s->local_ip, s->mask)) return false;
    } else if (ip != s->pending_ip) return false;
    s->selected = true; s->selected_ip = ip; s->method = s->pending_method;
    memcpy(s->selected_mac, s->pending_mac, 6); return true;
}
