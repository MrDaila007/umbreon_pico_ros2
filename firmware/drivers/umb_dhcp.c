/* DHCP layout/flow adapted from Raspberry Pi pico-examples access_point/dhcpserver,
 * originally MicroPython Copyright (c) 2018-2019 Damien P. George, MIT license.
 * See LICENSE-dhcp.txt. Parser/lease representation rewritten for bounded packets,
 * PAD/END options, exact lengths, full ms expiries and offered-vs-bound distinction. */
#include "umb_dhcp.h"
#include <string.h>
#define COOKIE 0x63825363u
#define LEASE_MS 86400000u
#define OFFER_MS 30000u
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void write32(uint8_t *p, uint32_t n) {
    p[0] = (uint8_t)(n >> 24); p[1] = (uint8_t)(n >> 16); p[2] = (uint8_t)(n >> 8); p[3] = (uint8_t)n;
}
static bool fresh(const umb_dhcp_lease_t *l, uint32_t now) {
    return l->used && (int32_t)(now - l->expires_ms) < 0;
}
umb_dhcp_t umb_dhcp_init(uint32_t ip) { umb_dhcp_t s = {.server_ip = ip}; return s; }
void umb_dhcp_expire(umb_dhcp_t *s, uint32_t now) {
    for (unsigned i = 0; i < UMB_DHCP_LEASES; i++)
        if (s->leases[i].used && !fresh(&s->leases[i], now)) memset(&s->leases[i], 0, sizeof s->leases[i]);
}
uint32_t umb_dhcp_bound_ip(const umb_dhcp_t *s, unsigned slot, uint32_t now) {
    if (slot >= UMB_DHCP_LEASES || !s->leases[slot].bound || !fresh(&s->leases[slot], now)) return 0;
    return (s->server_ip & 0xffffff00u) | (2u + slot);
}
int umb_dhcp_process(umb_dhcp_t *s, const uint8_t *p, size_t n, uint32_t now,
                     uint8_t *reply, size_t cap, size_t *reply_size) {
    if (!reply_size) return -1;
    *reply_size = 0;
    if (!s || !p || !reply || n < 240 || n > UMB_DHCP_PACKET_CAP ||
        p[0] != 1 || p[1] != 1 || p[2] != 6 || read32(p + 236) != COOKIE || read32(p + 24)) return -1;
    bool have_mac = false;
    for (unsigned i = 0; i < 6; i++) if (p[28 + i]) have_mac = true;
    if (!have_mac || (p[28] & 1u)) return -1;
    if (cap < UMB_DHCP_REPLY_CAP) return -2;
    uint8_t type = 0; uint32_t wanted = 0, server = 0; bool end = false;
    bool have_wanted = false, have_server = false, have_type = false;
    for (size_t i = 240; i < n;) {
        uint8_t code = p[i++];
        if (code == 0) continue;
        if (code == 255) { end = true; break; }
        if (i == n) return -1;
        size_t len = p[i++];
        if (len > n - i) return -1;
        if (code == 53) {
            if (len != 1 || have_type || !p[i]) return -1;
            type = p[i]; have_type = true;
        } else if (code == 50) {
            if (len != 4 || have_wanted) return -1;
            wanted = read32(p + i); have_wanted = true;
        } else if (code == 54) {
            if (len != 4 || have_server) return -1;
            server = read32(p + i); have_server = true;
        }
        i += len;
    }
    if (!end || !type) return -1;
    if (have_server && server != s->server_ip) return 0;
    umb_dhcp_t candidate = *s; umb_dhcp_expire(&candidate, now);
    int slot = -1;
    for (unsigned i = 0; i < UMB_DHCP_LEASES; i++)
        if (candidate.leases[i].used && !memcmp(candidate.leases[i].mac, p + 28, 6)) { slot = (int)i; break; }
    if (type == 4) { /* DECLINE: quarantine the offered/bound address for 60 seconds. */
        if (slot >= 0 && wanted == ((s->server_ip & 0xffffff00u) | (2u + (unsigned)slot))) {
            memset(candidate.leases[slot].mac, 0, 6);
            candidate.leases[slot].bound = false;
            candidate.leases[slot].expires_ms = now + 60000u;
        }
        *s = candidate; return 0;
    }
    if (type == 7) { /* RELEASE */
        if (slot >= 0 && (!read32(p + 12) || read32(p + 12) == ((s->server_ip & 0xffffff00u) | (2u + (unsigned)slot))))
            memset(&candidate.leases[slot], 0, sizeof candidate.leases[slot]);
        *s = candidate; return 0;
    }
    uint8_t response;
    if (type == 1) { /* DISCOVER */
        if (slot < 0) {
            for (unsigned i = 0; i < UMB_DHCP_LEASES; i++) if (!candidate.leases[i].used) { slot = (int)i; break; }
        }
        if (slot < 0) return 0;
        response = 2;
    } else if (type == 3) { /* REQUEST: selecting, INIT-REBOOT, or ciaddr renewal */
        if (!wanted) wanted = read32(p + 12);
        if (!wanted) return 0;
        unsigned index = (wanted & 255u) - 2u;
        bool same_subnet = (wanted & 0xffffff00u) == (s->server_ip & 0xffffff00u);
        if (same_subnet && index < UMB_DHCP_LEASES &&
            (!candidate.leases[index].used || !memcmp(candidate.leases[index].mac, p + 28, 6))) {
            if (slot >= 0 && (unsigned)slot != index) memset(&candidate.leases[slot], 0, sizeof candidate.leases[slot]);
            slot = (int)index; response = 5;
        } else { slot = -1; response = 6; }
    } else return 0;
    memset(reply, 0, UMB_DHCP_REPLY_CAP);
    reply[0] = 2; reply[1] = 1; reply[2] = 6;
    memcpy(reply + 4, p + 4, 4); /* transaction ID */
    memcpy(reply + 10, p + 10, 2); memcpy(reply + 28, p + 28, 6);
    write32(reply + 20, s->server_ip); write32(reply + 236, COOKIE);
    if (slot >= 0) {
        write32(reply + 16, (s->server_ip & 0xffffff00u) | (2u + (unsigned)slot));
        umb_dhcp_lease_t *l = &candidate.leases[slot];
        bool keep_bound = l->bound && fresh(l, now) && !memcmp(l->mac, p + 28, 6);
        memcpy(l->mac, p + 28, 6); l->used = true; l->bound = response == 5 || keep_bound;
        l->expires_ms = now + (l->bound ? LEASE_MS : OFFER_MS);
    }
    size_t at = 240;
    reply[at++] = 53; reply[at++] = 1; reply[at++] = response;
    reply[at++] = 54; reply[at++] = 4; write32(reply + at, s->server_ip); at += 4;
    if (response != 6) {
        reply[at++] = 1; reply[at++] = 4; write32(reply + at, 0xffffff00u); at += 4;
        reply[at++] = 3; reply[at++] = 4; write32(reply + at, s->server_ip); at += 4;
        reply[at++] = 51; reply[at++] = 4; write32(reply + at, LEASE_MS / 1000u); at += 4;
    }
    reply[at] = 255;
    *reply_size = UMB_DHCP_REPLY_CAP;
    *s = candidate;
    return 1;
}
