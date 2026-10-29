#include "test_runner.h"
#include "umb_dhcp.h"
#include <string.h>
#define SERVER 0xc0a80401u
static uint8_t packet[UMB_DHCP_PACKET_CAP], reply[UMB_DHCP_REPLY_CAP];
static size_t build(uint8_t type, uint8_t mac_last, uint8_t wanted) {
    memset(packet, 0, sizeof packet);
    packet[0] = packet[1] = 1; packet[2] = 6; packet[7] = 42;
    packet[28] = 2; packet[33] = mac_last;
    packet[236] = 0x63; packet[237] = 0x82; packet[238] = 0x53; packet[239] = 0x63;
    size_t n = 240; packet[n++] = 0; /* PAD must not consume a length byte */
    packet[n++] = 53; packet[n++] = 1; packet[n++] = type;
    if (wanted) {
        packet[n++] = 50; packet[n++] = 4;
        packet[n++] = 192; packet[n++] = 168; packet[n++] = 4; packet[n++] = wanted;
    }
    packet[n++] = 255; return n;
}
TEST(test_dhcp_offer_ack_and_renewal) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n = build(1, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    ASSERT_EQ(out, 300); ASSERT_EQ(reply[19], 2); ASSERT_EQ(reply[7], 42);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 0, 0), 0); /* offer alone is not agent candidate */
    n = build(3, 1, 2); ASSERT_EQ(umb_dhcp_process(&s, packet, n, 100, reply, sizeof reply, &out), 1);
    ASSERT_EQ(reply[242], 5); ASSERT_EQ(umb_dhcp_bound_ip(&s, 0, 100), SERVER + 1);
    n = build(3, 1, 0); packet[12] = 192; packet[13] = 168; packet[14] = 4; packet[15] = 2;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 1000, reply, sizeof reply, &out), 1);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 0, 86400099), SERVER + 1);
}
TEST(test_dhcp_two_clients_collision_release_and_expiry) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n = build(3, 1, 2);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    n = build(3, 2, 2);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    ASSERT_EQ(reply[242], 6); /* cannot steal a bound lease */
    n = build(1, 2, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1); ASSERT_EQ(reply[19], 3);
    n = build(7, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 0);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 0, 0), 0);
    umb_dhcp_expire(&s, 30000); ASSERT_FALSE(s.leases[1].used);
}
TEST(test_dhcp_malformed_options_never_modify_leases) {
    umb_dhcp_t s = umb_dhcp_init(SERVER), before = s; size_t out;
    size_t n = build(1, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, 239, 0, reply, sizeof reply, &out), -1);
    packet[n - 1] = 12; /* option without length */
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
    packet[n++] = 255; /* length overruns packet */
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
    n = build(1, 1, 0); packet[242] = 2; /* malformed type length */
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
    n = build(1, 1, 0); packet[n - 1] = 53; packet[n++] = 1; packet[n++] = 1; packet[n++] = 255;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
    ASSERT_TRUE(memcmp(&s, &before, sizeof s) == 0);
}
TEST(test_dhcp_pool_bounds_and_wrap) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n;
    for (int i = 0; i < UMB_DHCP_LEASES; i++) {
        n = build(3, (uint8_t)(i + 1), (uint8_t)(i + 2));
        ASSERT_EQ(umb_dhcp_process(&s, packet, n, UINT32_MAX - 100, reply, sizeof reply, &out), 1);
    }
    n = build(1, 99, 0); ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 0);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 8, 0), 0);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 0, 0), SERVER + 1);
    umb_dhcp_expire(&s, 86400000); ASSERT_FALSE(s.leases[0].used);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 86400000, reply, sizeof reply, &out), 1);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, 299, &out), -2);
}
TEST(test_dhcp_server_selection_subnet_and_unknown_options) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n = build(1, 1, 0);
    packet[n - 1] = 54; packet[n++] = 4;
    packet[n++] = 192; packet[n++] = 168; packet[n++] = 4; packet[n++] = 99; packet[n++] = 255;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 0);
    n = build(3, 1, 2); packet[247] = 5;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    ASSERT_EQ(reply[242], 6);
    n = build(8, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 0);
    n = build(1, 1, 0); packet[n - 1] = 12; packet[n++] = 0; packet[n++] = 255;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    n = build(3, 1, 3);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    ASSERT_EQ(umb_dhcp_bound_ip(&s, 1, 0), SERVER + 2);
}
TEST(test_dhcp_option_lengths_and_deterministic_packet_mutations) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n = build(1, 1, 0);
    for (size_t length = 0; length < n; length++)
        ASSERT_TRUE(umb_dhcp_process(&s, packet, length, 0, reply, sizeof reply, &out) < 0);
    uint32_t random = 0x42deadbeu;
    struct { uint8_t before[16], data[300], after[16]; } guarded;
    for (unsigned trial = 0; trial < 10000; trial++) {
        n = build((uint8_t)(1 + trial % 8), (uint8_t)(trial % 200 + 1), 2);
        random ^= random << 13; random ^= random >> 17; random ^= random << 5;
        size_t length = random % 601u;
        for (size_t i = 240; i < sizeof packet; i++) {
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            packet[i] = (uint8_t)random;
        }
        memset(&guarded, 0xa5, sizeof guarded);
        int rc = umb_dhcp_process(&s, packet, length, trial, guarded.data,
                                   trial % 2 ? 300 : 299, &out);
        ASSERT_TRUE(rc >= -2 && rc <= 1); ASSERT_TRUE(out <= 300);
        for (unsigned i = 0; i < 16; i++) {
            ASSERT_EQ(guarded.before[i], 0xa5); ASSERT_EQ(guarded.after[i], 0xa5);
        }
    }
}

TEST(test_dhcp_decline_quarantines_collision_and_rejects_bad_mac) {
    umb_dhcp_t s = umb_dhcp_init(SERVER); size_t out, n = build(1, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    n = build(4, 1, 2);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 0);
    n = build(1, 1, 0);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), 1);
    ASSERT_EQ(reply[19], 3); /* .2 quarantined, offer .3 */
    umb_dhcp_expire(&s, 60000); ASSERT_FALSE(s.leases[0].used);
    memset(packet + 28, 0, 6);
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
    packet[28] = 0xff;
    ASSERT_EQ(umb_dhcp_process(&s, packet, n, 0, reply, sizeof reply, &out), -1);
}

int main(void) {
    RUN_TEST(test_dhcp_decline_quarantines_collision_and_rejects_bad_mac);
    RUN_TEST(test_dhcp_server_selection_subnet_and_unknown_options);
    RUN_TEST(test_dhcp_option_lengths_and_deterministic_packet_mutations);
    RUN_TEST(test_dhcp_offer_ack_and_renewal);
    RUN_TEST(test_dhcp_two_clients_collision_release_and_expiry);
    RUN_TEST(test_dhcp_malformed_options_never_modify_leases);
    RUN_TEST(test_dhcp_pool_bounds_and_wrap);
    TEST_SUMMARY();
}
