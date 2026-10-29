/* Production callbacks, deterministic fake sockets; never opens a host/device socket. */
#include "test_runner.h"
#include "hal_udp_transport.h"
#include "hal_time.h"
#include "lwip/sockets.h"
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
static uint32_t now_ms, receive_step;
static int fail_open, fail_bind, fail_close, fail_option, short_send;
static unsigned closes, receives, options;
static int last_flags, last_timeout, first_timeout;
static struct { int n, flags; uint32_t ip; uint16_t port; } packets[32];
static unsigned packet_count;
static int next_socket, last_closed;
static bool ping_accept;
static hal_udp_transport_t state;
static struct uxrCustomTransport transport;
uint32_t hal_now_ms(void) { return now_ms; }
int umb_test_socket(int d, int t, int p) { (void)d; (void)t; (void)p; return fail_open ? -1 : next_socket++; }
int umb_test_bind(int fd, const struct sockaddr *addr, socklen_t n) { (void)fd; (void)addr; (void)n; return fail_bind ? -1 : 0; }
int umb_test_close(int fd) { last_closed = fd; closes++; if (fail_close) { errno = EIO; return -1; } return 0; }
int umb_test_sendto(int fd, const void *b, size_t n, int flags, const struct sockaddr *addr, socklen_t size) {
    (void)fd; (void)b; (void)flags; (void)addr; (void)size;
    return short_send ? (int)n - 1 : (int)n;
}
int umb_test_setsockopt(int fd, int level, int opt, const void *value, socklen_t n) {
    (void)fd; (void)level; (void)n;
    if (opt == SO_BROADCAST) return fail_option ? -1 : 0;
    const struct timeval *tv = value;
    if (opt == SO_SNDTIMEO) return fail_option ? -1 : 0;
    last_timeout = (int)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
    if (!options++) first_timeout = last_timeout;
    return fail_option ? -1 : 0;
}
ssize_t umb_test_recvmsg(int fd, struct msghdr *msg, int flags) {
    (void)fd; last_flags = flags; now_ms += receive_step;
    unsigned i = receives++;
    if (i >= packet_count) { errno = EAGAIN; return -1; }
    if (packets[i].n < 0) { errno = EIO; return -1; }
    struct sockaddr_in *peer = msg->msg_name;
    peer->sin_family = AF_INET; peer->sin_addr.s_addr = htonl(packets[i].ip);
    peer->sin_port = htons(packets[i].port); msg->msg_flags = packets[i].flags;
    if (packets[i].n > 0) memset(msg->msg_iov[0].iov_base, 0x42,
        (size_t)packets[i].n < msg->msg_iov[0].iov_len ? (size_t)packets[i].n : msg->msg_iov[0].iov_len);
    return packets[i].n;
}
void uxr_set_custom_transport_callbacks(uxrCustomTransport *t, bool framing, open_custom_func open,
                                       close_custom_func close, write_custom_func write, read_custom_func read) {
    t->framing = framing; t->open = open; t->close = close; t->write = write; t->read = read;
}
bool uxr_init_custom_transport(uxrCustomTransport *t, void *args) {
    t->args = args; t->comm.instance = t; return t->open(t);
}
bool uxr_close_custom_transport(uxrCustomTransport *t) { return t->close(t); }
bool uxr_ping_agent_attempts(uxrCommunication *comm, const int timeout, const uint8_t attempts) {
    (void)attempts; uxrCustomTransport *t = comm->instance; uint8_t buf[16], error;
    size_t n = t->read(t, buf, sizeof buf, timeout, &error);
    return ping_accept && n > 0 && error == 0; /* engine result is controlled, not a fake XRCE parser */
}
static void setup(void) {
    now_ms = receive_step = closes = receives = options = packet_count = 0;
    fail_open = fail_bind = fail_close = fail_option = short_send = last_flags = last_timeout = first_timeout = 0;
    next_socket = 7; last_closed = -1; ping_accept = true;
    memset(packets, 0, sizeof packets);
    hal_udp_transport_init(&state, 0xc0a80402u, 8888);
    memset(&transport, 0, sizeof transport); transport.args = &state;
}
static void packet(unsigned i, int n, int flags, uint32_t ip, uint16_t port) {
    packets[i].n = n; packets[i].flags = flags; packets[i].ip = ip; packets[i].port = port;
    if (packet_count <= i) packet_count = i + 1;
}
TEST(test_udp_open_failures_and_idempotent_close) {
    setup(); fail_open = true; ASSERT_FALSE(hal_udp_transport_open(&transport));
    ASSERT_EQ(state.socket, -1);
    fail_open = false; fail_bind = true;
    ASSERT_FALSE(hal_udp_transport_open(&transport)); ASSERT_EQ(closes, 1); ASSERT_EQ(state.socket, -1);
    fail_bind = false; ASSERT_TRUE(hal_udp_transport_open(&transport));
    ASSERT_TRUE(hal_udp_transport_open(&transport));
    ASSERT_TRUE(hal_udp_transport_close(&transport)); ASSERT_EQ(closes, 2);
    ASSERT_TRUE(hal_udp_transport_close(&transport)); ASSERT_EQ(closes, 2);
}
TEST(test_udp_write_requires_complete_datagram) {
    setup(); uint8_t buf[4] = {0}, error = 99;
    ASSERT_EQ(hal_udp_transport_write(&transport, buf, sizeof buf, &error), 0);
    ASSERT_TRUE(hal_udp_transport_open(&transport));
    ASSERT_EQ(hal_udp_transport_write(&transport, buf, sizeof buf, &error), 4); ASSERT_EQ(error, 0);
    short_send = true;
    ASSERT_EQ(hal_udp_transport_write(&transport, buf, sizeof buf, &error), 0); ASSERT_EQ(error, 1);
}
TEST(test_udp_nonblocking_and_peer_filter) {
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport));
    packet(0, 2, 0, 0xc0a80403u, 8888); packet(1, 3, 0, state.remote_ip, 8890);
    packet(2, 4, 0, state.remote_ip, state.remote_port);
    uint8_t buf[8], error = 99;
    ASSERT_EQ(hal_udp_transport_read(&transport, buf, sizeof buf, 0, &error), 4);
    ASSERT_TRUE(last_flags & MSG_DONTWAIT); ASSERT_EQ(options, 0); ASSERT_EQ(error, 0); ASSERT_EQ(state.dropped, 2);
}
TEST(test_udp_truncation_and_flood_do_not_extend_deadline) {
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport)); receive_step = 2;
    for (unsigned i = 0; i < 20; i++) packet(i, 2000, MSG_TRUNC, state.remote_ip, state.remote_port);
    uint8_t buf[8], error = 0;
    ASSERT_EQ(hal_udp_transport_read(&transport, buf, sizeof buf, 5, &error), 0);
    ASSERT_EQ(error, 1); ASSERT_EQ(receives, 3); ASSERT_EQ(first_timeout, 5); ASSERT_EQ(last_timeout, 1);
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport));
    for (unsigned i = 0; i < 20; i++) packet(i, 1, 0, 0xc0a80403u, 8888);
    ASSERT_EQ(hal_udp_transport_read(&transport, buf, sizeof buf, 0, &error), 0); ASSERT_EQ(receives, 16);
}
TEST(test_udp_option_failure_and_timeout_wrap) {
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport)); fail_option = true;
    uint8_t buf[8], error = 0;
    ASSERT_EQ(hal_udp_transport_read(&transport, buf, sizeof buf, 20, &error), 0); ASSERT_EQ(receives, 0);
    fail_option = false; now_ms = UINT32_MAX - 2; receive_step = 2;
    packet(0, 2, 0, 0xc0a80403u, 8888); packet(1, 3, 0, state.remote_ip, 8888);
    ASSERT_EQ(hal_udp_transport_read(&transport, buf, sizeof buf, 5, &error), 3); ASSERT_EQ(error, 0);
}
TEST(test_udp_endpoint_is_fixed_until_close_and_option_failure_closes) {
    setup(); fail_option = true;
    ASSERT_FALSE(hal_udp_transport_open(&transport)); ASSERT_EQ(state.socket, -1); ASSERT_EQ(closes, 1);
    fail_option = false; ASSERT_TRUE(hal_udp_transport_open(&transport));
    uint8_t bytes[4] = {0}, error;
    state.remote_ip++;
    ASSERT_FALSE(hal_udp_transport_open(&transport));
    ASSERT_EQ(hal_udp_transport_write(&transport, bytes, sizeof bytes, &error), 0);
    ASSERT_EQ(hal_udp_transport_read(&transport, bytes, sizeof bytes, 0, &error), 0);
    ASSERT_EQ(receives, 0);
    ASSERT_TRUE(hal_udp_transport_close(&transport));
    ASSERT_TRUE(hal_udp_transport_open(&transport));
    ASSERT_EQ(hal_udp_transport_write(&transport, bytes, sizeof bytes, &error), sizeof bytes);
}

TEST(test_udp_probe_is_private_and_requires_ping_validation) {
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport)); int original = state.socket;
    uint32_t found = 42;
    packet(0, 4, 0, 0xc0a80403u, 8888); ping_accept = false;
    ASSERT_FALSE(hal_udp_probe(UINT32_MAX, 8888, 20, true, &found)); ASSERT_EQ(found, 42);
    ASSERT_EQ(state.socket, original); ASSERT_TRUE(last_closed != original);
    receives = 0; ping_accept = true;
    ASSERT_TRUE(hal_udp_probe(UINT32_MAX, 8888, 20, true, &found)); ASSERT_EQ(found, 0xc0a80403u);
    ASSERT_EQ(state.remote_ip, 0xc0a80402u); ASSERT_EQ(state.socket, original);
    ASSERT_FALSE(hal_udp_probe(0, 8888, 20, false, &found));
    ASSERT_FALSE(hal_udp_probe(state.remote_ip, 8888, 1001, false, &found));
}

TEST(test_udp_failed_close_retains_ownership_and_probe_retries) {
    setup(); ASSERT_TRUE(hal_udp_transport_open(&transport)); int fd = state.socket;
    fail_close = true;
    ASSERT_FALSE(hal_udp_transport_close(&transport)); ASSERT_EQ(state.socket, fd);
    fail_close = false;
    ASSERT_TRUE(hal_udp_transport_close(&transport)); ASSERT_EQ(state.socket, -1);
    packet(0, 4, 0, 0xc0a80403u, 8888);
    uint32_t found = 42; fail_close = true;
    ASSERT_FALSE(hal_udp_probe(UINT32_MAX, 8888, 20, true, &found)); ASSERT_EQ(found, 42);
    int probe_fd = last_closed;
    ASSERT_FALSE(hal_udp_probe(UINT32_MAX, 8888, 20, true, &found)); ASSERT_EQ(last_closed, probe_fd);
    fail_close = false; ASSERT_TRUE(hal_udp_probe_close());
}

int main(void) {
    RUN_TEST(test_udp_failed_close_retains_ownership_and_probe_retries);
    RUN_TEST(test_udp_probe_is_private_and_requires_ping_validation);
    RUN_TEST(test_udp_endpoint_is_fixed_until_close_and_option_failure_closes);
    RUN_TEST(test_udp_open_failures_and_idempotent_close);
    RUN_TEST(test_udp_write_requires_complete_datagram);
    RUN_TEST(test_udp_nonblocking_and_peer_filter);
    RUN_TEST(test_udp_truncation_and_flood_do_not_extend_deadline);
    RUN_TEST(test_udp_option_failure_and_timeout_wrap);
    TEST_SUMMARY();
}
