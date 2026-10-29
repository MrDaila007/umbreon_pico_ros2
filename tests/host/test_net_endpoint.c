#include "test_runner.h"
#include "net_endpoint.h"
#include "hal_dhcp_server.h"
#include <string.h>
static uint32_t now, probe_ip, reply_ip;
static uint16_t probe_port;
static bool probe_broadcast, probe_ok, disconnect_during_probe, close_ok;
static unsigned calls, requested_timeout;
static hal_wifi_status_t wifi;
static hal_dhcp_client_t clients[UMB_DHCP_LEASES];
static size_t lease_count;
static int associated_count;
static uint8_t macs[HAL_WIFI_AP_CLIENTS][6];
uint32_t hal_now_ms(void) { return now; }
void hal_wifi_status(hal_wifi_status_t *out) { *out=wifi; }
size_t hal_dhcp_server_clients(hal_dhcp_client_t out[UMB_DHCP_LEASES]) {
    memcpy(out,clients,sizeof clients); return lease_count;
}
int hal_wifi_ap_clients(uint8_t out[HAL_WIFI_AP_CLIENTS][6]) {
    memcpy(out,macs,sizeof macs); return associated_count;
}
bool hal_udp_probe(uint32_t ip,uint16_t port,unsigned timeout,bool broadcast,uint32_t *found) {
    calls++; probe_ip=ip;probe_port=port;probe_broadcast=broadcast;
    requested_timeout=timeout;
    now+=20;
    if(disconnect_during_probe) associated_count=0;
    if(probe_ok) *found=reply_ip ? reply_ip : ip;
    return probe_ok;
}
bool hal_udp_probe_close(void) { return close_ok; }
static void reset(void) {
    now=0;probe_ip=0;reply_ip=0;calls=0;lease_count=0;associated_count=0;
    probe_ok=false;disconnect_during_probe=false;close_ok=true;
    memset(&wifi,0,sizeof wifi);memset(clients,0,sizeof clients);memset(macs,0,sizeof macs);
}
static void setup_ap(void) {
    wifi.ap_up=true;wifi.ap_ip=0xc0a80401;lease_count=2;associated_count=2;
    clients[0]=(hal_dhcp_client_t){.ip=0xc0a80402,.mac={2,1,2,3,4,5}};
    clients[1]=(hal_dhcp_client_t){.ip=0xc0a80403,.mac={2,1,2,3,4,6}};
    memcpy(macs[0],clients[0].mac,6);memcpy(macs[1],clients[1].mac,6);
}
TEST(test_bridge_sta_fixed_and_subnet_broadcast) {
    reset(); net_endpoint_t s;net_endpoint_init(&s);
    wifi.sta_up=true;wifi.sta_ip=0x0a000002;wifi.sta_mask=0xffffff00;
    ASSERT_FALSE(net_endpoint_refresh(&s,UMB_ENDPOINT_STA,0x0a000005,7777));
    probe_ok=true; ASSERT_TRUE(net_endpoint_probe_once(&s));
    ASSERT_EQ(probe_ip,0x0a000005);ASSERT_EQ(probe_port,7777);ASSERT_EQ(requested_timeout,100);ASSERT_FALSE(probe_broadcast);
    ASSERT_FALSE(net_endpoint_probe_once(&s));ASSERT_EQ(calls,1);
    ASSERT_TRUE(net_endpoint_refresh(&s,UMB_ENDPOINT_STA,0,8888));
    reply_ip=0x0a000006;ASSERT_TRUE(net_endpoint_probe_once(&s));
    ASSERT_EQ(probe_ip,0x0a0000ff);ASSERT_TRUE(probe_broadcast);ASSERT_EQ(s.policy.selected_ip,reply_ip);
    wifi.sta_up=false;ASSERT_TRUE(net_endpoint_refresh(&s,UMB_ENDPOINT_STA,0,8888));
    ASSERT_FALSE(net_endpoint_probe_once(&s));ASSERT_EQ(calls,2);
}
TEST(test_bridge_ap_phone_then_pc_and_association_loss) {
    reset();setup_ap();net_endpoint_t s;net_endpoint_init(&s);
    ASSERT_FALSE(net_endpoint_refresh(&s,UMB_ENDPOINT_AP,0x0a000005,8888));
    ASSERT_FALSE(net_endpoint_probe_once(&s));ASSERT_EQ(probe_ip,0xc0a80402);
    now+=100;probe_ok=true;ASSERT_TRUE(net_endpoint_probe_once(&s));
    ASSERT_EQ(probe_ip,0xc0a80403);ASSERT_FALSE(probe_broadcast);
    associated_count=-1;ASSERT_TRUE(net_endpoint_refresh(&s,UMB_ENDPOINT_AP,0x0a000005,8888));
    ASSERT_FALSE(net_endpoint_probe_once(&s));ASSERT_EQ(calls,2);
    ASSERT_EQ(s.probes,2);ASSERT_EQ(s.failures,1);
}
TEST(test_bridge_rechecks_disconnect_and_retains_failed_close) {
    reset();setup_ap();net_endpoint_t s;net_endpoint_init(&s);
    ASSERT_FALSE(net_endpoint_refresh(&s,UMB_ENDPOINT_AP,0,8888));
    probe_ok=true;disconnect_during_probe=true;
    ASSERT_FALSE(net_endpoint_probe_once(&s));ASSERT_FALSE(s.policy.selected);
    disconnect_during_probe=false;associated_count=2;now+=100;
    ASSERT_FALSE(net_endpoint_refresh(&s,UMB_ENDPOINT_AP,0,8888));
    ASSERT_TRUE(net_endpoint_probe_once(&s));
    close_ok=false;ASSERT_FALSE(net_endpoint_close(&s));ASSERT_TRUE(s.policy.selected);
    close_ok=true;ASSERT_TRUE(net_endpoint_close(&s));ASSERT_FALSE(s.policy.selected);
    ASSERT_FALSE(net_endpoint_probe_once(&s));
}
int main(void) {
    RUN_TEST(test_bridge_sta_fixed_and_subnet_broadcast);
    RUN_TEST(test_bridge_ap_phone_then_pc_and_association_loss);
    RUN_TEST(test_bridge_rechecks_disconnect_and_retains_failed_close);
    TEST_SUMMARY();
}
