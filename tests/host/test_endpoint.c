#include "test_runner.h"
#include "umb_endpoint.h"
#include <string.h>
static umb_endpoint_input_t station(void) {
    return (umb_endpoint_input_t){.mode=UMB_ENDPOINT_STA,.configured_ip=0x0a000005,
        .local_ip=0x0a000002,.mask=0xffffff00,.port=8888};
}
static umb_endpoint_input_t ap(void) {
    umb_endpoint_input_t in = {.mode=UMB_ENDPOINT_AP,.configured_ip=0x0a000005,
        .local_ip=0xc0a80401,.mask=0xffffff00,.port=8888,.lease_count=2,
        .associated_count=2,.associations_known=true};
    in.leases[0] = (umb_endpoint_client_t){.ip=0xc0a80402,.mac={2,1,2,3,4,5}};
    in.leases[1] = (umb_endpoint_client_t){.ip=0xc0a80403,.mac={2,1,2,3,4,6}};
    memcpy(in.associated[0],in.leases[0].mac,6); memcpy(in.associated[1],in.leases[1].mac,6);
    return in;
}
TEST(test_sta_fixed_precedes_discovery_and_freezes_validated_endpoint) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=station(); umb_endpoint_probe_t p;
    ASSERT_TRUE(umb_endpoint_next(&s,&in,0,&p)); ASSERT_EQ(p.ip,in.configured_ip);
    ASSERT_EQ(p.port,8888); ASSERT_EQ(p.timeout_ms,100); ASSERT_FALSE(p.broadcast);
    ASSERT_FALSE(umb_endpoint_next(&s,&in,1,&p));
    ASSERT_TRUE(umb_endpoint_result(&s,p.token,true,p.ip,2));
    ASSERT_TRUE(s.selected); ASSERT_EQ(s.method,UMB_ENDPOINT_CONFIGURED);
    ASSERT_FALSE(umb_endpoint_next(&s,&in,2000,&p));
    in.port=9999; ASSERT_TRUE(umb_endpoint_reconcile(&s,&in,2001)); ASSERT_FALSE(s.selected);
    ASSERT_TRUE(umb_endpoint_next(&s,&in,2001,&p)); ASSERT_EQ(p.port,9999);
}
TEST(test_sta_broadcast_accepts_only_validated_local_unicast) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=station(); in.configured_ip=0;
    umb_endpoint_probe_t p;
    ASSERT_TRUE(umb_endpoint_next(&s,&in,0,&p)); ASSERT_TRUE(p.broadcast); ASSERT_EQ(p.ip,0x0a0000ff);
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,false,0x0a000005,1));
    ASSERT_FALSE(umb_endpoint_next(&s,&in,50,&p)); ASSERT_TRUE(umb_endpoint_next(&s,&in,101,&p));
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,true,0x0b000005,102));
    ASSERT_TRUE(umb_endpoint_next(&s,&in,202,&p));
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,true,0x0a0000ff,203));
    ASSERT_TRUE(umb_endpoint_next(&s,&in,303,&p));
    ASSERT_TRUE(umb_endpoint_result(&s,p.token,true,0x0a000005,304));
    ASSERT_EQ(s.method,UMB_ENDPOINT_BROADCAST);
}
TEST(test_ap_phone_first_pc_second_and_lease_invalidation) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=ap(); umb_endpoint_probe_t p;
    ASSERT_TRUE(umb_endpoint_next(&s,&in,0,&p)); ASSERT_EQ(p.ip,0xc0a80402);
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,false,0,1));
    ASSERT_TRUE(umb_endpoint_next(&s,&in,101,&p)); ASSERT_EQ(p.ip,0xc0a80403);
    ASSERT_TRUE(umb_endpoint_result(&s,p.token,true,p.ip,102)); ASSERT_EQ(s.method,UMB_ENDPOINT_DHCP);
    /* Reordered leases alone do not invalidate the agent. */
    umb_endpoint_client_t tmp=in.leases[0];in.leases[0]=in.leases[1];in.leases[1]=tmp;
    ASSERT_FALSE(umb_endpoint_reconcile(&s,&in,103)); ASSERT_TRUE(s.selected);
    in.lease_count=0; ASSERT_TRUE(umb_endpoint_reconcile(&s,&in,104)); ASSERT_FALSE(s.selected);
}
TEST(test_ap_disconnect_and_ip_reuse_reject_stale_results) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=ap(); umb_endpoint_probe_t p;
    ASSERT_TRUE(umb_endpoint_next(&s,&in,0,&p)); uint32_t old=p.token;
    in.associated_count=0; ASSERT_FALSE(umb_endpoint_reconcile(&s,&in,1));
    ASSERT_FALSE(umb_endpoint_result(&s,old,true,p.ip,2));
    ASSERT_FALSE(umb_endpoint_next(&s,&in,3,&p));
    in.associated_count=2; ASSERT_TRUE(umb_endpoint_next(&s,&in,4,&p));
    ASSERT_TRUE(umb_endpoint_result(&s,p.token,true,p.ip,5));
    in.associations_known=false; ASSERT_TRUE(umb_endpoint_reconcile(&s,&in,6));
    in.associations_known=true; ASSERT_TRUE(umb_endpoint_next(&s,&in,7,&p));
    old=p.token; in.leases[0].mac[5]=99; in.leases[1].mac[5]=99;
    ASSERT_FALSE(umb_endpoint_reconcile(&s,&in,8));
    ASSERT_FALSE(umb_endpoint_result(&s,old,true,p.ip,9)); ASSERT_FALSE(s.selected);
}
TEST(test_deadline_wrap_generation_and_mode_change) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=station(); umb_endpoint_probe_t p;
    s.serial=UINT32_MAX;
    ASSERT_TRUE(umb_endpoint_next(&s,&in,UINT32_MAX-49,&p)); ASSERT_EQ(p.token,1);
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,true,p.ip,50)); /* exact deadline */
    ASSERT_FALSE(umb_endpoint_next(&s,&in,149,&p)); ASSERT_TRUE(umb_endpoint_next(&s,&in,150,&p));
    uint32_t old=p.token; in.mode=UMB_ENDPOINT_OFF;
    ASSERT_FALSE(umb_endpoint_reconcile(&s,&in,151));
    ASSERT_FALSE(umb_endpoint_result(&s,old,true,p.ip,152));
    ASSERT_FALSE(umb_endpoint_next(&s,&in,153,&p));
    in=station(); ASSERT_TRUE(umb_endpoint_next(&s,&in,154,&p)); old=p.token;
    ASSERT_FALSE(umb_endpoint_next(&s,&in,254,&p)); /* expire pending */
    ASSERT_FALSE(umb_endpoint_result(&s,old,true,p.ip,255));
    ASSERT_TRUE(umb_endpoint_next(&s,&in,354,&p)); ASSERT_TRUE(p.token!=old);
    ASSERT_FALSE(umb_endpoint_result(&s,old,true,p.ip,355));
    ASSERT_FALSE(umb_endpoint_result(&s,p.token,true,p.ip+1,355));
}
TEST(test_ap_fair_retries_and_session_clear) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=ap(); umb_endpoint_probe_t p;
    for(unsigned i=0;i<20;i++) {
        ASSERT_TRUE(umb_endpoint_next(&s,&in,i*101,&p)); ASSERT_EQ(p.ip,0xc0a80402+(i%2));
        ASSERT_FALSE(umb_endpoint_result(&s,p.token,false,0,i*101+1));
    }
    ASSERT_TRUE(umb_endpoint_next(&s,&in,2020,&p)); ASSERT_TRUE(umb_endpoint_result(&s,p.token,true,p.ip,2021));
    umb_endpoint_clear(&s,2022); ASSERT_TRUE(umb_endpoint_next(&s,&in,2022,&p)); ASSERT_EQ(p.ip,0xc0a80403);
}
TEST(test_invalid_network_and_candidates_never_probe) {
    umb_endpoint_t s=umb_endpoint_init(); umb_endpoint_input_t in=station(); umb_endpoint_probe_t p;
    uint32_t invalid[]={0x7f000001,0xe0000001,UINT32_MAX,in.local_ip,1};
    for(unsigned i=0;i<sizeof invalid/sizeof *invalid;i++) {
        in.configured_ip=invalid[i]; ASSERT_FALSE(umb_endpoint_next(&s,&in,i,&p));
    }
    in.configured_ip=0; in.mask=0; ASSERT_FALSE(umb_endpoint_next(&s,&in,10,&p));
    in.mask=0xff00ff00; ASSERT_FALSE(umb_endpoint_next(&s,&in,11,&p));
    in=station(); in.local_ip=0x0a000000; ASSERT_FALSE(umb_endpoint_next(&s,&in,12,&p));
    in.local_ip=0x0a0000ff; ASSERT_FALSE(umb_endpoint_next(&s,&in,12,&p));
    in=ap(); in.associated_count=SIZE_MAX; in.lease_count=SIZE_MAX;
    in.leases[0].ip=in.local_ip; in.leases[1].ip=0xc0a804ff;
    ASSERT_FALSE(umb_endpoint_next(&s,&in,12,&p));
    in.port=0; ASSERT_FALSE(umb_endpoint_next(&s,&in,13,&p));
}
int main(void) {
    RUN_TEST(test_sta_fixed_precedes_discovery_and_freezes_validated_endpoint);
    RUN_TEST(test_sta_broadcast_accepts_only_validated_local_unicast);
    RUN_TEST(test_ap_phone_first_pc_second_and_lease_invalidation);
    RUN_TEST(test_ap_disconnect_and_ip_reuse_reject_stale_results);
    RUN_TEST(test_deadline_wrap_generation_and_mode_change);
    RUN_TEST(test_ap_fair_retries_and_session_clear);
    RUN_TEST(test_invalid_network_and_candidates_never_probe);
    TEST_SUMMARY();
}
