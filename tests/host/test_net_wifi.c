#include "test_runner.h"
#include "net_wifi.h"
#include <string.h>
static uint32_t now;
static hal_wifi_status_t status;
static char calls[64];
static unsigned count;
static bool sta_ok, ap_ok, mac_ok;
uint32_t hal_now_ms(void) { return now; }
void hal_wifi_status(hal_wifi_status_t *out) { *out=status; }
bool hal_wifi_start_sta(const char *ssid,const char *password) {
    (void)ssid;(void)password;calls[count++]='S';return sta_ok;
}
void hal_wifi_stop_sta(void) { calls[count++]='s';status.sta_up=false; }
bool hal_wifi_start_ap(const char *ssid,const char *password) {
    calls[count++]='A';return ap_ok && !strcmp(ssid,"umbreon-ABCD") && !strcmp(password,"host-only-pass");
}
void hal_wifi_stop_ap(void) { calls[count++]='a';status.ap_up=false; }
bool hal_wifi_mac(uint8_t mac[6]) {
    const uint8_t value[6]={2,1,2,3,0xab,0xcd};memcpy(mac,value,6);return mac_ok;
}
static umb_settings_t reset(void) {
    now=0;count=0;memset(calls,0,sizeof calls);memset(&status,0,sizeof status);
    sta_ok=true;ap_ok=true;mac_ok=true;
    umb_settings_t settings=umb_settings_defaults();settings.wifi_mode=UMB_WIFI_AUTO;
    strcpy(settings.wifi_ssid,"host-network");strcpy(settings.wifi_pass,"host-only-pass");
    strcpy(settings.ap_pass,"host-only-pass");settings.sta_timeout_s=1;
    return settings;
}
TEST(test_wifi_async_sta_to_ap_actions_and_retry) {
    umb_settings_t settings=reset();net_wifi_t s;net_wifi_init(&s);
    net_wifi_result_t r=net_wifi_step(&s,&settings,false);ASSERT_FALSE(r.loss);ASSERT_EQ(count,0);
    ASSERT_EQ(s.policy.stage,UMB_WIFI_JOINING);
    r=net_wifi_step(&s,&settings,false);ASSERT_FALSE(r.action_failed);ASSERT_EQ(calls[0],'S');
    now=999;r=net_wifi_step(&s,&settings,false);ASSERT_EQ(count,1);ASSERT_EQ(s.policy.stage,UMB_WIFI_JOINING);
    now=1000;r=net_wifi_step(&s,&settings,false);ASSERT_EQ(count,1);
    ASSERT_EQ(s.deferred_actions,UMB_NET_STOP_STA|UMB_NET_START_AP);
    now=1001;r=net_wifi_step(&s,&settings,false);ASSERT_FALSE(r.action_failed);
    ASSERT_EQ(calls[1],'s');ASSERT_EQ(calls[2],'A');
    status.ap_up=true;now=1002;r=net_wifi_step(&s,&settings,false);ASSERT_EQ(s.policy.stage,UMB_WIFI_ON_AP);
    now=61002;r=net_wifi_step(&s,&settings,true);ASSERT_EQ(s.policy.stage,UMB_WIFI_ON_AP);ASSERT_EQ(count,3);
    r=net_wifi_step(&s,&settings,false);ASSERT_TRUE(r.loss);ASSERT_EQ(count,3);
    r=net_wifi_step(&s,&settings,false);ASSERT_EQ(calls[3],'a');ASSERT_EQ(calls[4],'S');
}
TEST(test_wifi_loss_precedes_radio_change_and_teardown_wait) {
    umb_settings_t settings=reset();net_wifi_t s;net_wifi_init(&s);
    (void)net_wifi_step(&s,&settings,false);(void)net_wifi_step(&s,&settings,false);
    status.sta_up=true;(void)net_wifi_step(&s,&settings,true);
    ASSERT_EQ(s.policy.stage,UMB_WIFI_ON_STA);status.sta_up=false;
    net_wifi_result_t r=net_wifi_step(&s,&settings,true);
    ASSERT_TRUE(r.loss);ASSERT_TRUE(r.needs_teardown);ASSERT_EQ(count,1);
    now=5000;r=net_wifi_step(&s,&settings,true);ASSERT_TRUE(r.needs_teardown);ASSERT_EQ(count,1);
    now=6000;r=net_wifi_step(&s,&settings,false);ASSERT_FALSE(r.action_failed);
    ASSERT_EQ(calls[1],'s');ASSERT_EQ(calls[2],'S');ASSERT_EQ(s.policy.deadline_ms,7000);
    now=6500;(void)net_wifi_step(&s,&settings,false);ASSERT_EQ(s.policy.stage,UMB_WIFI_JOINING);
}
TEST(test_wifi_missing_ap_password_stays_closed_with_backoff) {
    umb_settings_t settings=reset();settings.wifi_mode=UMB_WIFI_AP_ONLY;settings.ap_pass[0]=0;
    net_wifi_t s;net_wifi_init(&s);(void)net_wifi_step(&s,&settings,false);
    net_wifi_result_t r=net_wifi_step(&s,&settings,false);ASSERT_TRUE(r.action_failed);ASSERT_EQ(count,0);
    r=net_wifi_step(&s,&settings,false);ASSERT_EQ(s.policy.stage,UMB_WIFI_BACKOFF);
    r=net_wifi_step(&s,&settings,false);ASSERT_EQ(calls[0],'a');
    for(now=1;now<1000;now++) (void)net_wifi_step(&s,&settings,false);
    ASSERT_EQ(count,1);ASSERT_EQ(s.policy.stage,UMB_WIFI_BACKOFF);
    (void)net_wifi_step(&s,&settings,false);r=net_wifi_step(&s,&settings,false);
    ASSERT_TRUE(r.action_failed);ASSERT_EQ(count,1);
}
TEST(test_wifi_failed_join_and_deferred_owner_mode_change) {
    umb_settings_t settings=reset();sta_ok=false;net_wifi_t s;net_wifi_init(&s);
    (void)net_wifi_step(&s,&settings,false);
    net_wifi_result_t r=net_wifi_step(&s,&settings,false);ASSERT_TRUE(r.action_failed);
    (void)net_wifi_step(&s,&settings,false);ASSERT_EQ(s.policy.stage,UMB_WIFI_STARTING_AP);
    (void)net_wifi_step(&s,&settings,false);status.ap_up=true;
    (void)net_wifi_step(&s,&settings,true);ASSERT_EQ(s.policy.stage,UMB_WIFI_ON_AP);
    settings.wifi_mode=UMB_WIFI_STA_ONLY;unsigned before=count;
    r=net_wifi_step(&s,&settings,true);ASSERT_EQ(count,before);ASSERT_EQ(s.policy.stage,UMB_WIFI_ON_AP);
    r=net_wifi_step(&s,&settings,false);ASSERT_TRUE(r.loss);ASSERT_EQ(count,before);
    sta_ok=true;r=net_wifi_step(&s,&settings,false);ASSERT_EQ(calls[before],'a');ASSERT_EQ(calls[before+1],'S');
}
int main(void) {
    RUN_TEST(test_wifi_async_sta_to_ap_actions_and_retry);
    RUN_TEST(test_wifi_loss_precedes_radio_change_and_teardown_wait);
    RUN_TEST(test_wifi_missing_ap_password_stays_closed_with_backoff);
    RUN_TEST(test_wifi_failed_join_and_deferred_owner_mode_change);
    TEST_SUMMARY();
}
