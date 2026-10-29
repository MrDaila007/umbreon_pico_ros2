#include "test_runner.h"
#include "settings_apply.h"
#include "shared_state.h"
#include "hal_flash_store.h"
#include "hal_time.h"
#include "FreeRTOS.h"
#include "task.h"
static uint32_t clock_ms;
static bool give_ack, flash_ok, rejected_arm, rejected_settings;
static int flash_calls;
static umb_settings_t flashed, persisted;
#include <string.h>
uint32_t hal_now_ms(void) { return clock_ms; }
TickType_t xTaskGetTickCount(void) { return clock_ms; }
void vTaskDelay(TickType_t ticks) {
    clock_ms += ticks;
    if (give_ack) {
        shared_control_mailbox_t box; shared_take_control_mailbox(&box);
        if (box.maintenance) shared_ack_maintenance(box.maintenance_id);
    }
}
bool hal_settings_save(const umb_settings_t *s) {
    flashed = *s; flash_calls++;
    if (flash_ok) persisted = *s;
    rejected_arm = !shared_post_arm_request(SHARED_ARM_CAR);
    umb_settings_t settings; uint32_t revision = shared_settings_snapshot(&settings);
    rejected_settings = !shared_commit_settings(&settings, revision, false);
    return flash_ok;
}
bool hal_settings_persisted(umb_settings_t *out) { *out = persisted; return true; }
static void setup(void) {
    clock_ms = flash_calls = 0; give_ack = flash_ok = true;
    rejected_arm = rejected_settings = false;
    umb_settings_t s = umb_settings_defaults(); shared_init(&s); persisted = s;
}
TEST(test_save_waits_for_neutral_and_blocks_racing_producers) {
    setup();
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_OK);
    ASSERT_EQ(flash_calls, 1);
    ASSERT_TRUE(clock_ms > 0);
    ASSERT_TRUE(rejected_arm);
    ASSERT_TRUE(rejected_settings);
    ASSERT_TRUE(shared_settings_write_allowed());
}
TEST(test_save_ack_timeout_never_touches_flash_and_releases) {
    setup(); give_ack = false;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_NEUTRAL_TIMEOUT);
    ASSERT_EQ(flash_calls, 0);
    ASSERT_EQ(clock_ms, 60);
    ASSERT_TRUE(shared_settings_write_allowed());
}
TEST(test_save_failure_releases_reservation) {
    setup(); flash_ok = false;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_FLASH_FAILED);
    ASSERT_EQ(flash_calls, 1);
    ASSERT_TRUE(shared_settings_write_allowed());
}
TEST(test_save_armed_or_queued_refused_without_flash) {
    setup(); umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_BUSY);
    ASSERT_EQ(flash_calls, 0);
    setup(); shared_post_arm_request(SHARED_ARM_CAR);
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_BUSY);
    ASSERT_EQ(flash_calls, 0);
}
#define TEST_AP_PASS "SYNTH3T1CPASS234"
TEST(test_provision_persists_only_saved_settings_then_publishes) {
    setup();
    umb_settings_t edit; uint32_t rev = shared_settings_snapshot(&edit);
    edit.agent_port = 9999; ASSERT_TRUE(shared_commit_settings(&edit, rev, false)); /* unsaved */
    ASSERT_EQ(settings_provision_ap_pass(TEST_AP_PASS), SETTINGS_SAVE_OK);
    ASSERT_EQ(flash_calls, 1);
    ASSERT_TRUE(strcmp(flashed.ap_pass, TEST_AP_PASS) == 0);
    ASSERT_EQ(flashed.agent_port, umb_settings_defaults().agent_port); /* no dirty RAM edit */
    umb_settings_t now; uint32_t after = shared_settings_snapshot(&now);
    ASSERT_TRUE(strcmp(now.ap_pass, TEST_AP_PASS) == 0);
    ASSERT_EQ(now.agent_port, 9999);          /* the unsaved edit stays unsaved in RAM */
    ASSERT_EQ(after, rev + 2);               /* edit, then provisioning */
    ASSERT_TRUE(shared_settings_write_allowed());
    /* A second call never rewrites an already provisioned credential. */
    ASSERT_EQ(settings_provision_ap_pass("OTHERPASSWORD234"), SETTINGS_SAVE_ALREADY_SET);
    ASSERT_EQ(flash_calls, 1);
    shared_get_settings(&now); ASSERT_TRUE(strcmp(now.ap_pass, TEST_AP_PASS) == 0);
}
TEST(test_provision_restores_persisted_pass_when_ram_lost_it) {
    setup(); strcpy(persisted.ap_pass, TEST_AP_PASS);   /* flash provisioned, RAM empty */
    ASSERT_EQ(settings_provision_ap_pass("OTHERPASSWORD234"), SETTINGS_SAVE_ALREADY_SET);
    ASSERT_EQ(flash_calls, 0);
    umb_settings_t now; shared_get_settings(&now);
    ASSERT_TRUE(strcmp(now.ap_pass, TEST_AP_PASS) == 0);
    ASSERT_TRUE(shared_settings_write_allowed());
}
TEST(test_provision_flash_failure_leaves_ram_unprovisioned) {
    setup(); flash_ok = false;
    ASSERT_EQ(settings_provision_ap_pass(TEST_AP_PASS), SETTINGS_SAVE_FLASH_FAILED);
    umb_settings_t now; shared_get_settings(&now); ASSERT_EQ(now.ap_pass[0], 0);
    ASSERT_TRUE(shared_settings_write_allowed());
    setup(); give_ack = false;
    ASSERT_EQ(settings_provision_ap_pass(TEST_AP_PASS), SETTINGS_SAVE_NEUTRAL_TIMEOUT);
    ASSERT_EQ(flash_calls, 0);
    setup(); umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);
    ASSERT_EQ(settings_provision_ap_pass(TEST_AP_PASS), SETTINGS_SAVE_BUSY);
    ASSERT_EQ(flash_calls, 0);
}
TEST(test_provision_rejects_invalid_password_without_flash) {
    setup();
    ASSERT_EQ(settings_provision_ap_pass(NULL), SETTINGS_SAVE_INVALID);
    ASSERT_EQ(settings_provision_ap_pass("short"), SETTINGS_SAVE_INVALID);
    ASSERT_EQ(settings_provision_ap_pass("bad\tpassword12"), SETTINGS_SAVE_INVALID);
    char long_pass[70]; memset(long_pass, 'A', 69); long_pass[69] = 0;
    ASSERT_EQ(settings_provision_ap_pass(long_pass), SETTINGS_SAVE_INVALID);
    ASSERT_EQ(flash_calls, 0);
    ASSERT_TRUE(shared_settings_write_allowed());
    /* The reservation-scoped commit refuses a stale or absent reservation. */
    ASSERT_FALSE(shared_provision_ap_pass(1, TEST_AP_PASS));
}
int main(void) {
    RUN_TEST(test_save_waits_for_neutral_and_blocks_racing_producers);
    RUN_TEST(test_save_ack_timeout_never_touches_flash_and_releases);
    RUN_TEST(test_save_failure_releases_reservation);
    RUN_TEST(test_save_armed_or_queued_refused_without_flash);
    RUN_TEST(test_provision_persists_only_saved_settings_then_publishes);
    RUN_TEST(test_provision_restores_persisted_pass_when_ram_lost_it);
    RUN_TEST(test_provision_flash_failure_leaves_ram_unprovisioned);
    RUN_TEST(test_provision_rejects_invalid_password_without_flash);
    TEST_SUMMARY();
}
