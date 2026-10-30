#include "test_runner.h"
#include "settings_apply.h"
#include "shared_state.h"
#include "hal_flash_store.h"
#include "ros_save.h"
#include "hal_time.h"
#include "FreeRTOS.h"
#include "task.h"
static uint32_t clock_ms;
static bool give_ack, flash_ok, rejected_arm, rejected_settings;
static int flash_calls, flash_slot, hook_calls, hook_flash_calls, nested_result = -1;
static uint32_t flash_seq;
static bool nested_save;
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
int hal_settings_slot(void) { return flash_slot; }
uint32_t hal_settings_seq(void) { return flash_seq; }
bool hal_settings_save(const umb_settings_t *s) {
    flashed = *s; flash_calls++;
    if (nested_save) { nested_save = false; nested_result = (int)settings_save_current(); }
    if (flash_ok) { persisted = *s; flash_seq++; flash_slot = !flash_slot; }
    rejected_arm = !shared_post_arm_request(SHARED_ARM_CAR);
    umb_settings_t settings; uint32_t revision = shared_settings_snapshot(&settings);
    rejected_settings = !shared_commit_settings(&settings, revision, false);
    return flash_ok;
}
bool hal_settings_persisted(umb_settings_t *out) { *out = persisted; return true; }
static void setup(void) {
    clock_ms = flash_calls = hook_calls = hook_flash_calls = 0; give_ack = flash_ok = true;
    flash_slot = 1; flash_seq = 4; nested_save = false; nested_result = -1;
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
static void progress_hook(void) { hook_calls++; hook_flash_calls = flash_calls; }

TEST(test_progress_hook_runs_after_neutral_ack_before_flash_only) {
    setup();
    ASSERT_EQ(settings_save_with_progress(progress_hook), SETTINGS_SAVE_OK);
    ASSERT_EQ(hook_calls, 1);
    ASSERT_EQ(hook_flash_calls, 0);   /* before any erase/program */
    ASSERT_EQ(flash_calls, 1);
    setup(); give_ack = false;
    ASSERT_EQ(settings_save_with_progress(progress_hook), SETTINGS_SAVE_NEUTRAL_TIMEOUT);
    ASSERT_EQ(hook_calls, 0);
    setup(); umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);
    ASSERT_EQ(settings_save_with_progress(progress_hook), SETTINGS_SAVE_BUSY);
    ASSERT_EQ(hook_calls, 0);
    ASSERT_EQ(flash_calls, 0);
    setup();
    ASSERT_EQ(settings_save_with_progress(NULL), SETTINGS_SAVE_OK); /* hook optional */
}
TEST(test_second_save_during_flash_transaction_is_refused) {
    setup(); nested_save = true;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_OK);
    ASSERT_EQ(nested_result, SETTINGS_SAVE_BUSY);
    ASSERT_EQ(flash_calls, 1);        /* the shared flash buffer was entered once */
    ASSERT_TRUE(shared_settings_write_allowed());
}
TEST(test_retry_after_flash_failure_succeeds_and_reservation_released) {
    setup(); flash_ok = false;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_FLASH_FAILED);
    ASSERT_TRUE(shared_settings_write_allowed());
    ASSERT_TRUE(shared_post_arm_request(SHARED_ARM_CAR)); /* arming possible again */
    setup(); flash_ok = false;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_FLASH_FAILED);
    flash_ok = true;
    ASSERT_EQ(settings_save_current(), SETTINGS_SAVE_OK);
    ASSERT_EQ(flash_calls, 2);
}
static ros_save_reply_t ros_save(uint32_t gen) { return ros_services_save(gen, progress_hook); }
static uint32_t ready_session(void) { shared_set_link(true, true); return shared_session_generation(); }
TEST(test_ros_save_success_reports_slot_and_sequence_only) {
    setup(); uint32_t gen = ready_session();
    ros_save_reply_t r = ros_save(gen);
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(strcmp(r.message, "saved: slot 0 seq 5") == 0);
    ASSERT_EQ(hook_calls, 1);
    ASSERT_EQ(shared_maintenance_active(), false);
}
TEST(test_ros_save_failure_does_not_claim_new_sequence) {
    setup(); uint32_t gen = ready_session(); flash_ok = false;
    ros_save_reply_t r = ros_save(gen);
    ASSERT_FALSE(r.success);
    ASSERT_TRUE(strcmp(r.message, "error: flash write failed") == 0);
    ASSERT_TRUE(strstr(r.message, "seq") == NULL);
    ASSERT_EQ(flash_seq, 4);
    ASSERT_FALSE(shared_maintenance_active());
    flash_ok = true;                         /* retry works, reservation was released */
    r = ros_save(gen);
    ASSERT_TRUE(r.success && strcmp(r.message, "saved: slot 0 seq 5") == 0);
}
TEST(test_ros_save_refusals_never_touch_flash) {
    setup(); uint32_t gen = ready_session();
    umb_outputs_t o = {.armed = true}; shared_publish_outputs(&o);
    ros_save_reply_t r = ros_save(gen);
    ASSERT_FALSE(r.success); ASSERT_TRUE(strcmp(r.message, "refused: armed or busy") == 0);
    setup(); gen = ready_session(); give_ack = false;
    r = ros_save(gen);
    ASSERT_FALSE(r.success); ASSERT_TRUE(strcmp(r.message, "error: neutral ack timeout") == 0);
    ASSERT_EQ(clock_ms, 60);
    setup(); gen = ready_session();
    r = ros_save(gen + 1u);               /* stale session */
    ASSERT_FALSE(r.success); ASSERT_TRUE(strcmp(r.message, "refused: session") == 0);
    r = ros_save(0);
    ASSERT_FALSE(r.success); ASSERT_TRUE(strcmp(r.message, "refused: session") == 0);
    ASSERT_EQ(flash_calls, 0);
    ASSERT_EQ(hook_calls, 0);
    ASSERT_FALSE(shared_maintenance_active());
}
TEST(test_ros_save_reply_never_contains_credentials) {
    setup(); strcpy(persisted.ap_pass, TEST_AP_PASS);
    umb_settings_t s; uint32_t rev = shared_settings_snapshot(&s);
    strcpy(s.ap_pass, TEST_AP_PASS); strcpy(s.wifi_pass, "SYNTHWIFIPASS99");
    ASSERT_TRUE(shared_commit_settings(&s, rev, false));
    ros_save_reply_t r = ros_save(ready_session());
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(strstr(r.message, TEST_AP_PASS) == NULL && strstr(r.message, "SYNTHWIFIPASS99") == NULL);
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
    RUN_TEST(test_progress_hook_runs_after_neutral_ack_before_flash_only);
    RUN_TEST(test_second_save_during_flash_transaction_is_refused);
    RUN_TEST(test_retry_after_flash_failure_succeeds_and_reservation_released);
    RUN_TEST(test_ros_save_success_reports_slot_and_sequence_only);
    RUN_TEST(test_ros_save_failure_does_not_claim_new_sequence);
    RUN_TEST(test_ros_save_refusals_never_touch_flash);
    RUN_TEST(test_ros_save_reply_never_contains_credentials);
    TEST_SUMMARY();
}
