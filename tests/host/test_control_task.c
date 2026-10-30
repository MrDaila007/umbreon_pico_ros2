/* Real control task + shared mailboxes, simulated scheduler/clock/PWM only. */
#include <setjmp.h>
#include <string.h>
#include "test_runner.h"
#include "control_task.h"
#include "shared_state.h"
#include "hal_time.h"
#include "hal_pwm.h"
#include "hal_tach.h"
#include "hardware/watchdog.h"

static jmp_buf finished;
static uint32_t clock_ms, tick, max_ticks, ros_id, maintenance_id, watchdog_at;
static int scenario;
static bool saw_drive, drive_after_loss, saw_armed, maintenance_ack_order;
static uint16_t esc;

uint32_t hal_now_ms(void) { return clock_ms; }
TickType_t xTaskGetTickCount(void) { return clock_ms; }
void vTaskDelay(TickType_t ms) { clock_ms += ms; }
void watchdog_enable(uint32_t ms, bool pause) { (void)ms; (void)pause; watchdog_at = clock_ms; }
void watchdog_update(void) { watchdog_at = clock_ms; }
void hal_pwm_write(uint16_t servo, uint16_t value) {
    (void)servo; esc = value;
    if (value > 1500) saw_drive = true;
    if (tick >= 3 && value != 1500) drive_after_loss = true;
}
uint32_t hal_tach_count(void) { return 0; }
uint32_t hal_tach_age_us(void) {
    if (tick == 1 && scenario == 1) (void)shared_cancel_arm(ros_id);
    if (tick == 1 && scenario == 2) clock_ms += 60; /* expire between pop and final commit */
    return 0;
}
static void command(float throttle) {
    umb_cmd_t cmd = {UMB_CMD_RAW, UMB_SRC_CONSOLE, UMB_DRIVE_MANUAL, 0, throttle, clock_ms};
    (void)shared_post_cmd(&cmd);
}
void vTaskDelayUntil(TickType_t *wake, TickType_t ms) {
    if (tick) {
        umb_outputs_t out; shared_get_outputs(&out);
        if (out.armed) saw_armed = true;
        if (scenario == 4 && tick == 1)
            maintenance_ack_order = shared_maintenance_acknowledged(maintenance_id) && esc == 1500 && watchdog_at == clock_ms;
    }
    if (++tick > max_ticks) longjmp(finished, 1);
    *wake += ms;
    if (clock_ms < *wake) clock_ms = *wake;
    if ((scenario == 5 || scenario == 6) && tick == 2) clock_ms += 40; /* core 1 parked / starved */
    if (scenario != 3 || clock_ms <= 100) shared_core0_beat(clock_ms);
    if (scenario == 0) {
        if (tick == 1 || tick == 4) {
            (void)shared_post_arm_request(SHARED_ARM_CAR);
            (void)shared_post_arm_request(SHARED_ARM_ROS);
        }
        if (tick == 1) command(0);
        if (tick == 2) command(0.5f);
        if (tick == 3) { shared_set_link(false, false); shared_set_link(true, true); }
        /* No new command after loss: even new arm keys must not revive cached throttle. */
    } else if (scenario == 1 || scenario == 2) {
        if (tick == 1) {
            (void)shared_post_arm_request(SHARED_ARM_CAR);
            (void)shared_submit_arm(SHARED_ARM_ROS, 0, clock_ms, &ros_id);
            command(0);
        }
    } else if (scenario == 3) command(tick == 1 ? 0 : 0.5f); /* console alive, net beat stalled */
}

static void setup(int which, unsigned ticks, umb_arm_mode_t mode) {
    scenario = which; clock_ms = tick = ros_id = maintenance_id = watchdog_at = 0;
    max_ticks = ticks; esc = 1500;
    saw_drive = drive_after_loss = saw_armed = maintenance_ack_order = false;
    umb_settings_t cfg = umb_settings_defaults(); cfg.arming_mode = mode;
    shared_init(&cfg); shared_set_link(true, true);
}
static void run(void) { if (!setjmp(finished)) control_task(NULL); }

TEST(test_task_loss_clears_cached_command_even_after_rearm) {
    setup(0, 6, UMB_ARM_BOTH); run();
    ASSERT_TRUE(saw_drive);
    ASSERT_FALSE(drive_after_loss);
    ASSERT_EQ(esc, 1500);
}
TEST(test_task_cancel_after_pop_never_arms) {
    setup(1, 3, UMB_ARM_BOTH); run();
    ASSERT_FALSE(saw_armed);
    ASSERT_FALSE(saw_drive);
    shared_arm_ack_t ack; ASSERT_TRUE(shared_get_arm_ack(ros_id, &ack));
    ASSERT_EQ(ack.state, SHARED_REQUEST_CANCELLED);
}
TEST(test_task_expired_after_pop_never_arms) {
    setup(2, 3, UMB_ARM_BOTH); run();
    ASSERT_FALSE(saw_armed);
    ASSERT_FALSE(saw_drive);
}
TEST(test_task_console_traffic_does_not_mask_stale_net_beat) {
    setup(3, 35, UMB_ARM_NONE); run();
    ASSERT_TRUE(saw_drive);
    umb_outputs_t out; shared_get_outputs(&out);
    ASSERT_TRUE(out.stop_mask & UMB_STOP_CORE0);
    ASSERT_EQ(out.esc_us, 1500);
    ASSERT_FALSE(out.motion_allowed);
}
TEST(test_task_maintenance_ack_follows_pwm_and_watchdog) {
    setup(4, 2, UMB_ARM_BOTH);
    ASSERT_TRUE(shared_begin_maintenance(&maintenance_id));
    run();
    ASSERT_TRUE(maintenance_ack_order);
    ASSERT_TRUE(shared_maintenance_acknowledged(maintenance_id));
}
TEST(test_task_flash_pause_counted_apart_from_jitter) {
    setup(5, 4, UMB_ARM_BOTH);
    ASSERT_TRUE(shared_begin_maintenance(&maintenance_id));
    run();
    umb_cycle_stats_t st; shared_get_cycle_stats(&st);
    ASSERT_EQ(st.flash_pauses, 1);
    ASSERT_EQ(st.flash_pause_max_ms, 40);
    ASSERT_EQ(st.jitter_late, 0);
    ASSERT_TRUE(st.cycles >= 3);
}
TEST(test_task_late_cycle_without_maintenance_is_jitter) {
    setup(6, 4, UMB_ARM_BOTH); run();
    umb_cycle_stats_t st; shared_get_cycle_stats(&st);
    ASSERT_EQ(st.jitter_late, 1);
    ASSERT_EQ(st.jitter_max_ms, 40);
    ASSERT_EQ(st.flash_pauses, 0);
}
int main(void) {
    RUN_TEST(test_task_loss_clears_cached_command_even_after_rearm);
    RUN_TEST(test_task_cancel_after_pop_never_arms);
    RUN_TEST(test_task_expired_after_pop_never_arms);
    RUN_TEST(test_task_console_traffic_does_not_mask_stale_net_beat);
    RUN_TEST(test_task_maintenance_ack_follows_pwm_and_watchdog);
    RUN_TEST(test_task_flash_pause_counted_apart_from_jitter);
    RUN_TEST(test_task_late_cycle_without_maintenance_is_jitter);
    TEST_SUMMARY();
}
