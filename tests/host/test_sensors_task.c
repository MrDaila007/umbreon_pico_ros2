/* Exercise the real task/driver/filter path with simulated time and I2C. */
#include <setjmp.h>
#include <string.h>
#include "test_runner.h"
#include "fake_i2c.h"
#include "sensors_task.h"
#include "shared_state.h"
#include "hal_i2c.h"
#include "hal_adc.h"
#include "hal_gpio_misc.h"
#include "hal_time.h"
#include "semphr.h"
#include "umb_buzzer.h"

static fake_i2c_t bus;
static umb_i2c_t api;
static jmp_buf finish;
static uint32_t clock_ms, end_ms, block_at_ms;
static bool blocked, moving;
static shared_sensors_t snapshot;
static int cal_done, errors, init_attempts;
static uint32_t init_times[8];

uint32_t hal_now_ms(void) { return clock_ms; }
TickType_t xTaskGetTickCount(void) { return clock_ms; }
void vTaskDelay(TickType_t ticks) { clock_ms += ticks; }
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks) {
    *wake += ticks;
    if (clock_ms < *wake) clock_ms = *wake;
    if (clock_ms >= block_at_ms) blocked = true;
    if (clock_ms >= end_ms) longjmp(finish, 1);
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &bus; }
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t ticks) { (void)mutex; (void)ticks; return 1; }
int xSemaphoreGive(SemaphoreHandle_t mutex) { (void)mutex; return 1; }
const umb_i2c_t *hal_i2c_bus(hal_i2c_bus_t b) { (void)b; return &api; }
bool hal_i2c_lock(hal_i2c_bus_t b) { (void)b; return true; }
void hal_i2c_unlock(hal_i2c_bus_t b) { (void)b; }
void hal_adc_init(void) {}
uint16_t hal_adc_battery_raw(void) { return 2048; }
void hal_xshut_init(void) {}
void hal_xshut_set(int i, bool on) { (void)i; (void)on; }
void shared_publish_sensors(const shared_sensors_t *s) { snapshot = *s; }
void shared_get_settings(umb_settings_t *s) { *s = umb_settings_defaults(); }
void shared_get_outputs(umb_outputs_t *out) {
    memset(out, 0, sizeof *out);
    out->armed = blocked;
    out->speed_signed = moving ? 0.1f : 0.0f;
}
bool shared_settings_write_allowed(void) { return !blocked; }
bool shared_take_imu_cal(void) { return false; }
void shared_post_cue(int cue) {
    if (cue == UMB_CUE_CAL_DONE) cal_done++;
    if (cue == UMB_CUE_ERROR) errors++;
}

/* Count WHO_AM_I transactions without replacing the real driver. */
static int tracked_write(void *ctx, uint8_t addr, const uint8_t *buf, size_t n, bool nostop) {
    if (n == 1 && buf[0] == 0x75) {
        if (init_attempts < 8) init_times[init_attempts] = clock_ms;
        init_attempts++;
    }
    umb_i2c_t underlying = fake_i2c_bus(ctx);
    return underlying.write(ctx, addr, buf, n, nostop);
}

static void setup(uint8_t whoami, uint32_t duration) {
    fake_i2c_reset(&bus);
    bus.present[0x68] = true;
    bus.regs[0x68][0x75] = whoami;
    bus.regs[0x68][0x3f] = 0x10; /* az = 4096 -> +1g */
    bus.regs[0x68][0x47] = 0x00;
    bus.regs[0x68][0x48] = 0x62; /* gz = 98 -> ~1.5 dps bias */
    api = fake_i2c_bus(&bus);
    api.write = tracked_write;
    memset(&snapshot, 0, sizeof snapshot);
    clock_ms = 0; end_ms = duration; block_at_ms = UINT32_MAX;
    blocked = false; moving = false;
    cal_done = errors = init_attempts = 0;
    memset(init_times, 0, sizeof init_times);
    sensors_init();
}

static void run(void) {
    if (setjmp(finish) == 0) imu_task(NULL);
}

TEST(test_task_wrong_whoami_never_healthy) {
    setup(0x00, 2400);
    run();
    ASSERT_FALSE(snapshot.imu_ok);
    ASSERT_FALSE(snapshot.imu_calibrated);
    ASSERT_EQ(cal_done, 0);
    ASSERT_EQ(errors, 1); /* bounded failed boot calibration */
    ASSERT_EQ(init_attempts, 3);
    ASSERT_TRUE(init_times[1] - init_times[0] >= 1000);
    ASSERT_TRUE(init_times[2] - init_times[1] >= 1000);
    ASSERT_TRUE(snapshot.bat_have); /* dead IMU does not stop the battery task */
}

TEST(test_task_failed_configuration_never_healthy) {
    setup(0x68, 1400);
    bus.nack_writes_after = 1; /* WHO_AM_I pointer works; configuration fails */
    run();
    ASSERT_FALSE(snapshot.imu_ok);
    ASSERT_FALSE(snapshot.imu_calibrated);
    ASSERT_EQ(cal_done, 0);
}

TEST(test_task_boot_calibration_and_filter) {
    setup(0x68, 1500);
    run();
    ASSERT_TRUE(snapshot.imu_ok);
    ASSERT_TRUE(snapshot.imu_calibrated);
    ASSERT_EQ(cal_done, 1);
    ASSERT_EQ(errors, 0);
    ASSERT_FLOAT_EQ(snapshot.yaw_rate_dps, 0.0f, 0.01f);
    ASSERT_FLOAT_EQ(snapshot.accel_g[2], 1.0f, 0.001f);
}

TEST(test_task_arming_during_calibration_aborts) {
    setup(0x68, 1500);
    block_at_ms = 500;
    run();
    ASSERT_TRUE(snapshot.imu_ok);
    ASSERT_FALSE(snapshot.imu_calibrated);
    ASSERT_EQ(cal_done, 0);
    ASSERT_EQ(errors, 1);
}

TEST(test_task_moving_at_boot_refuses_calibration) {
    setup(0x68, 1500);
    moving = true;
    run();
    ASSERT_TRUE(snapshot.imu_ok);
    ASSERT_FALSE(snapshot.imu_calibrated);
    ASSERT_EQ(cal_done, 0);
    ASSERT_EQ(errors, 1);
}

int main(void) {
    RUN_TEST(test_task_wrong_whoami_never_healthy);
    RUN_TEST(test_task_failed_configuration_never_healthy);
    RUN_TEST(test_task_boot_calibration_and_filter);
    RUN_TEST(test_task_arming_during_calibration_aborts);
    RUN_TEST(test_task_moving_at_boot_refuses_calibration);
    TEST_SUMMARY();
}
