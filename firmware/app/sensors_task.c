/* sensors_task.c - see sensors_task.h.
 * Every driver call (one init / poll / read = a write+read pair) is wrapped in its own
 * hal_i2c_lock/unlock, never a whole sequence, so a slow VL53L0X init never starves the bus.
 * Both tasks own a disjoint part of s_snap; s_snap_mutex keeps their read-modify-publish
 * from losing each other's fields (FreeRTOS mutex, outside the shared_state spinlock). */
#include "sensors_task.h"
#include <string.h>
#include <math.h>
#include "semphr.h"
#include "task.h"
#include "hal_adc.h"
#include "hal_gpio_misc.h"
#include "hal_i2c.h"
#include "hal_time.h"
#include "mpu6050.h"
#include "shared_state.h"
#include "umb_battery.h"
#include "umb_buzzer.h"
#include "umb_imu_filter.h"
#include "umb_tof_array.h"
#include "vl53l0x.h"

#define TOF_BUS HAL_I2C1
#define IMU_BUS HAL_I2C0
#define MPU6050_ADDR 0x68u
#define XSHUT_RESET_MS 2u          /* XSHUT low before a restart */
#define XSHUT_BOOT_MS 2u           /* VL53L0X firmware boot after XSHUT high (t_boot 1.2 ms) */
#define IMU_REINIT_FAILS 10u       /* consecutive read failures before a re-init */
#define IMU_REINIT_MIN_MS 1000u    /* at most one re-init per second */
#define IMU_SETTLE_MS 100u         /* gyro start-up after (re)configuration: the first ~40 ms
                                    * read stale and transient values (-125 dps seen on the bench) */
#define IMU_CAL_MAX_ATTEMPTS (2 * UMB_IMU_CAL_SAMPLES)   /* bounds a calibration on a dead bus */

static shared_sensors_t s_snap;
static SemaphoreHandle_t s_snap_mutex;

void sensors_init(void) {
    memset(&s_snap, 0, sizeof s_snap);
    for (int i = 0; i < UMB_TOF_COUNT; i++) s_snap.tof_mm[i] = UMB_TOF_INVALID_MM;
    s_snap_mutex = xSemaphoreCreateMutex();
    configASSERT(s_snap_mutex != NULL);
}

/* ------------------------------------------------------------------ ToF */

static vl53l0x_t s_tof_dev[UMB_TOF_COUNT];

/* One locked driver call per step; the sensor must be out of reset and at 0x29. */
static bool tof_bring_up(int i) {
    const umb_i2c_t *bus = hal_i2c_bus(TOF_BUS);
    vl53l0x_t *d = &s_tof_dev[i];
    int rc;
    if (!hal_i2c_lock(TOF_BUS)) return false;
    rc = vl53l0x_init(d, bus, VL53L0X_DEFAULT_ADDR);
    hal_i2c_unlock(TOF_BUS);
    if (rc != 0) return false;
    if (!hal_i2c_lock(TOF_BUS)) return false;
    rc = vl53l0x_set_address(d, bus, umb_tof_addr(i));
    hal_i2c_unlock(TOF_BUS);
    if (rc != 0) return false;
    /* init keeps Pololu's recomputed budget; force the project high-speed profile. */
    if (!hal_i2c_lock(TOF_BUS)) return false;
    rc = vl53l0x_set_timing_budget(d, bus, VL53L0X_HIGH_SPEED_BUDGET_US);
    hal_i2c_unlock(TOF_BUS);
    if (rc != 0) return false;
    if (!hal_i2c_lock(TOF_BUS)) return false;
    rc = vl53l0x_start_continuous(d, bus);
    hal_i2c_unlock(TOF_BUS);
    return rc == 0;
}

/* Releases sensor i from reset and brings it up. A failed sensor goes back into reset:
 * left powered at 0x29 it would collide with the next sensor brought up. */
static umb_tof_array_t tof_start(umb_tof_array_t a, int i) {
    hal_xshut_set(i, true);
    vTaskDelay(pdMS_TO_TICKS(XSHUT_BOOT_MS));
    bool ok = tof_bring_up(i);
    if (!ok) hal_xshut_set(i, false);
    return umb_tof_on_init_result(a, i, ok, hal_now_ms());
}

static umb_tof_array_t tof_restart(umb_tof_array_t a, int i) {
    hal_xshut_set(i, false);
    vTaskDelay(pdMS_TO_TICKS(XSHUT_RESET_MS));
    return tof_start(a, i);
}

static umb_tof_array_t tof_poll(umb_tof_array_t a, int i, uint32_t now_ms) {
    uint16_t mm = 0;
    int rc = -1;
    if (hal_i2c_lock(TOF_BUS)) {
        rc = vl53l0x_poll_range(&s_tof_dev[i], hal_i2c_bus(TOF_BUS), &mm, NULL);
        hal_i2c_unlock(TOF_BUS);
    }
    return umb_tof_on_poll(a, i, rc, mm, now_ms);
}

static void tof_publish(const umb_tof_array_t *a) {
    uint32_t now = hal_now_ms();
    uint32_t restarts = 0;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        /* recoveries only: the slot's first (boot) init is not a restart */
        if (a->s[i].restarts > 0u) restarts += a->s[i].restarts - 1u;
    }
    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    umb_tof_distances(a, now, s_snap.tof_mm);
    for (int i = 0; i < UMB_TOF_COUNT; i++) s_snap.tof_stamp_ms[i] = a->s[i].last_ok_ms;
    s_snap.tof_online = umb_tof_online(a, now);
    s_snap.tof_online_mask = 0;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        if (a->s[i].link == UMB_TOF_ONLINE &&
            (int32_t)(now - a->s[i].last_ok_ms) <= (int32_t)UMB_TOF_STALE_MS)
            s_snap.tof_online_mask |= (uint8_t)(1u << i);
    }
    s_snap.tof_recovering = umb_tof_recovering(a, now);
    s_snap.tof_restarts = restarts;
    shared_publish_sensors(&s_snap);
    xSemaphoreGive(s_snap_mutex);
}

void tof_task(void *param) {
    (void)param;
    umb_tof_array_t tof = umb_tof_init();
    hal_xshut_init();                        /* all sensors held in reset */
    vTaskDelay(pdMS_TO_TICKS(XSHUT_RESET_MS));
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        tof = tof_start(tof, i);
        tof_publish(&tof);
    }

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TOF_PERIOD_MS));
        for (int i = 0; i < UMB_TOF_COUNT; i++) {
            uint32_t now = hal_now_ms();
            if (umb_tof_next_action(&tof, i, now) == UMB_TOF_ACT_RESTART) tof = tof_restart(tof, i);
            else if (tof.s[i].link == UMB_TOF_ONLINE) tof = tof_poll(tof, i, now);
        }
        tof_publish(&tof);
        /* a restart can take ~300 ms: do not burst-poll to catch up */
        if ((int32_t)(xTaskGetTickCount() - wake) > (int32_t)pdMS_TO_TICKS(TOF_PERIOD_MS))
            wake = xTaskGetTickCount();
    }
}

/* ------------------------------------------------------------- IMU + battery */

typedef struct {
    umb_imu_state_t st;
    umb_imu_raw_t raw;            /* last good sample */
    umb_imu_cal_t cal;
    int cal_attempts;             /* < 0: no calibration running */
    uint32_t fails;               /* consecutive read failures */
    bool ready;                  /* WHO_AM_I and all configuration writes succeeded */
    uint32_t last_init_ms;
    umb_battery_t bat;
    uint32_t next_bat_ms;
} imu_ctx_t;

/* Configures the chip, then waits out the gyro start-up so no transient reaches the
 * filter or a calibration. */
static bool imu_init_chip(void) {
    if (!hal_i2c_lock(IMU_BUS)) return false;
    int rc = mpu6050_init(hal_i2c_bus(IMU_BUS), MPU6050_ADDR);
    hal_i2c_unlock(IMU_BUS);
    if (rc == 0) vTaskDelay(pdMS_TO_TICKS(IMU_SETTLE_MS));
    return rc == 0;
}

static bool imu_read(umb_imu_raw_t *raw) {
    mpu6050_sample_t s;
    if (!hal_i2c_lock(IMU_BUS)) return false;
    int rc = mpu6050_read(hal_i2c_bus(IMU_BUS), MPU6050_ADDR, &s);
    hal_i2c_unlock(IMU_BUS);
    if (rc != 0) return false;
    memcpy(raw->accel_g, s.accel_g, sizeof raw->accel_g);
    memcpy(raw->gyro_dps, s.gyro_dps, sizeof raw->gyro_dps);
    return true;
}

static bool imu_cal_allowed(void) {
    umb_outputs_t out;
    shared_get_outputs(&out);
    return shared_settings_write_allowed() && !out.armed && fabsf(out.speed_signed) < 0.05f;
}

static void imu_cal_start(imu_ctx_t *c) {
    memset(&c->cal, 0, sizeof c->cal);
    c->cal_attempts = 0;
}

/* One calibration sample; finishes (and cues) after 200 good samples or the attempt bound.
 * A rejected boot calibration leaves bias 0 / uncalibrated (umb_imu_init); a rejected
 * re-calibration keeps the previous bias. */
static void imu_cal_step(imu_ctx_t *c, bool ok) {
    if (c->cal_attempts < 0) return;
    /* Recheck throughout collection: an arm request or motion cancels the calibration
     * before it can replace the previous bias. This also gates boot calibration. */
    if (!imu_cal_allowed()) {
        c->cal_attempts = -1;
        shared_post_cue(UMB_CUE_ERROR);
        return;
    }
    c->cal_attempts++;
    if (ok) c->cal = umb_imu_cal_add(c->cal, &c->raw);
    if (c->cal.n < UMB_IMU_CAL_SAMPLES && c->cal_attempts < IMU_CAL_MAX_ATTEMPTS) return;
    bool done = umb_imu_cal_finish(&c->cal, &c->st);
    shared_post_cue(done ? UMB_CUE_CAL_DONE : UMB_CUE_ERROR);
    c->cal_attempts = -1;
}

static void imu_battery_step(imu_ctx_t *c, uint32_t now, float multiplier) {
    if ((int32_t)(now - c->next_bat_ms) < 0) return;
    c->bat = umb_battery_add(c->bat, hal_adc_battery_raw(), multiplier);
    c->next_bat_ms = now + BAT_PERIOD_MS;
}

static void imu_publish(const imu_ctx_t *c, uint32_t now) {
    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    memcpy(s_snap.accel_g, c->raw.accel_g, sizeof s_snap.accel_g);
    memcpy(s_snap.gyro_dps, c->raw.gyro_dps, sizeof s_snap.gyro_dps);
    s_snap.yaw_rate_dps = c->st.yaw_rate_dps;
    s_snap.heading_deg = c->st.heading_deg;
    s_snap.imu_ok = umb_imu_is_ok(&c->st, now);
    s_snap.imu_calibrated = c->st.calibrated;
    s_snap.imu_stamp_ms = c->st.last_ok_ms;
    s_snap.bat_v = c->bat.v_filtered;
    s_snap.bat_v_min = c->bat.v_min;
    s_snap.bat_have = c->bat.have;
    shared_publish_sensors(&s_snap);
    xSemaphoreGive(s_snap_mutex);
}

void imu_task(void *param) {
    (void)param;
    imu_ctx_t c;
    memset(&c, 0, sizeof c);
    c.st = umb_imu_init();
    c.bat = umb_battery_init();
    hal_adc_init();
    c.ready = imu_init_chip();
    c.last_init_ms = hal_now_ms();
    c.next_bat_ms = c.last_init_ms;
    imu_cal_start(&c);

    umb_settings_t settings;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        /* a re-init sleeps IMU_SETTLE_MS: do not burst-read to catch up */
        if ((int32_t)(xTaskGetTickCount() - wake) > (int32_t)pdMS_TO_TICKS(IMU_PERIOD_MS))
            wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(c.cal_attempts >= 0 ? IMU_CAL_PERIOD_MS : IMU_PERIOD_MS));
        shared_get_settings(&settings);
        if (shared_take_imu_cal()) {
            if (imu_cal_allowed() && c.cal_attempts < 0) imu_cal_start(&c);
            else shared_post_cue(UMB_CUE_ERROR);
        }

        umb_imu_raw_t raw = { { 0 }, { 0 } };
        bool ok = c.ready && imu_read(&raw);
        uint32_t now = hal_now_ms();
        if (ok) { c.raw = raw; c.fails = 0; }
        else c.fails++;
        imu_cal_step(&c, ok);
        c.st = umb_imu_update(c.st, &raw, ok, now, settings.imu_rotate);
        imu_battery_step(&c, now, settings.bat_multiplier);
        if (c.fails >= IMU_REINIT_FAILS && (uint32_t)(now - c.last_init_ms) >= IMU_REINIT_MIN_MS) {
            c.ready = imu_init_chip();
            if (c.ready) c.fails = 0;
            c.last_init_ms = hal_now_ms();
        }
        imu_publish(&c, hal_now_ms());
    }
}
