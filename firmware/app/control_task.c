/* control_task.c - see control_task.h */
#include "control_task.h"
#include <string.h>
#include "task.h"
#include "hardware/watchdog.h"
#include "hal_pwm.h"
#include "hal_tach.h"
#include "hal_time.h"
#include "shared_state.h"
#include "umb_arming.h"
#include "umb_control.h"
#include "umb_imu_filter.h"

/* Hands one queued arm request to the arming state machine. */
static umb_arming_t apply_arm_request(umb_arming_t a, int kind) {
    switch (kind) {
    case SHARED_ARM_CAR:        return umb_arming_car_key(a, NULL);
    case SHARED_ARM_ROS:        return umb_arming_ros(a, true, NULL);
    case SHARED_ARM_ROS_DISARM: return umb_arming_ros(a, false, NULL);
    case SHARED_ARM_DISARM:     return umb_arming_disarm(a);
    default:                    return a;
    }
}

typedef struct {
    umb_cmd_t cmd;        /* last command received */
    bool have_cmd;
} cmd_state_t;

/* imu_ok only while the snapshot's sample is fresh: a stalled IMU task must not
 * leave a stale "ok" behind. Signed age; a future stamp counts as fresh. */
static bool imu_fresh(const shared_sensors_t *sn, uint32_t now_ms) {
    return sn->imu_ok && (int32_t)(now_ms - sn->imu_stamp_ms) <= (int32_t)UMB_IMU_STALE_MS;
}

static void fill_sensors(umb_sensor_in_t *si, const shared_sensors_t *sn, uint32_t now_ms) {
    si->tof_present = true;
    memcpy(si->tof_mm, sn->tof_mm, sizeof si->tof_mm);
    si->tof_online = sn->tof_online;
    si->tof_recovering = sn->tof_recovering;
    si->imu_present = true;
    si->imu_ok = imu_fresh(sn, now_ms);
    memcpy(si->accel_g, sn->accel_g, sizeof si->accel_g);
    si->bat_present = sn->bat_have;
    si->battery_v = sn->bat_v;
}

/* Reads the clock only after the command / beat / sensor snapshots, so none can be
 * stamped later than now_ms by another task in between. */
static umb_control_in_t gather_inputs(cmd_state_t *cs) {
    umb_cmd_t fresh;
    if (shared_take_cmd(&fresh)) {
        cs->cmd = fresh;
        cs->have_cmd = true;
    }
    uint32_t core0_beat_ms = shared_core0_beat_ms();
    shared_sensors_t sn;
    shared_get_sensors(&sn);
    umb_control_in_t in;
    memset(&in, 0, sizeof in);
    in.cmd = cs->cmd;
    in.safety.now_ms = hal_now_ms();
    fill_sensors(&in.sensors, &sn, in.safety.now_ms);
    in.safety.have_cmd = cs->have_cmd;
    in.safety.last_cmd_ms = cs->cmd.rx_ms;
    in.safety.source = cs->cmd.source;
    shared_get_link(&in.safety.wifi_up, &in.safety.agent_up);
    in.safety.core0_beat_ms = core0_beat_ms;
    in.tach_count = hal_tach_count();
    in.tach_age_us = hal_tach_age_us();
    return in;
}

void control_task(void *param) {
    (void)param;
    umb_settings_t settings;
    shared_get_settings(&settings);
    umb_control_t ctl = umb_control_init(&settings, hal_tach_count());
    cmd_state_t cs;
    memset(&cs, 0, sizeof cs);
    bool watchdog_on = false;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        shared_get_settings(&settings);
        int kind;
        while (shared_take_arm_request(&kind)) ctl.arming = apply_arm_request(ctl.arming, kind);

        umb_control_in_t in = gather_inputs(&cs);
        umb_outputs_t out;
        ctl = umb_control_step(ctl, &in, &settings, &out);
        hal_pwm_write(out.servo_us, out.esc_us);
        shared_publish_outputs(&out);
        shared_publish_arming(&ctl.arming);

        if (!watchdog_on) {
            watchdog_enable(CONTROL_WATCHDOG_MS, true);
            watchdog_on = true;
        } else {
            watchdog_update();
        }
    }
}
