/* shared_state.h - cross-core mailbox between core 0 (console, later networking)
 * and the core-1 control task. Spinlock-protected copies only: no call blocks
 * beyond the copy, and no FreeRTOS call runs inside the lock. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_arming.h"
#include "umb_control.h"
#include "umb_settings.h"
#include "umb_types.h"

typedef enum {
    SHARED_ARM_CAR = 0,
    SHARED_ARM_ROS = 1,
    SHARED_ARM_ROS_DISARM = 2,
    SHARED_ARM_DISARM = 3
} shared_arm_kind_t;

/* Latest sensor readings, published by the core-1 sensor tasks (ToF, IMU/battery).
 * tof_mm: 9999 = invalid / stale / offline. Order HR, FR, R, L, FL, HL. */
typedef struct { int16_t tof_mm[6]; uint32_t tof_stamp_ms[6]; uint8_t tof_online; uint8_t tof_online_mask; bool tof_recovering;
                 uint32_t tof_restarts;
                 float accel_g[3], gyro_dps[3], yaw_rate_dps, heading_deg; bool imu_ok, imu_calibrated;
                 uint32_t imu_stamp_ms;
                 float bat_v, bat_v_min; bool bat_have; } shared_sensors_t;

/* Encoder HAL state as reported by the UI task. */
typedef enum { SHARED_ENC_STARTING = 0, SHARED_ENC_OK = 1, SHARED_ENC_FAILED = 2 } shared_encoder_state_t;

/* Once, before the tasks start. Outputs start neutral and disarmed;
 * the core-0 heartbeat starts at "now". */
void shared_init(const umb_settings_t *s);

/* --- producers (core 0) --- */
void shared_post_cmd(const umb_cmd_t *cmd);                 /* latest value wins */
void shared_post_arm_request(int kind /* shared_arm_kind_t */); /* FIFO; dropped when full */
void shared_set_link(bool wifi_up, bool agent_up);
void shared_core0_beat(uint32_t now_ms);
void shared_set_settings(const umb_settings_t *s);
bool shared_settings_write_allowed(void);  /* true only while !armed (and no arm request queued) */

/* --- consumers (core-1 control task) --- */
bool shared_take_cmd(umb_cmd_t *out);                       /* true when a new command was posted */
bool shared_take_arm_request(int *kind);                    /* oldest pending request */
void shared_get_link(bool *wifi_up, bool *agent_up);
uint32_t shared_core0_beat_ms(void);
void shared_publish_outputs(const umb_outputs_t *o);
void shared_publish_arming(const umb_arming_t *a);

/* --- sensors (producers: core-1 sensor tasks; readers: control task, console, Plan 3) --- */
void shared_publish_sensors(const shared_sensors_t *s);
void shared_get_sensors(shared_sensors_t *out);            /* expires stale data even if a producer stalls */
void shared_request_imu_cal(void);       /* console: ask the IMU task to re-calibrate */
bool shared_take_imu_cal(void);          /* IMU task: true once per request */

/* --- UI (buzzer cue mailbox, latest wins; encoder health) --- */
void shared_post_cue(int cue /* umb_cue_t */);
bool shared_take_cue(int *cue);
void shared_set_encoder_state(int state /* shared_encoder_state_t */);
int shared_encoder_state(void);

/* --- readers (any core) --- */
void shared_get_settings(umb_settings_t *out);
void shared_get_outputs(umb_outputs_t *out);
void shared_get_arming(umb_arming_t *out);
