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
bool shared_post_cmd(const umb_cmd_t *cmd);                 /* latest value wins */
bool shared_post_arm_request(int kind /* shared_arm_kind_t */); /* disarm has sticky priority */
void shared_set_link(bool wifi_up, bool agent_up);
void shared_core0_beat(uint32_t now_ms);
uint32_t shared_settings_snapshot(umb_settings_t *out);
bool shared_commit_settings(const umb_settings_t *s, uint32_t revision, bool mode_only);
bool shared_settings_write_allowed(void);  /* true only while !armed (and no arm request queued) */

/* --- consumers (core-1 control task) --- */
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

/* Network callbacks must carry the generation of their ready session. */
uint32_t shared_session_generation(void);
/* Atomic: generation is nonzero, current, and WiFi + agent are both up. */
bool shared_session_ready(uint32_t generation);
bool shared_post_ros_cmd(const umb_cmd_t *cmd, uint32_t generation);
void shared_notify_network_loss(uint32_t reason_mask);

typedef struct {
    umb_cmd_t cmd;
    bool cmd_pending, wifi_up, agent_up, maintenance;
    uint32_t command_epoch, loss_mask, beat_ms, maintenance_id;
} shared_control_mailbox_t;
/* One atomic read-and-consume for the control task; loss cannot vanish on down/up. */
void shared_take_control_mailbox(shared_control_mailbox_t *out);

/* Reserve disarmed settings/flash work; only control can acknowledge neutral. */
bool shared_begin_maintenance(uint32_t *id);
void shared_ack_maintenance(uint32_t id);
bool shared_maintenance_acknowledged(uint32_t id);
void shared_end_maintenance(uint32_t id);
bool shared_maintenance_active(void); /* any reservation held (non-consuming read) */
/* Only for first-boot provisioning after its verified flash write: sets ap_pass
 * if reservation `id` is held and no AP password exists yet. Bumps the revision. */
bool shared_provision_ap_pass(uint32_t id, const char *password);

/* Network owner status for console/OLED/diagnostics. Never holds credentials;
 * the STA SSID is omitted too. Published by the core-0 network task only. */
typedef struct {
    uint8_t wifi_mode, wifi_stage, endpoint_method, ap_pass_stage, ap_pass_last;
    bool radio_ready, wifi_up, sta_up, ap_up, rssi_valid, endpoint_selected, probe_owned;
    int32_t rssi;
    uint32_t local_ip, endpoint_ip;
    uint16_t endpoint_port;
    char ap_ssid[13];
    uint32_t probes, probe_failures, wifi_errors, reconfigures, link_losses;
    uint32_t endpoint_losses, ap_pass_failures, loop_last_ms, loop_max_ms, radio_init_ms;
} shared_network_t;
void shared_publish_network(const shared_network_t *n);
void shared_get_network(shared_network_t *out);

#define SHARED_ARM_QUEUE_LEN 8u
typedef struct {
    int kind;
    uint32_t id, generation, deadline_ms;
} shared_arm_request_t;
typedef enum { SHARED_REQUEST_PENDING, SHARED_REQUEST_APPLYING,
               SHARED_REQUEST_DONE, SHARED_REQUEST_CANCELLED } shared_request_state_t;
typedef struct {
    uint32_t id;
    shared_request_state_t state;
    bool armed, car_key, ros_key;
    umb_arm_result_t result;
} shared_arm_ack_t;
/* expected_session=0 for local UI/console; nonzero for a ROS session request. */
bool shared_submit_arm(int kind, uint32_t expected_session, uint32_t now_ms, uint32_t *id);
bool shared_take_arm_record(shared_arm_request_t *out, uint32_t now_ms);
bool shared_cancel_arm(uint32_t id); /* false if already committed; read its ACK */
bool shared_get_arm_ack(uint32_t id, shared_arm_ack_t *out);
/* Final linearization point before PWM: cancelled/expired candidates cannot commit. */
bool shared_commit_arm_batch(const shared_arm_request_t *requests, const umb_arm_result_t *results,
                             unsigned count, const umb_outputs_t *out, const umb_arming_t *arming,
                             uint32_t now_ms);
