/* shared_state.c - see shared_state.h */
#include "shared_state.h"
#include <string.h>
#include "hardware/sync.h"
#include "umb_actuation.h"
#include "umb_tof_array.h"
#include "umb_imu_filter.h"
#include "hal_time.h"

#define ARM_QUEUE_LEN SHARED_ARM_QUEUE_LEN
#define ARM_ACK_LEN 16u
#define ARM_REQUEST_MS 60u

typedef struct {
    umb_cmd_t cmd;
    bool cmd_pending;
    shared_arm_request_t arm_queue[ARM_QUEUE_LEN];
    shared_arm_ack_t arm_ack[ARM_ACK_LEN];
    uint32_t next_arm_id;
    shared_arm_request_t disarm_request;
    uint8_t arm_head, arm_count;
    bool disarm_pending, arm_inflight, maintenance, maintenance_ack;
    uint32_t command_epoch, session_generation, loss_mask;
    uint32_t maintenance_id, settings_revision;
    bool wifi_up, agent_up;
    uint32_t core0_beat_ms;
    umb_settings_t settings;
    umb_outputs_t outputs;
    umb_arming_t arming;
    shared_sensors_t sensors;
    bool imu_cal_requested;
    int cue;
    bool cue_pending;
    int encoder_state;
} shared_t;

static shared_t s_shared;
static spin_lock_t *s_lock;

static uint32_t lock(void) {
    return spin_lock_blocking(s_lock);
}

static void unlock(uint32_t saved) {
    spin_unlock(s_lock, saved);
}

void shared_init(const umb_settings_t *s) {
    s_lock = spin_lock_instance((uint)spin_lock_claim_unused(true));
    memset(&s_shared, 0, sizeof s_shared);
    s_shared.settings = *s;
    s_shared.settings_revision = 1;
    s_shared.command_epoch = s_shared.session_generation = 1;
    s_shared.core0_beat_ms = hal_now_ms();
    s_shared.outputs.servo_us = umb_servo_us(0.0f, s);
    s_shared.outputs.esc_us = UMB_ESC_NEUTRAL_US;
    s_shared.outputs.stop_mask = UMB_STOP_NO_CMD;
    s_shared.arming = umb_arming_init((umb_arm_mode_t)s->arming_mode);
    s_shared.outputs.armed = umb_arming_is_armed(s_shared.arming);
    for (int i = 0; i < 6; i++) s_shared.sensors.tof_mm[i] = UMB_TOF_INVALID_MM;
    s_shared.encoder_state = SHARED_ENC_STARTING;
}

static void invalidate_commands(void) {
    s_shared.command_epoch++;
    s_shared.cmd_pending = false;
}

bool shared_post_cmd(const umb_cmd_t *cmd) {
    if (!cmd || cmd->source != UMB_SRC_CONSOLE) return false;
    uint32_t irq = lock();
    bool ok = !s_shared.maintenance && !s_shared.disarm_pending;
    if (ok) { s_shared.cmd = *cmd; s_shared.cmd_pending = true; }
    unlock(irq);
    return ok;
}

bool shared_post_ros_cmd(const umb_cmd_t *cmd, uint32_t generation) {
    if (!cmd || cmd->source != UMB_SRC_ROS) return false;
    uint32_t irq = lock();
    bool ok = s_shared.wifi_up && s_shared.agent_up &&
              generation == s_shared.session_generation &&
              !s_shared.maintenance && !s_shared.disarm_pending;
    if (ok) { s_shared.cmd = *cmd; s_shared.cmd_pending = true; }
    unlock(irq);
    return ok;
}

static void cancel_arm_requests(void) {
    s_shared.arm_head = s_shared.arm_count = 0;
    for (unsigned i = 0; i < ARM_ACK_LEN; i++) {
        shared_arm_ack_t *ack = &s_shared.arm_ack[i];
        if (ack->id && (ack->state == SHARED_REQUEST_PENDING || ack->state == SHARED_REQUEST_APPLYING))
            ack->state = SHARED_REQUEST_CANCELLED;
    }
}

static bool disarm_kind(int kind) {
    return kind == SHARED_ARM_DISARM || kind == SHARED_ARM_ROS_DISARM;
}

bool shared_submit_arm(int kind, uint32_t generation, uint32_t now_ms, uint32_t *id) {
    if (!id || kind < SHARED_ARM_CAR || kind > SHARED_ARM_DISARM) return false;
    uint32_t irq = lock();
    bool disarm = disarm_kind(kind);
    bool ready = generation == 0 || (generation == s_shared.session_generation &&
                                    s_shared.wifi_up && s_shared.agent_up);
    if (disarm) {
        cancel_arm_requests();
        s_shared.disarm_pending = true;
        invalidate_commands();
        uint32_t token = s_shared.next_arm_id + 1u;
        if (!token) token = 1;
        s_shared.next_arm_id = token;
        s_shared.disarm_request = (shared_arm_request_t){SHARED_ARM_DISARM, token,
            s_shared.session_generation, now_ms + ARM_REQUEST_MS};
        s_shared.arm_ack[token % ARM_ACK_LEN] = (shared_arm_ack_t){.id = token, .state = SHARED_REQUEST_PENDING};
        *id = token;
        unlock(irq);
        return true;
    }
    uint32_t token = s_shared.next_arm_id + 1u;
    if (!token) token = 1;
    shared_arm_ack_t *ack = &s_shared.arm_ack[token % ARM_ACK_LEN];
    bool busy = ack->id && (ack->state == SHARED_REQUEST_PENDING || ack->state == SHARED_REQUEST_APPLYING);
    bool ok = ready && !busy && !s_shared.maintenance && !s_shared.disarm_pending &&
              s_shared.arm_count < ARM_QUEUE_LEN;
    if (ok) {
        s_shared.next_arm_id = token;
        *ack = (shared_arm_ack_t){.id = token, .state = SHARED_REQUEST_PENDING};
        uint8_t tail = (uint8_t)((s_shared.arm_head + s_shared.arm_count) % ARM_QUEUE_LEN);
        s_shared.arm_queue[tail] = (shared_arm_request_t){kind, token,
            s_shared.session_generation, now_ms + ARM_REQUEST_MS};
        s_shared.arm_count++;
        *id = token;
    }
    unlock(irq);
    return ok;
}

bool shared_post_arm_request(int kind) {
    uint32_t id;
    return shared_submit_arm(kind, 0, hal_now_ms(), &id);
}

bool shared_take_arm_record(shared_arm_request_t *out, uint32_t now_ms) {
    uint32_t irq = lock();
    if (s_shared.disarm_pending) {
        *out = s_shared.disarm_request;
        if (!out->id) out->kind = SHARED_ARM_DISARM; /* network-loss disarm has no RPC token */
        else s_shared.arm_ack[out->id % ARM_ACK_LEN].state = SHARED_REQUEST_APPLYING;
        s_shared.disarm_request.id = 0;
        s_shared.disarm_pending = false;
        s_shared.arm_inflight = true;
        unlock(irq);
        return true;
    }
    while (s_shared.arm_count) {
        *out = s_shared.arm_queue[s_shared.arm_head];
        s_shared.arm_head = (uint8_t)((s_shared.arm_head + 1u) % ARM_QUEUE_LEN);
        s_shared.arm_count--;
        shared_arm_ack_t *ack = &s_shared.arm_ack[out->id % ARM_ACK_LEN];
        if (ack->id != out->id || ack->state != SHARED_REQUEST_PENDING) continue;
        if ((int32_t)(now_ms - out->deadline_ms) >= 0 ||
            out->generation != s_shared.session_generation || s_shared.maintenance) {
            ack->state = SHARED_REQUEST_CANCELLED;
            continue;
        }
        ack->state = SHARED_REQUEST_APPLYING;
        s_shared.arm_inflight = true;
        unlock(irq);
        return true;
    }
    unlock(irq);
    return false;
}

bool shared_cancel_arm(uint32_t id) {
    if (!id) return false;
    uint32_t irq = lock();
    shared_arm_ack_t *ack = &s_shared.arm_ack[id % ARM_ACK_LEN];
    bool ok = ack->id == id && (ack->state == SHARED_REQUEST_PENDING || ack->state == SHARED_REQUEST_APPLYING);
    if (ok) ack->state = SHARED_REQUEST_CANCELLED;
    unlock(irq);
    return ok;
}

bool shared_get_arm_ack(uint32_t id, shared_arm_ack_t *out) {
    if (!id || !out) return false;
    uint32_t irq = lock();
    shared_arm_ack_t *ack = &s_shared.arm_ack[id % ARM_ACK_LEN];
    bool ok = ack->id == id;
    if (ok) *out = *ack;
    unlock(irq);
    return ok;
}

bool shared_commit_arm_batch(const shared_arm_request_t *requests, const umb_arm_result_t *results,
                             unsigned count, const umb_outputs_t *out, const umb_arming_t *arming,
                             uint32_t now_ms) {
    if (count > ARM_QUEUE_LEN) return false;
    uint32_t irq = lock();
    bool ok = true;
    for (unsigned i = 0; i < count; i++) {
        if (!requests[i].id || disarm_kind(requests[i].kind)) continue; /* disarm always applies */
        shared_arm_ack_t *ack = &s_shared.arm_ack[requests[i].id % ARM_ACK_LEN];
        if (ack->id != requests[i].id || ack->state != SHARED_REQUEST_APPLYING ||
            (int32_t)(now_ms - requests[i].deadline_ms) >= 0 ||
            requests[i].generation != s_shared.session_generation ||
            s_shared.maintenance || s_shared.disarm_pending) ok = false;
    }
    for (unsigned i = 0; i < count; i++) {
        if (!requests[i].id) continue;
        shared_arm_ack_t *ack = &s_shared.arm_ack[requests[i].id % ARM_ACK_LEN];
        if (ack->id != requests[i].id || ack->state == SHARED_REQUEST_CANCELLED) continue;
        ack->state = ok ? SHARED_REQUEST_DONE : SHARED_REQUEST_CANCELLED;
        if (ok) {
            ack->armed = out->armed; ack->car_key = arming->car_key; ack->ros_key = arming->ros_key;
            ack->result = results[i];
        }
    }
    unlock(irq);
    return ok;
}

static void notify_loss(uint32_t mask) {
    mask &= UMB_STOP_DISARM_MASK;
    if (!mask) return;
    s_shared.loss_mask |= mask;
    if (++s_shared.session_generation == 0) s_shared.session_generation = 1;
    s_shared.disarm_pending = true;
    s_shared.disarm_request = (shared_arm_request_t){.kind = SHARED_ARM_DISARM};
    cancel_arm_requests();
    invalidate_commands();
}

void shared_notify_network_loss(uint32_t mask) {
    uint32_t irq = lock();
    notify_loss(mask);
    unlock(irq);
}

uint32_t shared_session_generation(void) {
    uint32_t irq = lock();
    uint32_t generation = s_shared.session_generation;
    unlock(irq);
    return generation;
}

void shared_set_link(bool wifi_up, bool agent_up) {
    uint32_t irq = lock();
    uint32_t loss = 0;
    if (s_shared.wifi_up && !wifi_up) loss |= UMB_STOP_WIFI;
    if (s_shared.agent_up && !agent_up) loss |= UMB_STOP_AGENT;
    notify_loss(loss);
    s_shared.wifi_up = wifi_up;
    s_shared.agent_up = agent_up;
    unlock(irq);
}

void shared_get_link(bool *wifi_up, bool *agent_up) {
    uint32_t irq = lock();
    *wifi_up = s_shared.wifi_up;
    *agent_up = s_shared.agent_up;
    unlock(irq);
}

void shared_core0_beat(uint32_t now_ms) {
    uint32_t irq = lock();
    s_shared.core0_beat_ms = now_ms;
    unlock(irq);
}

uint32_t shared_core0_beat_ms(void) {
    uint32_t irq = lock();
    uint32_t v = s_shared.core0_beat_ms;
    unlock(irq);
    return v;
}

uint32_t shared_settings_snapshot(umb_settings_t *out) {
    uint32_t irq = lock();
    *out = s_shared.settings;
    uint32_t revision = s_shared.settings_revision;
    unlock(irq);
    return revision;
}

bool shared_commit_settings(const umb_settings_t *s, uint32_t revision, bool mode_only) {
    if (!s) return false;
    umb_settings_t clean = umb_settings_sanitize(*s);
    if (!umb_settings_equal(s, &clean)) return false;
    uint32_t irq = lock();
    bool allowed = !s_shared.maintenance && !s_shared.arm_inflight &&
                   !s_shared.disarm_pending && s_shared.arm_count == 0 &&
                   s_shared.settings_revision == revision;
    if (mode_only) {
        clean.arming_mode = s_shared.settings.arming_mode;
        allowed = allowed && umb_settings_equal(&clean, &s_shared.settings);
    } else allowed = allowed && !s_shared.outputs.armed;
    if (allowed) {
        bool mode_changed = s->arming_mode != s_shared.settings.arming_mode;
        s_shared.settings = *s;
        s_shared.settings_revision++;
        if (mode_changed) {
            s_shared.disarm_pending = true;
            invalidate_commands();
        }
    }
    unlock(irq);
    return allowed;
}

void shared_get_settings(umb_settings_t *out) {
    uint32_t irq = lock();
    *out = s_shared.settings;
    unlock(irq);
}

void shared_publish_outputs(const umb_outputs_t *o) {
    uint32_t irq = lock();
    s_shared.outputs = *o;
    s_shared.arm_inflight = false;
    unlock(irq);
}

void shared_get_outputs(umb_outputs_t *out) {
    uint32_t irq = lock();
    *out = s_shared.outputs;
    unlock(irq);
}

void shared_publish_arming(const umb_arming_t *a) {
    uint32_t irq = lock();
    s_shared.arming = *a;
    unlock(irq);
}

void shared_get_arming(umb_arming_t *out) {
    uint32_t irq = lock();
    *out = s_shared.arming;
    unlock(irq);
}

bool shared_settings_write_allowed(void) {
    uint32_t irq = lock();
    /* A queued key press may arm the car on the next control step. */
    bool allowed = !s_shared.outputs.armed && s_shared.arm_count == 0u &&
                   !s_shared.arm_inflight && !s_shared.disarm_pending && !s_shared.maintenance;
    unlock(irq);
    return allowed;
}

void shared_publish_sensors(const shared_sensors_t *s) {
    uint32_t irq = lock();
    s_shared.sensors = *s;
    unlock(irq);
}

void shared_get_sensors(shared_sensors_t *out) {
    uint32_t irq = lock();
    *out = s_shared.sensors;
    unlock(irq);
    uint32_t now = hal_now_ms();
    out->tof_online = 0;
    for (int i = 0; i < UMB_TOF_COUNT; i++) {
        bool fresh = (out->tof_online_mask & (1u << i)) &&
                     (int32_t)(now - out->tof_stamp_ms[i]) <= (int32_t)UMB_TOF_STALE_MS;
        if (fresh) out->tof_online++;
        else {
            out->tof_online_mask &= (uint8_t)~(1u << i);
            out->tof_mm[i] = UMB_TOF_INVALID_MM;
        }
    }
    out->imu_ok = out->imu_ok &&
                  (int32_t)(now - out->imu_stamp_ms) <= (int32_t)UMB_IMU_STALE_MS;
}

void shared_request_imu_cal(void) {
    uint32_t irq = lock();
    s_shared.imu_cal_requested = true;
    unlock(irq);
}

bool shared_take_imu_cal(void) {
    uint32_t irq = lock();
    bool req = s_shared.imu_cal_requested;
    s_shared.imu_cal_requested = false;
    unlock(irq);
    return req;
}

void shared_post_cue(int cue) {
    uint32_t irq = lock();
    s_shared.cue = cue;
    s_shared.cue_pending = true;
    unlock(irq);
}

bool shared_take_cue(int *cue) {
    uint32_t irq = lock();
    bool pending = s_shared.cue_pending;
    if (pending) *cue = s_shared.cue;
    s_shared.cue_pending = false;
    unlock(irq);
    return pending;
}

void shared_set_encoder_state(int state) {
    uint32_t irq = lock();
    s_shared.encoder_state = state;
    unlock(irq);
}

int shared_encoder_state(void) {
    uint32_t irq = lock();
    int v = s_shared.encoder_state;
    unlock(irq);
    return v;
}

void shared_take_control_mailbox(shared_control_mailbox_t *out) {
    uint32_t irq = lock();
    out->cmd = s_shared.cmd;
    out->cmd_pending = s_shared.cmd_pending;
    out->command_epoch = s_shared.command_epoch;
    out->loss_mask = s_shared.loss_mask;
    out->wifi_up = s_shared.wifi_up;
    out->agent_up = s_shared.agent_up;
    out->beat_ms = s_shared.core0_beat_ms;
    out->maintenance = s_shared.maintenance;
    out->maintenance_id = s_shared.maintenance_id;
    s_shared.cmd_pending = false;
    s_shared.loss_mask = 0;
    unlock(irq);
}

bool shared_begin_maintenance(uint32_t *id) {
    if (!id) return false;
    uint32_t irq = lock();
    bool ok = !s_shared.maintenance && !s_shared.outputs.armed &&
              !s_shared.arm_inflight && !s_shared.disarm_pending && s_shared.arm_count == 0;
    if (ok) {
        s_shared.maintenance = true;
        s_shared.maintenance_ack = false;
        s_shared.maintenance_id++;
        *id = s_shared.maintenance_id;
        invalidate_commands();
    }
    unlock(irq);
    return ok;
}

void shared_ack_maintenance(uint32_t id) {
    uint32_t irq = lock();
    if (s_shared.maintenance && s_shared.maintenance_id == id &&
        !s_shared.outputs.armed && !s_shared.outputs.motion_allowed &&
        s_shared.outputs.esc_us == UMB_ESC_NEUTRAL_US)
        s_shared.maintenance_ack = true;
    unlock(irq);
}

bool shared_maintenance_acknowledged(uint32_t id) {
    uint32_t irq = lock();
    bool ack = s_shared.maintenance && s_shared.maintenance_id == id && s_shared.maintenance_ack;
    unlock(irq);
    return ack;
}

void shared_end_maintenance(uint32_t id) {
    uint32_t irq = lock();
    if (s_shared.maintenance && s_shared.maintenance_id == id) {
        s_shared.maintenance = s_shared.maintenance_ack = false;
        invalidate_commands();
    }
    unlock(irq);
}
