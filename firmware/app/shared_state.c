/* shared_state.c - see shared_state.h */
#include "shared_state.h"
#include <string.h>
#include "hardware/sync.h"
#include "umb_actuation.h"
#include "hal_time.h"

#define ARM_QUEUE_LEN 8u

typedef struct {
    umb_cmd_t cmd;
    bool cmd_pending;
    uint8_t arm_queue[ARM_QUEUE_LEN];
    uint8_t arm_head, arm_count;
    bool wifi_up, agent_up;
    uint32_t core0_beat_ms;
    umb_settings_t settings;
    umb_outputs_t outputs;
    umb_arming_t arming;
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
    s_shared.core0_beat_ms = hal_now_ms();
    s_shared.outputs.servo_us = umb_servo_us(0.0f, s);
    s_shared.outputs.esc_us = UMB_ESC_NEUTRAL_US;
    s_shared.outputs.stop_mask = UMB_STOP_NO_CMD;
    s_shared.arming = umb_arming_init((umb_arm_mode_t)s->arming_mode);
}

void shared_post_cmd(const umb_cmd_t *cmd) {
    uint32_t irq = lock();
    s_shared.cmd = *cmd;
    s_shared.cmd_pending = true;
    unlock(irq);
}

bool shared_take_cmd(umb_cmd_t *out) {
    uint32_t irq = lock();
    bool pending = s_shared.cmd_pending;
    if (pending) *out = s_shared.cmd;
    s_shared.cmd_pending = false;
    unlock(irq);
    return pending;
}

void shared_post_arm_request(int kind) {
    if (kind < SHARED_ARM_CAR || kind > SHARED_ARM_DISARM) return;
    uint32_t irq = lock();
    if (s_shared.arm_count < ARM_QUEUE_LEN) {
        uint8_t tail = (uint8_t)((s_shared.arm_head + s_shared.arm_count) % ARM_QUEUE_LEN);
        s_shared.arm_queue[tail] = (uint8_t)kind;
        s_shared.arm_count++;
    }
    unlock(irq);
}

bool shared_take_arm_request(int *kind) {
    uint32_t irq = lock();
    bool have = s_shared.arm_count > 0u;
    if (have) {
        *kind = s_shared.arm_queue[s_shared.arm_head];
        s_shared.arm_head = (uint8_t)((s_shared.arm_head + 1u) % ARM_QUEUE_LEN);
        s_shared.arm_count--;
    }
    unlock(irq);
    return have;
}

void shared_set_link(bool wifi_up, bool agent_up) {
    uint32_t irq = lock();
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

void shared_set_settings(const umb_settings_t *s) {
    uint32_t irq = lock();
    s_shared.settings = *s;
    unlock(irq);
}

void shared_get_settings(umb_settings_t *out) {
    uint32_t irq = lock();
    *out = s_shared.settings;
    unlock(irq);
}

void shared_publish_outputs(const umb_outputs_t *o) {
    uint32_t irq = lock();
    s_shared.outputs = *o;
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
    bool allowed = !s_shared.outputs.armed && s_shared.arm_count == 0u;
    unlock(irq);
    return allowed;
}
