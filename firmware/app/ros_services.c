/* ros_services.c - see ros_services.h. Only shared-state copies and the injected
 * clock are used; no shared spinlock is held across a sleep. */
#include "ros_services.h"
#include <stddef.h>
#include "shared_state.h"

static ros_arm_reply_t make_reply(umb_arm_reply_t r) {
    ros_arm_reply_t out = {umb_arm_reply_success(r), umb_arm_reply_message(r)};
    return out;
}

static umb_arm_ack_state_t read_ack(uint32_t id, umb_arm_result_t *result) {
    shared_arm_ack_t ack;
    if (!shared_get_arm_ack(id, &ack)) return UMB_ARM_ACK_MISSING;
    switch (ack.state) {
    case SHARED_REQUEST_PENDING:
    case SHARED_REQUEST_APPLYING: return UMB_ARM_ACK_PENDING;
    case SHARED_REQUEST_DONE:     *result = ack.result; return UMB_ARM_ACK_DONE;
    case SHARED_REQUEST_CANCELLED: break;
    }
    return UMB_ARM_ACK_CANCELLED;
}

/* The ACK is read before the time sample: a commit racing the timeout is then
 * seen through the failed cancel and reported, never as a failure. */
static umb_arm_wait_view_t observe(bool arm, uint32_t id, uint32_t generation,
                                   uint32_t start_ms, const ros_arm_clock_t *clock) {
    umb_arm_wait_view_t v = {.arm = arm, .result = UMB_ARM_OK};
    v.ack = read_ack(id, &v.result);
    v.session_ready = shared_session_ready(generation);
    v.maintenance = shared_maintenance_active();
    v.elapsed_ms = clock->now_ms(clock->ctx) - start_ms;
    return v;
}

static umb_arm_reply_t wait_for_ack(bool arm, uint32_t id, uint32_t generation,
                                    uint32_t start_ms, const ros_arm_clock_t *clock) {
    bool cancel_attempted = false;
    /* Each poll sleeps >= 1 ms, so this cap only matters for a non-advancing clock. */
    for (uint32_t polls = 0;; polls++) {
        umb_arm_wait_view_t v = observe(arm, id, generation, start_ms, clock);
        v.cancel_attempted = cancel_attempted;
        if (polls > UMB_ARM_WAIT_MS) v.elapsed_ms = UMB_ARM_WAIT_MS;
        umb_arm_wait_decision_t d = umb_arm_wait_decide(&v);
        if (d.step == UMB_ARM_WAIT_REPLY) return d.reply;
        if (d.step == UMB_ARM_WAIT_CANCEL) {
            /* false means control already committed or cancelled it: re-read. */
            (void)shared_cancel_arm(id);
            cancel_attempted = true;
            continue;
        }
        clock->sleep_ms(clock->ctx, umb_arm_wait_sleep_ms(v.elapsed_ms));
    }
}

static umb_arm_reply_t current_refusal(uint32_t generation) {
    return umb_arm_reply_refusal(shared_session_ready(generation), shared_maintenance_active());
}

ros_arm_reply_t ros_services_arm(bool arm, uint32_t generation, const ros_arm_clock_t *clock) {
    bool have_clock = clock != NULL && clock->now_ms != NULL && clock->sleep_ms != NULL;
    if (arm) {
        if (!have_clock) return make_reply(UMB_ARM_REPLY_BUSY_TIMEOUT);
        umb_arm_reply_t refusal = current_refusal(generation);
        if (refusal != UMB_ARM_REPLY_BUSY_TIMEOUT) return make_reply(refusal);
    }
    uint32_t start_ms = have_clock ? clock->now_ms(clock->ctx) : 0u;
    uint32_t id = 0;
    /* Disarm is sticky priority and cannot be refused, regardless of session or
     * maintenance; it drops queued arm requests and clears both keys in control. */
    int kind = arm ? SHARED_ARM_ROS : SHARED_ARM_ROS_DISARM;
    if (!shared_submit_arm(kind, generation, start_ms, &id))
        return make_reply(arm ? current_refusal(generation) : UMB_ARM_REPLY_DISARMED);
    if (!have_clock) return make_reply(UMB_ARM_REPLY_DISARMED);
    return make_reply(wait_for_ack(arm, id, generation, start_ms, clock));
}
