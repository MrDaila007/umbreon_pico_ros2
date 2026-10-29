/* umb_arm_reply.c - see umb_arm_reply.h */
#include "umb_arm_reply.h"
#include <stddef.h>

const char *umb_arm_reply_message(umb_arm_reply_t reply) {
    switch (reply) {
    case UMB_ARM_REPLY_ARMED:               return "armed";
    case UMB_ARM_REPLY_WAITING_CAR:         return "waiting for car arm";
    case UMB_ARM_REPLY_WAITING_ROS:         return "waiting for ros arm";
    case UMB_ARM_REPLY_DISARMED:            return "disarmed";
    case UMB_ARM_REPLY_REFUSED_MODE:        return "refused: mode";
    case UMB_ARM_REPLY_REFUSED_SESSION:     return "refused: session";
    case UMB_ARM_REPLY_REFUSED_MAINTENANCE: return "refused: maintenance";
    case UMB_ARM_REPLY_BUSY_TIMEOUT:        break;
    }
    return "busy: timeout";
}

bool umb_arm_reply_success(umb_arm_reply_t reply) {
    return reply == UMB_ARM_REPLY_ARMED || reply == UMB_ARM_REPLY_WAITING_CAR ||
           reply == UMB_ARM_REPLY_WAITING_ROS || reply == UMB_ARM_REPLY_DISARMED;
}

umb_arm_reply_t umb_arm_reply_for_result(umb_arm_result_t result) {
    switch (result) {
    case UMB_ARM_OK:            return UMB_ARM_REPLY_ARMED;
    case UMB_ARM_WAITING_CAR:   return UMB_ARM_REPLY_WAITING_CAR;
    case UMB_ARM_WAITING_ROS:   return UMB_ARM_REPLY_WAITING_ROS;
    case UMB_ARM_REJECTED_MODE: return UMB_ARM_REPLY_REFUSED_MODE;
    }
    return UMB_ARM_REPLY_BUSY_TIMEOUT;
}

umb_arm_reply_t umb_arm_reply_refusal(bool session_ready, bool maintenance) {
    if (!session_ready) return UMB_ARM_REPLY_REFUSED_SESSION;
    if (maintenance) return UMB_ARM_REPLY_REFUSED_MAINTENANCE;
    return UMB_ARM_REPLY_BUSY_TIMEOUT;
}

static umb_arm_wait_decision_t decision(umb_arm_wait_step_t step, umb_arm_reply_t reply) {
    umb_arm_wait_decision_t d = {step, reply};
    return d;
}

static umb_arm_wait_decision_t decide_disarm(const umb_arm_wait_view_t *v) {
    bool waiting = v->ack == UMB_ARM_ACK_PENDING && v->elapsed_ms < UMB_ARM_WAIT_MS;
    return decision(waiting ? UMB_ARM_WAIT_POLL : UMB_ARM_WAIT_REPLY, UMB_ARM_REPLY_DISARMED);
}

umb_arm_wait_decision_t umb_arm_wait_decide(const umb_arm_wait_view_t *v) {
    if (v == NULL) return decision(UMB_ARM_WAIT_REPLY, UMB_ARM_REPLY_BUSY_TIMEOUT);
    if (!v->arm) return decide_disarm(v);
    umb_arm_reply_t refusal = umb_arm_reply_refusal(v->session_ready, v->maintenance);
    switch (v->ack) {
    case UMB_ARM_ACK_DONE:
        return decision(UMB_ARM_WAIT_REPLY, umb_arm_reply_for_result(v->result));
    case UMB_ARM_ACK_PENDING:
        if (v->cancel_attempted) return decision(UMB_ARM_WAIT_REPLY, refusal);
        if (!v->session_ready || v->maintenance || v->elapsed_ms >= UMB_ARM_WAIT_MS)
            return decision(UMB_ARM_WAIT_CANCEL, refusal);
        return decision(UMB_ARM_WAIT_POLL, refusal);
    case UMB_ARM_ACK_CANCELLED:
    case UMB_ARM_ACK_MISSING:
        break;
    }
    return decision(UMB_ARM_WAIT_REPLY, refusal);
}

uint32_t umb_arm_wait_sleep_ms(uint32_t elapsed_ms) {
    if (elapsed_ms >= UMB_ARM_WAIT_MS) return 0;
    uint32_t left = UMB_ARM_WAIT_MS - elapsed_ms;
    return left < UMB_ARM_POLL_MS ? left : UMB_ARM_POLL_MS;
}
