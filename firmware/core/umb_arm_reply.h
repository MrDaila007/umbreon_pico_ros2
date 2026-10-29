/* umb_arm_reply.h - pure /umbreon/arm (std_srvs/SetBool) reply selection and
 * bounded wait decisions (C11, no SDK/RTOS/ROS). The service handler polls the
 * control-task ACK and asks this module what to do next. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "umb_arming.h"

#define UMB_ARM_WAIT_MS 60u  /* service reply bound == shared request deadline */
#define UMB_ARM_POLL_MS 5u   /* ACK poll period while waiting */

typedef enum {
    UMB_ARM_REPLY_ARMED,
    UMB_ARM_REPLY_WAITING_CAR,
    UMB_ARM_REPLY_WAITING_ROS,
    UMB_ARM_REPLY_DISARMED,
    UMB_ARM_REPLY_REFUSED_MODE,
    UMB_ARM_REPLY_REFUSED_SESSION,
    UMB_ARM_REPLY_REFUSED_MAINTENANCE,
    UMB_ARM_REPLY_BUSY_TIMEOUT
} umb_arm_reply_t;

/* Exact SetBool message; unknown values read as "busy: timeout". */
const char *umb_arm_reply_message(umb_arm_reply_t reply);
/* true only when a key was accepted (armed / waiting ...) or for disarmed. */
bool umb_arm_reply_success(umb_arm_reply_t reply);
/* Control's committed arming result for an arm(true) request. */
umb_arm_reply_t umb_arm_reply_for_result(umb_arm_result_t result);
/* Why an arm(true) was not (or no longer) accepted: session, then maintenance, else busy. */
umb_arm_reply_t umb_arm_reply_refusal(bool session_ready, bool maintenance);

typedef enum { UMB_ARM_ACK_MISSING, UMB_ARM_ACK_PENDING, UMB_ARM_ACK_DONE,
               UMB_ARM_ACK_CANCELLED } umb_arm_ack_state_t;
typedef struct {
    bool arm;                 /* requested SetBool data */
    umb_arm_ack_state_t ack;  /* PENDING covers queued and taken-but-uncommitted */
    umb_arm_result_t result;  /* valid when ack == DONE */
    bool session_ready;       /* request's generation is current and link ready */
    bool maintenance;
    bool cancel_attempted;    /* ack was re-read after a cancel attempt */
    uint32_t elapsed_ms;      /* since submit, local monotonic clock */
} umb_arm_wait_view_t;
typedef enum { UMB_ARM_WAIT_POLL, UMB_ARM_WAIT_CANCEL, UMB_ARM_WAIT_REPLY } umb_arm_wait_step_t;
typedef struct { umb_arm_wait_step_t step; umb_arm_reply_t reply; } umb_arm_wait_decision_t;

/* A committed ACK always wins. arm(true): stale session, maintenance or reaching
 * the bound -> CANCEL; the caller cancels, re-reads the ACK and decides again.
 * arm(false) is never refused or cancelled: the sticky disarm still applies. */
umb_arm_wait_decision_t umb_arm_wait_decide(const umb_arm_wait_view_t *view);
/* Next sleep: one poll period, never past the bound (0 once reached). */
uint32_t umb_arm_wait_sleep_ms(uint32_t elapsed_ms);
