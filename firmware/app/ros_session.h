/* ros_session.h - micro-ROS session lifecycle for the core-0 network owner:
 * drives the pure umb_session policy, the entity tracker and the backend.
 *
 * Per step at most one bounded blocking backend operation besides the executor
 * spin: one entity creation (<= ROS_CREATE_TIMEOUT_MS), one ping or one time
 * sync attempt (<= 20 ms). Readiness is reported only once every entity and the
 * executor exist; callbacks carry generation 0 until then.
 *
 * Loss order (caller): when out.teardown is set, first publish agent_up=false
 * (latching the loss event in shared state), then call ros_session_teardown(). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "ros_entities.h"
#include "umb_net_fsm.h"
#include "umb_telemetry.h"

#define ROS_SESSION_ERROR_LIMIT 3u   /* consecutive actual rcl/transport errors -> loss */
#define ROS_SYNC_TIMEOUT_MS 20u
#define ROS_SYNC_RETRY_MS 100u       /* until synchronized, and after a failed resync */
#define ROS_SYNC_PERIOD_MS 30000u

typedef struct {
    uint32_t now_ms;
    bool link_up, endpoint_selected;
    ros_session_config_t endpoint; /* selected agent, domain and client key */
    uint32_t generation;           /* current shared session generation */
    bool open_allowed;             /* false while armed: no (blocking) session open */
} ros_session_in_t;

typedef struct {
    bool ready;    /* agent_up may be true */
    bool loss;     /* session dropped this step (ready already false) */
    bool teardown; /* caller latches loss, then calls ros_session_teardown() */
} ros_session_out_t;

typedef struct {
    umb_session_t fsm;
    umb_session_in_t pulse; /* results of this step's action for the next step */
    ros_entities_t entities;
    ros_session_config_t cfg;
    umb_time_sync_t sync;   /* invalidated for every new session */
    uint32_t next_sync_ms, last_sync_ms;
    uint32_t generation;    /* nonzero only while ready */
    uint8_t errors_in_row;
    bool ready;
    uint32_t sessions, losses, create_failures, transport_failures;
    uint32_t ping_failures, sync_failures, teardowns;
} ros_session_t;

void ros_session_init(ros_session_t *s);
ros_session_out_t ros_session_step(ros_session_t *s, const ros_session_in_t *in, const ros_backend_t *b);
/* Drops readiness/generation, finalizes every created entity in reverse order
 * (destroy-session timeout zeroed first) and releases the transport, even with a
 * dead agent. Idempotent. False: socket still owned; the session stays busy. */
bool ros_session_teardown(ros_session_t *s, const ros_backend_t *b);
bool ros_session_busy(const ros_session_t *s); /* entities, failed partial set or socket owned */
