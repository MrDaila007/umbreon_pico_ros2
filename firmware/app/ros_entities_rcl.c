/* ros_entities_rcl.c - rcl/rclc implementation of ros_backend_t (see
 * ros_entities.h for the contract). One static instance; core 0 only.
 * Allocation ownership: rcl/rmw objects allocate through the installed rcl
 * allocator and are released by their fini in reverse creation order; all
 * message strings/sequences are fixed buffers in ros_messages_t, never passed
 * to generated init/fini. */
#include "ros_entities_rcl.h"
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "hal_time.h"
#include "hal_udp_transport.h"
#include "ros_commands.h"
#include "ros_messages.h"
#include "ros_params_rcl.h"
#include "ros_services.h"
#include "shared_state.h"

_Static_assert(ROS_PARAM_SERVER_HANDLES == RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
               "executor handle budget must use the pinned parameter server count");

#define PUBLISHERS (ROS_ENT_PUB_DIAGNOSTICS - ROS_ENT_PUB_RANGE_HR + 1)

typedef struct {
    rcl_allocator_t allocator;
    rcl_init_options_t init_options;
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t pub[PUBLISHERS];
    rcl_subscription_t sub[ROS_COMMAND_TOPICS];
    rcl_service_t arm;
    rcl_timer_t timer;
    rclc_executor_t executor;
    hal_udp_transport_t transport;
    uxrCustomTransport release; /* only .args is used: closes an owned socket */
    ros_messages_t msgs;
    ros_telemetry_t telemetry;
    const ros_session_t *session;
    uint32_t generation, fini_errors;
} rcl_backend_t;

static rcl_backend_t s_rcl;

/* ---- callbacks (run inside rclc_executor_spin_some, core 0) ---------------- */

static void on_command(umb_ros_command_topic_t topic, const void *msg) {
    umb_twist_fields_t fields;
    ros_messages_twist(msg, &fields);
    (void)ros_commands_post(topic, &fields, s_rcl.generation);
}
static void cmd_vel_cb(const void *msg) { on_command(UMB_ROS_CMD_VEL, msg); }
static void cmd_teleop_cb(const void *msg) { on_command(UMB_ROS_CMD_TELEOP, msg); }
static void cmd_raw_cb(const void *msg) { on_command(UMB_ROS_CMD_RAW, msg); }
static const rclc_subscription_callback_t k_cmd_cb[ROS_COMMAND_TOPICS] = {cmd_vel_cb, cmd_teleop_cb, cmd_raw_cb};

static uint32_t clock_now(void *ctx) { (void)ctx; return hal_now_ms(); }
static void clock_sleep(void *ctx, uint32_t ms) {
    (void)ctx;
    TickType_t ticks = (TickType_t)(((uint64_t)ms * configTICK_RATE_HZ + 999u) / 1000u);
    vTaskDelay(ticks ? ticks : 1); /* always yield at least one tick */
}
static const ros_arm_clock_t k_arm_clock = {clock_now, clock_sleep, NULL};

/* Blocks at most ROS_ARM_WAIT_MS (60 ms) for the control ACK; rclc then sends
 * the response, whose reliable confirm is bounded by ROS_SERVICE_REPLY_TIMEOUT_MS
 * (20 ms). The owner loop budgets both (beat before/after the step). */
static void arm_cb(const void *request, void *response) {
    (void)response; /* == &s_rcl.msgs.arm_response */
    const std_srvs__srv__SetBool_Request *req = request;
    ros_arm_reply_t r = ros_services_arm(req->data, s_rcl.generation, &k_arm_clock);
    ros_messages_arm_reply(&s_rcl.msgs, r.success, r.message);
}

static bool publish_cb(void *ctx, ros_entity_id_t id, const void *msg) {
    (void)ctx;
    rcl_ret_t rc = rcl_publish(&s_rcl.pub[id - ROS_ENT_PUB_RANGE_HR], msg, NULL);
    if (rc != RCL_RET_OK) rcl_reset_error();
    return rc == RCL_RET_OK;
}

static void timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (!timer) return;
    shared_sensors_t sensors;
    umb_outputs_t outputs;
    shared_get_sensors(&sensors);
    shared_get_outputs(&outputs);
    const ros_session_t *s = s_rcl.session;
    uint32_t now = hal_now_ms();
    ros_diag_info_t diag = {s->generation, s->sessions, s->losses,
                            s->sync.synced ? now - s->last_sync_ms : 0u};
    (void)ros_telemetry_tick(&s_rcl.telemetry, &s_rcl.msgs, &sensors, outputs.speed_signed,
                             hal_now_us64() / 1000u, &s->sync, &diag, publish_cb, NULL);
}

/* ---- entity creation ----------------------------------------------------- */

static const rosidl_message_type_support_t *msg_type(ros_type_t t) {
    switch (t) {
    case ROS_TYPE_RANGE: return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range);
    case ROS_TYPE_IMU: return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    case ROS_TYPE_ODOMETRY: return ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
    case ROS_TYPE_BATTERY: return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState);
    case ROS_TYPE_DIAG_STATUS: return ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    case ROS_TYPE_DIAG_ARRAY: return ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray);
    case ROS_TYPE_TWIST: return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    default: return NULL;
    }
}

static rmw_qos_profile_t qos_for(const ros_entity_desc_t *d) {
    rmw_qos_profile_t q = rmw_qos_profile_default;
    q.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    q.depth = d->depth;
    q.reliability = d->reliability == ROS_QOS_BEST_EFFORT ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                                                          : RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    q.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    return q;
}

static bool ok(rcl_ret_t rc) {
    if (rc == RCL_RET_OK) return true;
    rcl_reset_error();
    return false;
}

/* Fresh session: known-empty messages (fresh receive/request buffers), client
 * key, domain and the custom transport bound to the selected agent. */
static bool create_init_options(const ros_session_config_t *cfg) {
    ros_messages_reset(&s_rcl.msgs);
    s_rcl.allocator = rcl_get_default_allocator();
    s_rcl.init_options = rcl_get_zero_initialized_init_options();
    if (!ok(rcl_init_options_init(&s_rcl.init_options, s_rcl.allocator))) return false;
    rmw_init_options_t *rmw = rcl_init_options_get_rmw_init_options(&s_rcl.init_options);
    hal_udp_transport_init(&s_rcl.transport, cfg->agent_ip, cfg->agent_port);
    bool done = ok(rcl_init_options_set_domain_id(&s_rcl.init_options, cfg->domain_id)) && rmw &&
        rmw_uros_options_set_client_key(cfg->client_key, rmw) == RMW_RET_OK &&
        rmw_uros_options_set_custom_transport(false, &s_rcl.transport, hal_udp_transport_open,
            hal_udp_transport_close, hal_udp_transport_write, hal_udp_transport_read, rmw) == RMW_RET_OK;
    if (!done) { (void)ok(rcl_init_options_fini(&s_rcl.init_options)); rcl_reset_error(); }
    return done;
}

/* uxr_create_session inside rcl_init retries for seconds on a silent agent, so
 * a fresh bounded ping first narrows that window to a few milliseconds. */
static bool create_context(void) {
    rmw_init_options_t *rmw = rcl_init_options_get_rmw_init_options(&s_rcl.init_options);
    if (rmw_uros_ping_agent_options((int)ROS_OPEN_PING_TIMEOUT_MS, 1, rmw) != RMW_RET_OK) return false;
    s_rcl.support.context = rcl_get_zero_initialized_context();
    if (!ok(rcl_init(0, NULL, &s_rcl.init_options, &s_rcl.support.context))) return false;
    s_rcl.support.allocator = &s_rcl.allocator;
    rmw_context_t *ctx = rcl_context_get_rmw_context(&s_rcl.support.context);
    if (ctx) (void)rmw_uros_set_context_entity_creation_session_timeout(ctx, (int)ROS_CREATE_TIMEOUT_MS);
    return true;
}

static void set_creation_timeout(uint32_t ms) {
    rmw_context_t *ctx = rcl_context_get_rmw_context(&s_rcl.support.context);
    if (ctx) (void)rmw_uros_set_context_entity_creation_session_timeout(ctx, (int)ms);
}

/* The participant behind the node takes the agent longest to create. */
static bool create_node(void) {
    s_rcl.node = rcl_get_zero_initialized_node();
    set_creation_timeout(ROS_NODE_CREATE_TIMEOUT_MS);
    bool done = ok(rclc_node_init_default(&s_rcl.node, ROS_NODE_NAME, ROS_NODE_NAMESPACE, &s_rcl.support));
    set_creation_timeout(ROS_CREATE_TIMEOUT_MS);
    return done;
}

/* Library default response confirm is 1000 ms inside the executor spin; the
 * contract bound is applied or the creation fails (service finalized). */
static bool create_service(rcl_service_t *srv, const rosidl_service_type_support_t *ts,
                           const ros_entity_desc_t *d) {
    *srv = rcl_get_zero_initialized_service();
    if (!ok(rclc_service_init_default(srv, &s_rcl.node, ts, d->name))) return false;
    rmw_service_t *rmw = rcl_service_get_rmw_handle(srv);
    if (d->reply_timeout_ms && rmw &&
        rmw_uros_set_service_session_timeout(rmw, (int)d->reply_timeout_ms) == RMW_RET_OK) return true;
    (void)ok(rcl_service_fini(srv, &s_rcl.node));
    return false;
}

static bool add_handle(const ros_entity_desc_t *d) {
    rclc_executor_t *e = &s_rcl.executor;
    switch (d->target) {
    case ROS_ENT_TIMER: return ok(rclc_executor_add_timer(e, &s_rcl.timer));
    case ROS_ENT_PARAM_SERVER: return ros_params_rcl_add_to_executor(e);
    case ROS_ENT_SRV_ARM:
        return ok(rclc_executor_add_service(e, &s_rcl.arm, &s_rcl.msgs.arm_request,
                                            &s_rcl.msgs.arm_response, arm_cb));
    default: {
        unsigned i = ros_entity_desc(d->target)->index;
        return ok(rclc_executor_add_subscription(e, &s_rcl.sub[i], &s_rcl.msgs.cmd[i], k_cmd_cb[i], ON_NEW_DATA));
    }
    }
}

static bool create(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d, const ros_session_config_t *cfg) {
    (void)ctx;
    rmw_qos_profile_t q = qos_for(d);
    switch (d->kind) {
    case ROS_KIND_INIT_OPTIONS: return create_init_options(cfg);
    case ROS_KIND_CONTEXT: return create_context();
    case ROS_KIND_CLOCK: return ok(rcl_clock_init(RCL_STEADY_TIME, &s_rcl.support.clock, &s_rcl.allocator));
    case ROS_KIND_NODE: return create_node();
    case ROS_KIND_PUBLISHER: {
        rcl_publisher_t *p = &s_rcl.pub[id - ROS_ENT_PUB_RANGE_HR];
        *p = rcl_get_zero_initialized_publisher();
        return ok(rclc_publisher_init(p, &s_rcl.node, msg_type(d->type), d->name, &q));
    }
    case ROS_KIND_SUBSCRIPTION: {
        rcl_subscription_t *sub = &s_rcl.sub[d->index];
        *sub = rcl_get_zero_initialized_subscription();
        return ok(rclc_subscription_init(sub, &s_rcl.node, msg_type(d->type), d->name, &q));
    }
    case ROS_KIND_SERVICE:
        return create_service(&s_rcl.arm, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), d);
    case ROS_KIND_PARAM_SERVER:
        return ros_params_rcl_create(&s_rcl.node, rcl_context_get_rmw_context(&s_rcl.support.context),
                                     ROS_CREATE_TIMEOUT_MS);
    case ROS_KIND_TIMER:
        s_rcl.timer = rcl_get_zero_initialized_timer();
        return ok(rclc_timer_init_default2(&s_rcl.timer, &s_rcl.support,
                                           RCL_MS_TO_NS(ROS_TIMER_PERIOD_MS), timer_cb, true));
    case ROS_KIND_EXECUTOR:
        s_rcl.executor = rclc_executor_get_zero_initialized_executor();
        return ok(rclc_executor_init(&s_rcl.executor, &s_rcl.support.context, ROS_EXECUTOR_HANDLES,
                                     &s_rcl.allocator));
    case ROS_KIND_EXECUTOR_HANDLE: return add_handle(d);
    }
    return false;
}

/* ---- teardown -------------------------------------------------------------- */

static void fini(rcl_ret_t rc) {
    if (rc != RCL_RET_OK) { s_rcl.fini_errors++; rcl_reset_error(); }
}

static void destroy(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d) {
    (void)ctx;
    switch (d->kind) {
    case ROS_KIND_INIT_OPTIONS: fini(rcl_init_options_fini(&s_rcl.init_options)); break;
    case ROS_KIND_CONTEXT:
        fini(rcl_shutdown(&s_rcl.support.context)); /* deletes the session, closes the transport */
        fini(rcl_context_fini(&s_rcl.support.context));
        break;
    case ROS_KIND_CLOCK: fini(rcl_clock_fini(&s_rcl.support.clock)); break;
    case ROS_KIND_NODE: fini(rcl_node_fini(&s_rcl.node)); break;
    case ROS_KIND_PUBLISHER: fini(rcl_publisher_fini(&s_rcl.pub[id - ROS_ENT_PUB_RANGE_HR], &s_rcl.node)); break;
    case ROS_KIND_SUBSCRIPTION: fini(rcl_subscription_fini(&s_rcl.sub[d->index], &s_rcl.node)); break;
    case ROS_KIND_SERVICE: fini(rcl_service_fini(&s_rcl.arm, &s_rcl.node)); break;
    case ROS_KIND_PARAM_SERVER: if (!ros_params_rcl_destroy(&s_rcl.node)) s_rcl.fini_errors++; break;
    case ROS_KIND_TIMER: fini(rcl_timer_fini(&s_rcl.timer)); break;
    case ROS_KIND_EXECUTOR: fini(rclc_executor_fini(&s_rcl.executor)); break; /* frees handles + wait set */
    case ROS_KIND_EXECUTOR_HANDLE: break; /* no own resource */
    }
}

/* A dead agent must not make each fini wait for a delete confirmation. */
static void begin_teardown(void *ctx) {
    (void)ctx;
    rmw_context_t *rmw = rcl_context_get_rmw_context(&s_rcl.support.context);
    if (rmw) (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
}

static bool release_transport(void *ctx) {
    (void)ctx;
    s_rcl.release.args = &s_rcl.transport;
    return hal_udp_transport_close(&s_rcl.release); /* no-op if rmw already closed it */
}

static void set_generation(void *ctx, uint32_t generation) { (void)ctx; s_rcl.generation = generation; }

static void session_started(void *ctx) {
    (void)ctx;
    ros_telemetry_session_start(&s_rcl.telemetry, hal_now_ms());
}

/* Executor timeout is normal. An error is an rcl failure, or a spin whose
 * publishes all failed (a single dropped datagram is not loss). Console/UI
 * changes are written into the parameter server only after the spin returned,
 * never from inside a parameter callback. */
static ros_spin_result_t spin(void *ctx) {
    (void)ctx;
    uint32_t published = s_rcl.telemetry.published, failed = s_rcl.telemetry.publish_failures;
    rcl_ret_t rc = rclc_executor_spin_some(&s_rcl.executor, RCL_MS_TO_NS(ROS_SPIN_TIMEOUT_MS));
    bool rcl_error = rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT;
    if (rcl_error) rcl_reset_error();
    ros_params_rcl_reconcile();
    bool publish_error = s_rcl.telemetry.publish_failures != failed && s_rcl.telemetry.published == published;
    return rcl_error || publish_error ? ROS_SPIN_ERROR : ROS_SPIN_OK;
}

static bool ping(void *ctx, uint32_t timeout_ms) {
    (void)ctx;
    return rmw_uros_ping_agent((int)timeout_ms, 1) == RMW_RET_OK;
}

static bool sync_time(void *ctx, uint32_t timeout_ms, int64_t *epoch_ns, uint64_t *mono_ms) {
    (void)ctx;
    if (rmw_uros_sync_session((int)timeout_ms) != RMW_RET_OK || !rmw_uros_epoch_synchronized()) return false;
    uint64_t us = hal_now_us64();
    int64_t epoch = rmw_uros_epoch_nanos();
    *mono_ms = us / 1000u;
    *epoch_ns = epoch - (int64_t)(us % 1000u) * 1000; /* epoch at the whole monotonic ms */
    return epoch > 0;
}

static const ros_backend_t k_backend = {create, destroy, begin_teardown, release_transport, set_generation,
                                        session_started, spin, ping, sync_time, NULL};

const ros_backend_t *ros_rcl_backend(const ros_session_t *session) {
    memset(&s_rcl, 0, sizeof s_rcl);
    s_rcl.session = session;
    hal_udp_transport_init(&s_rcl.transport, 0, 0);
    ros_messages_reset(&s_rcl.msgs);
    ros_telemetry_init(&s_rcl.telemetry); /* boot pose */
    ros_params_rcl_boot();
    return &k_backend;
}
