/* ros_params_rcl.c - see ros_params_rcl.h. One static server instance. The
 * modification callback only validates/commits settings through ros_params and
 * never calls an rclc setter (the library disables them inside callbacks); the
 * server is refreshed from the owning task by ros_params_rcl_reconcile(). */
#include "ros_params_rcl.h"
#include <string.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rcl_interfaces/msg/parameter_type.h>
#include <rmw_microros/rmw_microros.h>
#include "hal_time.h"
#include "ros_entities.h"
#include "ros_params.h"
#include "shared_state.h"

_Static_assert(ROS_PARAM_SERVER_HANDLES == RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
               "executor handle budget must use the pinned parameter server count");

static rclc_parameter_server_t s_server;
static ros_params_t s_params;
static bool s_created;

/* ---- modification callback (inside rclc_executor_spin_some) ------------------ */

static ros_param_value_t convert(const Parameter *p) {
    ros_param_value_t v = {.type = (ros_param_type_t)255}; /* anything else never matches a descriptor */
    switch (p->value.type) {
    case rcl_interfaces__msg__ParameterType__PARAMETER_BOOL:
        v.type = ROS_PARAM_BOOL; v.b = p->value.bool_value; break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER:
        v.type = ROS_PARAM_INT; v.i = p->value.integer_value; break;
    case rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE:
        v.type = ROS_PARAM_DOUBLE; v.d = p->value.double_value; break;
    default: break;
    }
    return v;
}

static bool on_modification(const Parameter *old_param, const Parameter *new_param, void *context) {
    const Parameter *named = new_param ? new_param : old_param;
    if (!named || !named->name.data) return false;
    ros_param_value_t v;
    if (new_param) v = convert(new_param);
    return ros_params_on_modification(context, named->name.data, old_param != NULL, new_param ? &v : NULL);
}

/* ---- sink: the rclc server as seen by ros_params_reconcile ------------------- */

static bool rc_ok(rcl_ret_t rc) {
    if (rc == RCL_RET_OK) return true;
    rcl_reset_error();
    return false;
}

static bool sink_get(void *ctx, const ros_param_desc_t *d, ros_param_value_t *out) {
    (void)ctx;
    out->type = d->type;
    switch (d->type) {
    case ROS_PARAM_BOOL: return rc_ok(rclc_parameter_get_bool(&s_server, d->name, &out->b));
    case ROS_PARAM_INT: return rc_ok(rclc_parameter_get_int(&s_server, d->name, &out->i));
    default: return rc_ok(rclc_parameter_get_double(&s_server, d->name, &out->d));
    }
}

static bool sink_put(void *ctx, const ros_param_desc_t *d, const ros_param_value_t *v) {
    (void)ctx;
    switch (d->type) {
    case ROS_PARAM_BOOL: return rc_ok(rclc_parameter_set_bool(&s_server, d->name, v->b));
    case ROS_PARAM_INT: return rc_ok(rclc_parameter_set_int(&s_server, d->name, v->i));
    default: return rc_ok(rclc_parameter_set_double(&s_server, d->name, v->d));
    }
}

static const ros_param_sink_t k_sink = {sink_get, sink_put, NULL};

/* ---- lifecycle --------------------------------------------------------------- */

void ros_params_rcl_boot(void) {
    ros_params_init(&s_params, ros_params_shared_io());
    s_created = false;
}

static rclc_parameter_type_t rclc_type(ros_param_type_t t) {
    return t == ROS_PARAM_BOOL ? RCLC_PARAMETER_BOOL : t == ROS_PARAM_INT ? RCLC_PARAMETER_INT
                                                                          : RCLC_PARAMETER_DOUBLE;
}

/* Library default response confirm is 1000 ms inside the executor spin: every
 * service reply and the (reliable) event publish are bounded or creation fails. */
static bool bound_confirm_timeouts(void) {
    rcl_service_t *services[] = {&s_server.get_service, &s_server.get_types_service, &s_server.set_service,
                                 &s_server.set_atomically_service, &s_server.list_service,
                                 &s_server.describe_service};
    for (size_t i = 0; i < sizeof services / sizeof services[0]; i++) {
        rmw_service_t *rmw = rcl_service_get_rmw_handle(services[i]);
        if (!rmw || rmw_uros_set_service_session_timeout(rmw, (int)ROS_SERVICE_REPLY_TIMEOUT_MS) != RMW_RET_OK)
            return false;
    }
    rmw_publisher_t *event = rcl_publisher_get_rmw_handle(&s_server.event_publisher);
    return event && rmw_uros_set_publisher_session_timeout(event, (int)ROS_SERVICE_REPLY_TIMEOUT_MS) == RMW_RET_OK;
}

/* Declaration and the first values are local operations: events are suppressed
 * (a client sees the complete server on its first request). */
static bool declare_and_populate(void) {
    s_server.notify_changed_over_dds = false;
    bool ok = true;
    for (size_t i = 0; ok && i < ros_params_count(); i++) {
        const ros_param_desc_t *d = ros_params_desc(i);
        ok = rc_ok(rclc_add_parameter(&s_server, d->name, rclc_type(d->type)));
    }
    ros_params_invalidate(&s_params);
    size_t n = ros_params_count();
    ok = ok && ros_params_reconcile(&s_params, &k_sink, (unsigned)n, (unsigned)n);
    s_server.notify_changed_over_dds = true;
    return ok;
}

bool ros_params_rcl_create(rcl_node_t *node, rmw_context_t *rmw_ctx, unsigned restore_timeout_ms) {
    const rclc_parameter_options_t options = {
        .notify_changed_over_dds = true,
        .max_params = ros_params_count(),
        .allow_undeclared_parameters = false,
        .low_mem_mode = true,
    };
    memset(&s_server, 0, sizeof s_server);
    if (rmw_ctx) (void)rmw_uros_set_context_entity_creation_session_timeout(rmw_ctx, (int)ROS_PARAM_CREATE_TIMEOUT_MS);
    bool ok = rc_ok(rclc_parameter_server_init_with_option(&s_server, node, &options));
    if (rmw_ctx) (void)rmw_uros_set_context_entity_creation_session_timeout(rmw_ctx, (int)restore_timeout_ms);
    if (!ok) return false;
    if (bound_confirm_timeouts() && declare_and_populate()) { s_created = true; return true; }
    (void)rc_ok(rclc_parameter_server_fini(&s_server, node));
    return false;
}

bool ros_params_rcl_add_to_executor(rclc_executor_t *executor) {
    return rc_ok(rclc_executor_add_parameter_server_with_context(executor, &s_server, on_modification, &s_params));
}

bool ros_params_rcl_destroy(rcl_node_t *node) {
    if (!s_created) return true;
    s_created = false;
    ros_params_invalidate(&s_params);
    return rc_ok(rclc_parameter_server_fini(&s_server, node));
}

void ros_params_rcl_reconcile(void) {
    if (s_created) (void)ros_params_reconcile(&s_params, &k_sink, ROS_PARAMS_RECONCILE_BUDGET, ROS_PARAMS_WRITE_BUDGET);
}
