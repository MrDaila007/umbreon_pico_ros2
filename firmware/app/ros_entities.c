/* ros_entities.c - see ros_entities.h. Pure table + bitmap; the backend owns
 * every rcl object and socket. */
#include "ros_entities.h"
#include <stddef.h>

#define PUB(n, t, i) {ROS_KIND_PUBLISHER, n, t, ROS_QOS_BEST_EFFORT, 1u, ROS_ENT_COUNT, i, 0u}
#define SUB(n, i) {ROS_KIND_SUBSCRIPTION, n, ROS_TYPE_TWIST, ROS_QOS_BEST_EFFORT, ROS_COMMAND_DEPTH, ROS_ENT_COUNT, i, 0u}
#define LOCAL(k) {k, NULL, ROS_TYPE_NONE, ROS_QOS_NONE, 0u, ROS_ENT_COUNT, 0u, 0u}
#define HANDLE(target) {ROS_KIND_EXECUTOR_HANDLE, NULL, ROS_TYPE_NONE, ROS_QOS_NONE, 0u, target, 0u, 0u}

/* Topic/service names are absolute so the /umbreon node namespace is not doubled. */
static const ros_entity_desc_t s_graph[ROS_ENT_COUNT] = {
    [ROS_ENT_INIT_OPTIONS] = LOCAL(ROS_KIND_INIT_OPTIONS),
    [ROS_ENT_CONTEXT] = LOCAL(ROS_KIND_CONTEXT),
    [ROS_ENT_CLOCK] = LOCAL(ROS_KIND_CLOCK),
    [ROS_ENT_NODE] = LOCAL(ROS_KIND_NODE),
    [ROS_ENT_PUB_RANGE_HR] = PUB("/umbreon/range/hr", ROS_TYPE_RANGE, 0u),
    [ROS_ENT_PUB_RANGE_FR] = PUB("/umbreon/range/fr", ROS_TYPE_RANGE, 1u),
    [ROS_ENT_PUB_RANGE_R] = PUB("/umbreon/range/r", ROS_TYPE_RANGE, 2u),
    [ROS_ENT_PUB_RANGE_L] = PUB("/umbreon/range/l", ROS_TYPE_RANGE, 3u),
    [ROS_ENT_PUB_RANGE_FL] = PUB("/umbreon/range/fl", ROS_TYPE_RANGE, 4u),
    [ROS_ENT_PUB_RANGE_HL] = PUB("/umbreon/range/hl", ROS_TYPE_RANGE, 5u),
    [ROS_ENT_PUB_IMU] = PUB("/umbreon/imu/data_raw", ROS_TYPE_IMU, 0u),
    [ROS_ENT_PUB_ODOM] = PUB("/umbreon/odom", ROS_TYPE_ODOMETRY, 0u),
    [ROS_ENT_PUB_BATTERY] = PUB("/umbreon/battery", ROS_TYPE_BATTERY, 0u),
    [ROS_ENT_PUB_REFLEX] = PUB("/umbreon/reflex", ROS_TYPE_DIAG_STATUS, 0u),
    [ROS_ENT_PUB_DIAGNOSTICS] = PUB("/umbreon/diagnostics", ROS_TYPE_DIAG_ARRAY, 0u),
    [ROS_ENT_SUB_CMD_VEL] = SUB("/umbreon/cmd_vel", 0u),       /* UMB_ROS_CMD_VEL */
    [ROS_ENT_SUB_CMD_TELEOP] = SUB("/umbreon/cmd_teleop", 1u), /* UMB_ROS_CMD_TELEOP */
    [ROS_ENT_SUB_CMD_RAW] = SUB("/umbreon/cmd_raw", 2u),       /* UMB_ROS_CMD_RAW */
    [ROS_ENT_SRV_ARM] = {ROS_KIND_SERVICE, "/umbreon/arm", ROS_TYPE_SET_BOOL, ROS_QOS_RELIABLE,
                         0u, ROS_ENT_COUNT, 0u, ROS_SERVICE_REPLY_TIMEOUT_MS},
    [ROS_ENT_PARAM_SERVER] = {ROS_KIND_PARAM_SERVER, "/umbreon/pico", ROS_TYPE_NONE, ROS_QOS_RELIABLE,
                              0u, ROS_ENT_COUNT, 0u, ROS_SERVICE_REPLY_TIMEOUT_MS},
    [ROS_ENT_TIMER] = LOCAL(ROS_KIND_TIMER),
    [ROS_ENT_EXECUTOR] = LOCAL(ROS_KIND_EXECUTOR),
    [ROS_ENT_EXEC_CMD_VEL] = HANDLE(ROS_ENT_SUB_CMD_VEL),
    [ROS_ENT_EXEC_CMD_TELEOP] = HANDLE(ROS_ENT_SUB_CMD_TELEOP),
    [ROS_ENT_EXEC_CMD_RAW] = HANDLE(ROS_ENT_SUB_CMD_RAW),
    [ROS_ENT_EXEC_ARM] = HANDLE(ROS_ENT_SRV_ARM),
    [ROS_ENT_EXEC_PARAMS] = HANDLE(ROS_ENT_PARAM_SERVER),
    [ROS_ENT_EXEC_TIMER] = HANDLE(ROS_ENT_TIMER),
};

const ros_entity_desc_t *ros_entity_desc(ros_entity_id_t id) {
    return (unsigned)id < ROS_ENT_COUNT ? &s_graph[id] : NULL;
}

void ros_entities_init(ros_entities_t *e) {
    *e = (ros_entities_t){0};
}

ros_create_result_t ros_entities_create_next(ros_entities_t *e, const ros_backend_t *b,
                                             const ros_session_config_t *cfg) {
    if (e->failed || e->transport_pending) return ROS_CREATE_FAILED;
    if (e->next >= ROS_ENT_COUNT) return ROS_CREATE_DONE;
    ros_entity_id_t id = (ros_entity_id_t)e->next;
    if (!b->create(b->ctx, id, &s_graph[id], cfg)) { e->failed = true; return ROS_CREATE_FAILED; }
    e->created |= 1u << id;
    e->next++;
    return e->next >= ROS_ENT_COUNT ? ROS_CREATE_DONE : ROS_CREATE_PROGRESS;
}

bool ros_entities_destroy(ros_entities_t *e, const ros_backend_t *b) {
    /* Zero the destroy-session timeout before any fini can wait on a dead agent. */
    if (e->created & (1u << ROS_ENT_CONTEXT)) b->begin_teardown(b->ctx);
    for (int id = ROS_ENT_COUNT - 1; id >= 0; id--) {
        if (!(e->created & (1u << id))) continue;
        e->created &= ~(1u << id); /* cleared first: never finalized twice */
        b->destroy(b->ctx, (ros_entity_id_t)id, &s_graph[id]);
    }
    e->next = 0; e->failed = false;
    e->transport_pending = !b->release_transport(b->ctx);
    return !e->transport_pending;
}

bool ros_entities_complete(const ros_entities_t *e) {
    return !e->failed && e->next >= ROS_ENT_COUNT;
}

bool ros_entities_busy(const ros_entities_t *e) {
    return e->created || e->failed || e->transport_pending;
}
