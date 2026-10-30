/* ros_entities.h - the production ROS graph (exact names, QoS, creation order)
 * and its partial-creation tracker. No rcl headers: ros_entities_rcl.c is the
 * rcl/rclc backend, host tests inject a fake backend that fails any step.
 * Single core-0 owner; one entity is created per call so every network-task
 * iteration stays bounded by ROS_CREATE_TIMEOUT_MS. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define ROS_NODE_NAME "pico"
#define ROS_NODE_NAMESPACE "/umbreon"
/* Parameter server (Task 10) handles; the rcl backend asserts this equals the
 * pinned RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES. */
#define ROS_PARAM_SERVER_HANDLES 6u
/* 3 command subscriptions + arm and save services + parameter server + timer. */
#define ROS_EXECUTOR_HANDLES (3u + 2u + ROS_PARAM_SERVER_HANDLES + 1u)
#define ROS_TIMER_PERIOD_MS 10u
#define ROS_CREATE_TIMEOUT_MS 200u  /* per reliable entity creation; < 500 ms core-0 beat limit */
#define ROS_OPEN_PING_TIMEOUT_MS 50u /* fresh XRCE ping just before the session open */
#define ROS_COMMAND_DEPTH 1u
#define ROS_SPIN_TIMEOUT_MS 2u      /* executor opportunity per owner iteration */

typedef enum {
    ROS_KIND_INIT_OPTIONS, /* client key, domain, custom transport registration */
    ROS_KIND_CONTEXT,      /* rcl_init: opens the transport and the XRCE session */
    ROS_KIND_CLOCK,
    ROS_KIND_NODE,
    ROS_KIND_PUBLISHER,
    ROS_KIND_SUBSCRIPTION,
    ROS_KIND_SERVICE,
    ROS_KIND_TIMER,
    ROS_KIND_EXECUTOR,
    ROS_KIND_EXECUTOR_HANDLE /* released by the executor fini, no own resource */
} ros_entity_kind_t;

typedef enum {
    ROS_TYPE_NONE, ROS_TYPE_RANGE, ROS_TYPE_IMU, ROS_TYPE_ODOMETRY, ROS_TYPE_BATTERY,
    ROS_TYPE_DIAG_STATUS, ROS_TYPE_DIAG_ARRAY, ROS_TYPE_TWIST, ROS_TYPE_SET_BOOL
} ros_type_t;

typedef enum { ROS_QOS_NONE, ROS_QOS_BEST_EFFORT, ROS_QOS_RELIABLE } ros_reliability_t;

/* Creation order; teardown runs exactly in reverse. Dependencies always precede. */
typedef enum {
    ROS_ENT_INIT_OPTIONS, ROS_ENT_CONTEXT, ROS_ENT_CLOCK, ROS_ENT_NODE,
    ROS_ENT_PUB_RANGE_HR, ROS_ENT_PUB_RANGE_FR, ROS_ENT_PUB_RANGE_R,
    ROS_ENT_PUB_RANGE_L, ROS_ENT_PUB_RANGE_FL, ROS_ENT_PUB_RANGE_HL,
    ROS_ENT_PUB_IMU, ROS_ENT_PUB_ODOM, ROS_ENT_PUB_BATTERY, ROS_ENT_PUB_REFLEX,
    ROS_ENT_PUB_DIAGNOSTICS,
    ROS_ENT_SUB_CMD_VEL, ROS_ENT_SUB_CMD_TELEOP, ROS_ENT_SUB_CMD_RAW,
    ROS_ENT_SRV_ARM,
    /* Hook: /umbreon/save_settings (std_srvs/Trigger) is added by Task 11 here;
     * its executor handle is already reserved in ROS_EXECUTOR_HANDLES. */
    ROS_ENT_TIMER,
    ROS_ENT_EXECUTOR,
    ROS_ENT_EXEC_CMD_VEL, ROS_ENT_EXEC_CMD_TELEOP, ROS_ENT_EXEC_CMD_RAW,
    ROS_ENT_EXEC_ARM, ROS_ENT_EXEC_TIMER,
    ROS_ENT_COUNT
} ros_entity_id_t;
_Static_assert(ROS_ENT_COUNT <= 32, "creation bitmap is 32 bits");

typedef struct {
    ros_entity_kind_t kind;
    const char *name;            /* absolute topic/service name, NULL otherwise */
    ros_type_t type;
    ros_reliability_t reliability;
    uint8_t depth;               /* keep-last history depth (0 = not a topic) */
    ros_entity_id_t target;      /* executor handle: the entity it dispatches */
    uint8_t index;               /* range sensor index / umb_ros_command_topic_t */
} ros_entity_desc_t;

const ros_entity_desc_t *ros_entity_desc(ros_entity_id_t id); /* NULL if out of range */

typedef struct {
    uint32_t agent_ip;   /* host order */
    uint16_t agent_port;
    uint8_t domain_id;
    uint32_t client_key; /* nonzero, device-unique */
} ros_session_config_t;

typedef enum { ROS_SPIN_OK, ROS_SPIN_ERROR } ros_spin_result_t;

/* Backend contract. create() returns false with nothing of that entity left
 * allocated. destroy() is called once per created entity, in reverse order, and
 * must release memory even when the agent is unreachable. */
typedef struct {
    bool (*create)(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d,
                   const ros_session_config_t *cfg);
    void (*destroy)(void *ctx, ros_entity_id_t id, const ros_entity_desc_t *d);
    void (*begin_teardown)(void *ctx);    /* context exists: zero destroy-session timeout */
    bool (*release_transport)(void *ctx); /* idempotent; false = socket still owned */
    void (*set_generation)(void *ctx, uint32_t generation); /* 0 = setup/none */
    void (*session_started)(void *ctx);   /* entities complete: reset per-session state */
    ros_spin_result_t (*spin)(void *ctx); /* executor timeout is ROS_SPIN_OK */
    bool (*ping)(void *ctx, uint32_t timeout_ms);
    /* epoch_ns is the agent-synchronized epoch at monotonic mono_ms. */
    bool (*sync_time)(void *ctx, uint32_t timeout_ms, int64_t *epoch_ns, uint64_t *mono_ms);
    void *ctx;
} ros_backend_t;

typedef struct {
    uint32_t created;       /* bit per ros_entity_id_t */
    uint8_t next;           /* next id to create */
    bool failed;            /* a create failed: teardown required before retrying */
    bool transport_pending; /* release failed: socket still owned, retry teardown */
} ros_entities_t;

typedef enum { ROS_CREATE_PROGRESS, ROS_CREATE_DONE, ROS_CREATE_FAILED } ros_create_result_t;

void ros_entities_init(ros_entities_t *e);
/* Creates exactly one entity. FAILED leaves the already-created ones tracked
 * for ros_entities_destroy; no further create happens until then. */
ros_create_result_t ros_entities_create_next(ros_entities_t *e, const ros_backend_t *b,
                                             const ros_session_config_t *cfg);
/* begin_teardown first (if a context exists), then every created entity once in
 * reverse order, then the transport. Returns false only if the socket could not
 * be released; the tracker is then busy and the call must be repeated. */
bool ros_entities_destroy(ros_entities_t *e, const ros_backend_t *b);
bool ros_entities_complete(const ros_entities_t *e);
bool ros_entities_busy(const ros_entities_t *e); /* anything created or owned */
