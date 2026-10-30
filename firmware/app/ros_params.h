/* ros_params.h - ROS parameter model backed by the shared settings. Pure
 * logic (no rcl/rclc, RTOS or SDK): the descriptor table, typed validation
 * through the core settings validators, arming/maintenance gates, and the
 * bounded reconciliation of console edits into the parameter server.
 *
 * Core-0 network owner only. The rclc modification callback calls
 * ros_params_on_modification() (which only reads/commits settings, never an
 * rclc setter); ros_params_reconcile() runs from the owner iteration after the
 * executor spin returned and is the only place that writes values back. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "umb_settings.h"

typedef enum { ROS_PARAM_BOOL, ROS_PARAM_INT, ROS_PARAM_DOUBLE } ros_param_type_t;

typedef struct {
    const char *name;   /* ROS parameter name */
    const char *key;    /* canonical umb_settings key (console name) */
    ros_param_type_t type;
    int16_t bias;       /* ROS integer value = settings value - bias (steer_trim only) */
    bool network;       /* changing it reconnects WiFi/agent/session after the reply */
} ros_param_desc_t;

typedef struct {
    ros_param_type_t type;
    bool b;
    int64_t i;
    double d;
} ros_param_value_t;

size_t ros_params_count(void);
const ros_param_desc_t *ros_params_desc(size_t index);
const ros_param_desc_t *ros_params_find(const char *name);
/* Declared range in ROS terms (bool: 0..1; bias applied). */
bool ros_params_range(const ros_param_desc_t *d, double *min, double *max);
bool ros_params_read(const umb_settings_t *s, const ros_param_desc_t *d, ros_param_value_t *out);
/* True when `v` is what the settings already hold (compared in the settings'
 * own type: a double that rounds to the stored float matches). */
bool ros_params_matches(const umb_settings_t *s, const ros_param_desc_t *d, const ros_param_value_t *v);

typedef enum {
    ROS_PARAM_OK,
    ROS_PARAM_UNKNOWN,   /* not a declared parameter */
    ROS_PARAM_BAD_TYPE,
    ROS_PARAM_BAD_VALUE, /* NaN/Inf, out of range or rejected cross-field rule */
    ROS_PARAM_BUSY,      /* armed, arm pending, or maintenance reservation held */
    ROS_PARAM_CONFLICT   /* concurrent settings change kept winning; nothing written */
} ros_param_result_t;

/* Settings access, injectable for tests. Production: ros_params_shared_io(). */
typedef struct {
    uint32_t (*snapshot)(umb_settings_t *out);
    bool (*commit)(const umb_settings_t *s, uint32_t revision);
    bool (*writable)(void);
} ros_params_io_t;
const ros_params_io_t *ros_params_shared_io(void);

#define ROS_PARAMS_COMMIT_ATTEMPTS 3u
#define ROS_PARAMS_RECONCILE_BUDGET 8u /* descriptors examined per owner iteration */
#define ROS_PARAMS_WRITE_BUDGET 3u     /* server writes (each may publish a reliable event) per iteration */

typedef struct {
    bool (*get)(void *ctx, const ros_param_desc_t *d, ros_param_value_t *out);
    bool (*put)(void *ctx, const ros_param_desc_t *d, const ros_param_value_t *v);
    void *ctx;
} ros_param_sink_t;

typedef struct {
    const ros_params_io_t *io;
    umb_settings_t pass; /* frozen copy of the pass in progress */
    uint32_t pass_rev, cached_rev;
    size_t cursor;
    bool pass_active, cache_valid, refreshing;
    uint32_t set_ok, set_rejected, set_conflicts, refresh_writes, refresh_restarts, refresh_failures;
} ros_params_t;

void ros_params_init(ros_params_t *p, const ros_params_io_t *io);
/* Validate and commit one external edit. Nothing changes unless OK. */
ros_param_result_t ros_params_set(ros_params_t *p, const char *name, const ros_param_value_t *v);
/* rclc modification callback logic. Creation (no old value) and deletion (no new
 * value) are refused; an internal refresh (reconcile in progress) is accepted
 * without touching settings, even while armed. */
bool ros_params_on_modification(ros_params_t *p, const char *name, bool has_old, const ros_param_value_t *new_value);
/* Examine at most `budget` descriptors and write at most `writes` differing values
 * (the chunk ends early at the write limit). Restarts
 * from a fresh snapshot if settings changed during the pass. True when the cache
 * is current (revision reconciled); false while work remains or a write failed. */
bool ros_params_reconcile(ros_params_t *p, const ros_param_sink_t *sink, unsigned budget, unsigned writes);
/* Forget the cache (new server instance): the next reconcile writes every value. */
void ros_params_invalidate(ros_params_t *p);
