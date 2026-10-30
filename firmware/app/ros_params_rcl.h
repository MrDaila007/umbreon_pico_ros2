/* ros_params_rcl.h - rclc low-memory parameter server bound to ros_params.
 * Firmware build only (no host test links rclc); the host-testable logic lives
 * in ros_params.c. Core-0 network task only. */
#pragma once
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw/types.h>

/* Once at boot. */
void ros_params_rcl_boot(void);
/* Creates the server (6 services + event publisher, each bounded), declares every
 * descriptor and populates the current settings. On failure nothing is left
 * allocated. `rmw_ctx` carries the creation timeout, restored by the caller. */
bool ros_params_rcl_create(rcl_node_t *node, rmw_context_t *rmw_ctx, unsigned restore_timeout_ms);
/* Registers the six parameter handles (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES). */
bool ros_params_rcl_add_to_executor(rclc_executor_t *executor);
/* False if the finalizer reported an error. Safe if never created. */
bool ros_params_rcl_destroy(rcl_node_t *node);
/* Owner iteration, after the executor spin returned: write console/UI changes
 * into the server in bounded chunks. Never called from a callback. */
void ros_params_rcl_reconcile(void);
