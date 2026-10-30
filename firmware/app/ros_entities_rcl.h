/* ros_entities_rcl.h - production rcl/rclc backend for ros_entities/ros_session.
 * Owns every rcl object, the session transport socket, the bounded message
 * storage and the telemetry state (odometry pose survives reconnects).
 * Core-0 network task only; firmware build only (no host test links rcl). */
#pragma once
#include "ros_entities.h"
#include "ros_session.h"
/* Call once at boot. session: read by the timer for time sync/diagnostics. */
const ros_backend_t *ros_rcl_backend(const ros_session_t *session);
