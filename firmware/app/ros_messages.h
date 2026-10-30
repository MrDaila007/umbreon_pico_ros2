/* ros_messages.h - bounded ROS message storage and telemetry adapters for the
 * core-0 network owner.
 *
 * Ownership: every string/sequence of these messages points into fixed buffers
 * inside ros_messages_t. Only the generated *struct* headers are used: generated
 * __init would heap-allocate strings and generated __fini would free these
 * static buffers, so neither is ever called on this storage. Nothing here
 * allocates; ros_messages_reset() restores the known empty state, and the
 * subscription/service buffers are reset for every new session generation. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.h>
#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.h>
#include <diagnostic_msgs/msg/detail/key_value__struct.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <nav_msgs/msg/detail/odometry__struct.h>
#include <sensor_msgs/msg/detail/battery_state__struct.h>
#include <sensor_msgs/msg/detail/imu__struct.h>
#include <sensor_msgs/msg/detail/range__struct.h>
#include <std_srvs/srv/detail/set_bool__struct.h>
#include "ros_entities.h"
#include "shared_state.h"
#include "umb_odometry.h"
#include "umb_ros_command.h"
#include "umb_telemetry.h"

/* Bounds match the tests/wire serialization fixture (or are smaller). */
#define ROS_DIAG_NAME_MAX 32u
#define ROS_DIAG_MESSAGE_MAX 64u
#define ROS_DIAG_HWID_MAX 32u
#define ROS_DIAG_KEY_MAX 24u
#define ROS_DIAG_VALUE_MAX 32u
#define ROS_DIAG_VALUES 4u
#define ROS_ARM_MESSAGE_MAX 64u
#define ROS_COMMAND_TOPICS 3u
#define ROS_DIAG_HWID "umbreon_pico"

typedef struct {
    char range_frame[UMB_TOF_COUNT][UMB_FRAME_MAX];
    char imu_frame[UMB_FRAME_MAX], odom_frame[UMB_FRAME_MAX], odom_child[UMB_FRAME_MAX];
    char battery_frame[1], battery_location[1], battery_serial[1];
    char reflex_name[ROS_DIAG_NAME_MAX], reflex_message[ROS_DIAG_MESSAGE_MAX], reflex_hwid[ROS_DIAG_HWID_MAX];
    char diag_frame[1];
    char diag_name[ROS_DIAG_NAME_MAX], diag_message[ROS_DIAG_MESSAGE_MAX], diag_hwid[ROS_DIAG_HWID_MAX];
    char diag_key[ROS_DIAG_VALUES][ROS_DIAG_KEY_MAX], diag_value[ROS_DIAG_VALUES][ROS_DIAG_VALUE_MAX];
    char arm_message[ROS_ARM_MESSAGE_MAX];
} ros_message_text_t;

typedef struct {
    sensor_msgs__msg__Range range[UMB_TOF_COUNT];
    sensor_msgs__msg__Imu imu;
    nav_msgs__msg__Odometry odom;
    sensor_msgs__msg__BatteryState battery;
    diagnostic_msgs__msg__DiagnosticStatus reflex;     /* filled by Task 12 */
    diagnostic_msgs__msg__DiagnosticArray diagnostics;
    diagnostic_msgs__msg__DiagnosticStatus diag_status[1];
    diagnostic_msgs__msg__KeyValue diag_values[ROS_DIAG_VALUES];
    geometry_msgs__msg__Twist cmd[ROS_COMMAND_TOPICS]; /* subscription receive buffers */
    std_srvs__srv__SetBool_Request arm_request;
    std_srvs__srv__SetBool_Response arm_response;
    ros_message_text_t text;
} ros_messages_t;

/* Known empty state with all strings bound to their buffers and constant
 * fields (frames, IMU orientation, battery NaNs, diagnostics keys) set. */
void ros_messages_reset(ros_messages_t *m);
/* Zero only the receive/request/response buffers (new session generation). */
void ros_messages_reset_inputs(ros_messages_t *m);
void ros_messages_twist(const geometry_msgs__msg__Twist *in, umb_twist_fields_t *out);
/* Bounded copy into the response's own buffer (message may be truncated). */
void ros_messages_arm_reply(ros_messages_t *m, bool success, const char *message);

/* ---- Telemetry ---------------------------------------------------------- */
typedef struct {
    uint32_t generation, sessions, losses;
    uint32_t sync_age_ms;
} ros_diag_info_t;

typedef struct {
    umb_sched_t sched;
    umb_range_track_t range[UMB_TOF_COUNT];
    umb_gen_t imu;
    umb_odom_t odom; /* boot pose; retained across agent reconnects */
    uint32_t published, publish_failures, unstamped;
} ros_telemetry_t;

/* false = rcl publish error (counted, reported to the session as a failure). */
typedef bool (*ros_publish_fn)(void *ctx, ros_entity_id_t publisher, const void *msg);

void ros_telemetry_init(ros_telemetry_t *t);                           /* firmware boot */
void ros_telemetry_session_start(ros_telemetry_t *t, uint32_t now_ms); /* streams due now */
/* One scheduler tick (10 ms timer). Integrates odometry, then publishes the due
 * streams that have new samples. Nothing is published before time sync: stamps
 * come from the synchronized offset + monotonic acquisition time. Returns the
 * number of failed publishes. now_ms64: full monotonic boot ms. */
unsigned ros_telemetry_tick(ros_telemetry_t *t, ros_messages_t *m, const shared_sensors_t *s,
                            float speed_signed_mps, uint64_t now_ms64,
                            const umb_time_sync_t *sync, const ros_diag_info_t *diag,
                            ros_publish_fn publish, void *ctx);
