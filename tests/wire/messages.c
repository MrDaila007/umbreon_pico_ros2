/* Actual generated ARM serializers, executed on emulated Cortex-M33, no car I/O. */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <uxr/client/client.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/detail/odometry__rosidl_typesupport_microxrcedds_c.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/detail/imu__rosidl_typesupport_microxrcedds_c.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/detail/range__rosidl_typesupport_microxrcedds_c.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/detail/battery_state__rosidl_typesupport_microxrcedds_c.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/detail/diagnostic_array__rosidl_typesupport_microxrcedds_c.h>
#include <diagnostic_msgs/msg/detail/diagnostic_status__rosidl_typesupport_microxrcedds_c.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
#include <uxr/client/config.h>
#include <geometry_msgs/msg/detail/twist__rosidl_typesupport_microxrcedds_c.h>
#include <std_srvs/srv/detail/set_bool__rosidl_typesupport_microxrcedds_c.h>
#include <std_srvs/srv/detail/trigger__rosidl_typesupport_microxrcedds_c.h>
#include "ros_messages.h"

#define TS(pkg, name) ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, pkg, msg, name)()
#define MAX_PAIRS 12
#define OVERHEAD_BUDGET 64u
static uint8_t bytes[UXR_CONFIG_CUSTOM_TRANSPORT_MTU];
static size_t packet_size;
static bool capture(void *ctx, const uint8_t *buf, size_t len) {
    (void)ctx; (void)buf; packet_size = len; return true;
}
/* The session flush may inspect monotonic time; no real networking is involved. */
int clock_gettime(clockid_t id, struct timespec *tp) {
    (void)id; tp->tv_sec = 1; tp->tv_nsec = 0; return 0;
}
static char text31[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
static char text23[] = "xxxxxxxxxxxxxxxxxxxxxxx";
static char text63[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

static int semihost(int op, const void *args) {
    register int r0 __asm__("r0") = op;
    register const void *r1 __asm__("r1") = args;
    __asm__ volatile("bkpt 0xab" : "+r"(r0) : "r"(r1) : "memory");
    return r0;
}
void wire_exit(int code) {
    uint32_t args[2] = {0x20026u, (uint32_t)code};
    (void)semihost(0x20, args);
    for (;;) {}
}
static void put(const char *s) { (void)semihost(4, s); }
static void number(size_t n) {
    char b[24]; unsigned i = sizeof b - 1; b[i] = 0;
    do { b[--i] = (char)('0' + n % 10); n /= 10; } while (n);
    put(b + i);
}
static rosidl_runtime_c__String str(char *s) {
    return (rosidl_runtime_c__String){s, strlen(s), strlen(s) + 1};
}
static void check(const char *name, const rosidl_message_type_support_t *ts, const void *msg) {
    const message_type_support_callbacks_t *cb = ts->data;
    size_t size = cb->get_serialized_size(msg);
    ucdrBuffer b;
    ucdr_init_buffer(&b, bytes, sizeof bytes);
    /* The generated size function may over-estimate by trailing alignment
     * padding (strings ending off a 4-byte boundary inside a sequence); rmw
     * reserves that size, so it is the bound checked, never less than written. */
    if (!cb->cdr_serialize(msg, &b) || b.error || ucdr_buffer_length(&b) > size ||
        size - ucdr_buffer_length(&b) > 3u || size + OVERHEAD_BUDGET > sizeof bytes) {
        put("FAIL "); put(name); put(" size="); number(size); put("\n"); wire_exit(1);
    }
    size_t written = ucdr_buffer_length(&b);
    static uxrSession session;
    uxrCommunication comm = {.send_msg = capture, .mtu = sizeof bytes};
    uxr_init_session(&session, &comm, 0x12345678u);
    uxrStreamId stream = uxr_create_output_best_effort_stream(&session, bytes, sizeof bytes);
    uint16_t request = uxr_prepare_output_stream(&session, stream,
            uxr_object_id(1, UXR_DATAWRITER_ID), &b, (uint32_t)size);
    if (request == UXR_INVALID_REQUEST_ID || !cb->cdr_serialize(msg, &b) || b.error) {
        put("FAIL XRCE framing\n"); wire_exit(1);
    }
    packet_size = 0;
    uxr_flash_output_streams(&session);
    if (packet_size < size || packet_size > sizeof bytes || packet_size - size > OVERHEAD_BUDGET) {
        put("FAIL XRCE packet size\n"); wire_exit(1);
    }
    put(name); put(" CDR="); number(size);
    if (written != size) { put(" (serialized="); number(written); put(")"); }
    put(" actual-XRCE-packet="); number(packet_size);
    put(" overhead="); number(packet_size - size); put("\n");
}
/* The production storage itself, filled at its bounds by the real adapters. */
static ros_messages_t prod;
static ros_telemetry_t tele;
static bool keep(void *ctx, ros_entity_id_t id, const void *msg) { (void)ctx; (void)id; (void)msg; return true; }
static void check_production(void) {
    ros_messages_reset(&prod); ros_telemetry_init(&tele);
    umb_time_sync_t sync = umb_time_sync_init();
    (void)umb_time_sync_set(&sync, 1700000000000000000ll, 1000);
    shared_sensors_t s = {0};
    for (int i = 0; i < UMB_TOF_COUNT; i++) { s.tof_mm[i] = 1234; s.tof_stamp_ms[i] = 1000; }
    s.tof_online_mask = 0x3f; s.imu_ok = true; s.imu_stamp_ms = 1000; s.bat_have = true; s.bat_v = 7.4f;
    ros_diag_info_t diag = {4294967295u, 4294967295u, 4294967295u, 4294967295u}; /* widest values */
    ros_telemetry_session_start(&tele, 1000);
    (void)ros_telemetry_tick(&tele, &prod, &s, 1.0f, 1000, &sync, &diag, keep, NULL);
    char longest[ROS_ARM_MESSAGE_MAX + 8];
    memset(longest, 'x', sizeof longest - 1); longest[sizeof longest - 1] = 0;
    ros_messages_arm_reply(&prod, true, longest);
    for (int i = 0; i < UMB_TOF_COUNT; i++) check("prod Range", TS(sensor_msgs, Range), &prod.range[i]);
    check("prod Imu", TS(sensor_msgs, Imu), &prod.imu);
    check("prod Odometry", TS(nav_msgs, Odometry), &prod.odom);
    check("prod BatteryState", TS(sensor_msgs, BatteryState), &prod.battery);
    check("prod reflex DiagnosticStatus", TS(diagnostic_msgs, DiagnosticStatus), &prod.reflex);
    check("prod DiagnosticArray", TS(diagnostic_msgs, DiagnosticArray), &prod.diagnostics);
    check("prod Twist", TS(geometry_msgs, Twist), &prod.cmd[0]);
    check("prod SetBool_Response",
          ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, std_srvs, srv, SetBool_Response)(),
          &prod.arm_response);
    ros_messages_save_reply(&prod, true, longest);
    check("prod Trigger_Response",
          ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, std_srvs, srv, Trigger_Response)(),
          &prod.save_response);
}
int main(void) {
    check_production();
    nav_msgs__msg__Odometry odom = {0};
    odom.header.frame_id = str(text31); odom.child_frame_id = str(text31);
    check("Odometry", TS(nav_msgs, Odometry), &odom);
    sensor_msgs__msg__Imu imu = {0}; imu.header.frame_id = str(text31);
    check("Imu", TS(sensor_msgs, Imu), &imu);
    sensor_msgs__msg__Range range = {0}; range.header.frame_id = str(text31);
    check("Range", TS(sensor_msgs, Range), &range);
    sensor_msgs__msg__BatteryState battery = {0}; battery.header.frame_id = str(text31);
    battery.location = str(text31); battery.serial_number = str(text31);
    check("BatteryState", TS(sensor_msgs, BatteryState), &battery);
    diagnostic_msgs__msg__KeyValue kv[MAX_PAIRS] = {0};
    for (int i = 0; i < MAX_PAIRS; i++) { kv[i].key = str(text23); kv[i].value = str(text31); }
    diagnostic_msgs__msg__DiagnosticStatus status = {0};
    status.name = str(text31); status.message = str(text63); status.hardware_id = str(text31);
    status.values = (diagnostic_msgs__msg__KeyValue__Sequence){kv, MAX_PAIRS, MAX_PAIRS};
    check("DiagnosticStatus", TS(diagnostic_msgs, DiagnosticStatus), &status);
    diagnostic_msgs__msg__DiagnosticArray diag = {0}; diag.header.frame_id = str(text31);
    diag.status = (diagnostic_msgs__msg__DiagnosticStatus__Sequence){&status, 1, 1};
    check("DiagnosticArray", TS(diagnostic_msgs, DiagnosticArray), &diag);
    put("ARM serialized size checks passed\n"); wire_exit(0);
}
