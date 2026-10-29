# Umbreon ROS 2 interface (Plan 3 implementation ledger)

Target: ROS 2 Jazzy, node `/umbreon/pico`, UDP custom transport to a matching
micro-ROS Agent. Until Task 13 is complete, this describes the agreed contract;
it does not imply that the ROS graph is already live in firmware.

| Interface | Type | Rate / behavior |
|---|---|---|
| `/umbreon/cmd_vel` | geometry_msgs/Twist | SPEED/AUTO; x m/s, angular.z normalized steering |
| `/umbreon/cmd_teleop` | geometry_msgs/Twist | SPEED/MANUAL; same field meanings |
| `/umbreon/cmd_raw` | geometry_msgs/Twist | RAW/MANUAL; x throttle, angular.z steering |
| `/umbreon/arm` | std_srvs/SetBool | ACK of actual core-1 arming result |
| `/umbreon/save_settings` | std_srvs/Trigger | Success only after verified flash save |
| `/umbreon/range/{hr,fr,r,l,fl,hl}` | sensor_msgs/Range | 25 Hz scheduling, real acquisition stamps |
| `/umbreon/imu/data_raw` | sensor_msgs/Imu | 100 Hz, SI units, native axes |
| `/umbreon/odom` | nav_msgs/Odometry | 50 Hz, signed speed and corrected IMU yaw |
| `/umbreon/battery` | sensor_msgs/BatteryState | 1 Hz |
| `/umbreon/reflex` | diagnostic_msgs/DiagnosticStatus | on change, 2 Hz while active |
| `/umbreon/diagnostics` | diagnostic_msgs/DiagnosticArray | bounded samples, 1 Hz summary cycle |

Three command subscriptions and eleven telemetry publishers use best-effort,
volatile QoS; command history depth 1. Last accepted callback wins. No teleop mux.
Parameters use the standard six services of the pinned Jazzy rclc API, scalar
bool/int/double and low-memory mode. Parameter enumeration is added in Task 10.

## Static-library resource contract

- Custom transport, framing disabled at registration; custom MTU **1200 bytes**.
- One node, capacity16 publishers/4 subscriptions/10 services/0 clients/history4.
- Actual graph uses11 telemetry publishers plus1 parameter-event publisher,
  3 subscriptions, 6 parameter services plus arm and save. One scheduler timer
  requires12 executor handles with the pinned parameter API.
- Generated headers and required type-support symbols are checked by
  `tools/check_microros.py`; the verified build manifest detects an archive/header
  mismatch. Rebuild with `make microros`; the previous library is preserved until
  a verified candidate replaces it.
- Cortex-M33 softfp ABI, matching Pico SDK2.2.0. Neither library nor firmware is
  silently switched to hard-float procedure-call ABI.

## Serialization budget (Task 1)

`tests/wire/run.sh` runs the actual linked ARM serializers and XRCE best-effort
framing on QEMU's Cortex-M33 model, without flashing/controlling the car. It
compares each generated CDR size with serialized bytes and captures the complete
XRCE datagram through a fake send callback. It is not a WiFi performance test.

Bounded fixture: frame/child frame <=31 bytes; DiagnosticStatus name/hardware ID
<=31, summary<=63, at most12 key/value pairs with key<=23/value<=31 bytes; one
status entry per DiagnosticArray. Battery cell sequences empty; location/serial
strings <=31. Production adapters must obey or reduce these bounds. A larger
full diagnostics state is split into complete bounded samples, never cut bytes.

| Message | CDR bytes | Complete XRCE packet bytes |
|---|---:|---:|
| Odometry | 760 | 772 |
| Imu | 344 | 356 |
| Range | 68 | 80 |
| BatteryState | 156 | 168 |
| DiagnosticStatus | 916 | 928 |
| DiagnosticArray | 964 | 976 |

Observed packet overhead is12 bytes for these best-effort WRITE_DATA fixtures;
retain a conservative64-byte reservation in the test. All fit MTU1200. The old
MTU512 library fails the same test at Odometry (760 CDR bytes), as intended.
Reliable service/parameter traffic has separate size/history limits and is not
validated by these telemetry fixtures; check it when those adapters are added.

## Initial link/RAM evidence

Docker build with the verified production library: current sensor/control
firmware text109688 B, data0, bss209024 B; its production networking is not yet
linked. A separate compile-only Phase0 spike link against the new archive gives
text433704 B, data0, bss304684 B, leaving227796 B of the RP2350's532480 B SRAM
outside that static allocation. Both include their statically reserved FreeRTOS
heap; free heap during entity creation is a separate runtime measurement.
These are build/ABI probes, not evidence that the final12-publisher/8-service
runtime fits dynamically. Recheck linker map, heap/stack and allocator use in
Tasks5/9/13. No link-probe image was flashed.
