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
- Bounded session open: `UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3`,
  `UCLIENT_MIN_SESSION_CONNECTION_INTERVAL=50` (pinned default 10 x 1000 ms), so
  `rcl_init` against a silent agent is configured to give up after about 150 ms
  (not yet measured on hardware). Checked by
  `tools/check_microros.py`.
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

## Session lifecycle and production storage (Task 9)

Implemented in firmware, not yet validated against a live agent (Task 13).

- Graph (`firmware/app/ros_entities.c`): node `pico` in `/umbreon`; the 11
  telemetry publishers, 3 command subscriptions (best-effort, volatile, keep-last
  1) and `/umbreon/arm` (reliable service), one 10 ms scheduler timer and an
  executor sized for 12 handles. `/umbreon/save_settings` (Task 11) is a reserved
  hook: its handle is counted, the entity is not created yet. The parameter
  server (Task 10, below) is created after `/umbreon/arm`. `/umbreon/reflex` exists but is filled by Task 12.
- Creation is one entity per network-task iteration with a 200 ms reliable
  creation timeout (350 ms for the node/participant); the session open is
  preceded by a fresh 50 ms XRCE ping and bounded by the library (3 x 50 ms). No
  session is opened while the car is armed (console bench driving), so a slow
  open can never trip the core-0 stale-beat stop; a live session is kept.
  `/umbreon/arm` responses confirm within 20 ms (library default 1000 ms).
  `agent_up` is published only after every entity and the executor exist; the
  first executor spin happens in the next iteration, after `agent_up`.
  Callbacks carry generation 0 until then, so nothing queued during setup is
  accepted. Client key: FNV-1a of the WiFi MAC; domain from `ros_domain_id`.
- Loss: three failed 100 ms pings (20 ms each) or three consecutive spins with an
  actual rcl/transport error (executor timeout is normal; a spin counts as an
  error only if all its publishes failed) drop the session. The owner publishes
  `agent_up=false` (latching the loss for control), then zeroes the
  destroy-session timeout and finalizes all created entities in reverse order,
  then force-closes the session socket. A socket that cannot be closed keeps the
  session busy, defers radio reconfiguration and blocks re-creation.
  In practice the ping is the main loss detector: best-effort publishes rarely
  report transport errors and the executor does not surface read errors.
  False-loss/false-negative rates are measured in Task 13.
- Time: bounded 20 ms sync, retried every 100 ms until synchronized, then every
  30 s; a failed resync keeps the offset; a new session invalidates it. No
  telemetry is published before the first sync.
- Odometry integrates only in the 10 ms ROS timer: with no session the pose is
  held, and the first interval after reconnect (> 200 ms) re-seeds instead of
  integrating the outage; motion while offline is not tracked.
- Messages (`firmware/app/ros_messages.c`): all strings/sequences point into fixed
  buffers; generated init/fini are never called on them. Receive/request buffers
  are reset for every new session.

Measured with `make wire` on the production storage at its bounds:

| Message | Reserved CDR bytes | Complete XRCE packet bytes |
|---|---:|---:|
| Range (each of 6) | 40 | 52 |
| Imu | 320 | 332 |
| Odometry | 720 | 732 |
| BatteryState | 69 | 84 |
| reflex DiagnosticStatus (empty, Task 12 fills) | 44 | 56 |
| DiagnosticArray (1 status, 4 values) | 220 (219 written) | 232 |
| Twist (received) | 48 | 60 |
| SetBool response (63-char message) | 72 | 84 |

The generated size function can exceed the written bytes by up to 3 bytes of
trailing alignment padding (strings ending off a 4-byte boundary inside a
sequence); rmw reserves the larger value, so the wire test checks that bound.

## Parameter server (Task 10)

Implemented in firmware and host tests; not yet validated against a live agent
or `ros2 param` (Task 13). Node `/umbreon/pico`, rclc **low-memory** mode,
`allow_undeclared_parameters=false`, `max_params = 55` (the counted descriptor
table in `firmware/app/ros_params.c`), event publisher `/parameter_events` on.
The server creates its 6 services (get, get_types, set, set_atomically, list,
describe) and the event publisher in one session-creation step (7 reliable
entities, each bounded to 50 ms, 350 ms worst case) after `/umbreon/arm`; it
takes `RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES` (6, asserted at build time)
executor handles. Service replies and event publishes confirm within 20 ms. The
declare/populate step runs with change events suppressed.

Values are the shared firmware settings (the same ones the console edits); there
is no second copy to save. Edits are RAM only: persist with the console `save`
(the ROS save service is Task 11). Credentials (`wifi_ssid`, `wifi_pass`,
`ap_pass`) and the `calibrated` flag are not parameters; no string parameters.

### Parameter table (55)

Types are ROS `int` (int64), `double` (stored as the float setting), `bool`. The
range is enforced by the same validators as the console `set`, plus the
cross-field rules (`neutral_point` inside `min_point..max_point`, `cap_near_mm` <
`cap_far_mm`, `max_point` >= `min_point`). Reflex values are `0 off, 1 warn,
2 pico, 3 ros`; `arming_mode` is `0 none, 1 encoder, 2 ros, 3 both`;
`wifi_mode` is `0 auto, 1 sta, 2 ap`. "reconnect" parameters change the network
identity: the reply is sent first, then WiFi/agent/session are torn down and
recreated with the new value.

| Name | Type | Range | After set |
|---|---|---|---|
| `arming_mode` | int | 0..3 |  |
| `cmd_timeout_ms` | int | 100..1000 |  |
| `max_speed_ms` | double | 0..5 |  |
| `max_reverse_speed_ms` | double | 0..5 |  |
| `max_throttle` | double | 0..1 |  |
| `max_throttle_rev` | double | 0..1 |  |
| `min_speed` | int | 1500..2000 |  |
| `max_speed` | int | 1500..2000 |  |
| `min_bspeed` | int | 1000..1500 |  |
| `min_point` | int | 0..180 |  |
| `max_point` | int | 0..180 |  |
| `steer_trim` | int | -90..90 |  |
| `servo_reverse` | bool | false / true |  |
| `pid_kp` | double | 0..5000 |  |
| `pid_ki` | double | 0..10000 |  |
| `pid_kd` | double | 0..2000 |  |
| `spd_slew` | double | 0..20 |  |
| `kick_pct` | double | 0..100 |  |
| `kick_ms` | int | 0..2000 |  |
| `corner_kick_us` | int | 0..200 |  |
| `reverse_brake_cmd` | int | -1000..0 |  |
| `reverse_brake_ms` | int | 0..5000 |  |
| `encoder_holes` | int | 1..2000 |  |
| `wheel_diam_m` | double | 0.01..1.0 |  |
| `tach_glitch_filter_us` | int | 1..500 |  |
| `bat_enabled` | bool | false / true |  |
| `bat_multiplier` | double | 0.1..20 |  |
| `bat_low` | double | 0.1..20 |  |
| `imu_rotate` | bool | false / true |  |
| `wifi_mode` | int | 0..2 | reconnect |
| `sta_timeout_s` | int | 5..120 | reconnect |
| `agent_ip` | int | 0..4294967295 | reconnect |
| `agent_port` | int | 1..65535 | reconnect |
| `ros_domain_id` | int | 0..232 | reconnect |
| `reflex.battery.manual` | int | 0..3 |  |
| `reflex.battery.auto` | int | 0..3 |  |
| `reflex.collision.manual` | int | 0..3 |  |
| `reflex.collision.auto` | int | 0..3 |  |
| `reflex.speed_cap.manual` | int | 0..3 |  |
| `reflex.speed_cap.auto` | int | 0..3 |  |
| `reflex.stall.manual` | int | 0..3 |  |
| `reflex.stall.auto` | int | 0..3 |  |
| `reflex.impact.manual` | int | 0..3 |  |
| `reflex.impact.auto` | int | 0..3 |  |
| `reflex.sensor_fault.manual` | int | 0..3 |  |
| `reflex.sensor_fault.auto` | int | 0..3 |  |
| `ttc_ms` | int | 100..3000 |  |
| `min_front_mm` | int | 30..1000 |  |
| `cap_far_mm` | int | 100..2000 |  |
| `cap_near_mm` | int | 50..1500 |  |
| `cap_min_ms` | double | 0..5 |  |
| `collision_sensor_mask` | int | 1..63 |  |
| `stall_min_us` | int | 10..500 |  |
| `stall_ms` | int | 200..5000 |  |
| `impact_g` | double | 1.5..16 |  |

The unit test
`test_table_is_complete_unique_and_consistent_with_core` pins the total (55) and
that every row exists in the core settings table with the same type class and
range.

`steer_trim` is an **alias**: an integer degree offset from 90 that maps to the
canonical `neutral_point` (`neutral_point = 90 + steer_trim`, so `-90..90`).
`neutral_point` itself is not a ROS parameter and the struct/flash layout is
unchanged. A console `set neutral_point 83` shows up as `steer_trim = -7` on the
next reconciliation.

`agent_ip` is an int64 constrained to `0..4294967295`, the host-order value
`a<<24 | b<<16 | c<<8 | d`; `0` means discover. The console keeps dotted IPv4:

```
# console                          # ROS 2
set agent_ip 10.202.20.150         ros2 param set /umbreon/pico agent_ip 181015702
set agent_ip discover              ros2 param set /umbreon/pico agent_ip 0
```

(`10.202.20.150 = 10*2^24 + 202*2^16 + 20*2^8 + 150 = 181015702`.)

### Refusals and arming gates

A set is committed only when it is valid and the car is disarmed with no arm
request queued and no maintenance reservation (save in progress); otherwise the
reply is `successful=false` and nothing changes: wrong type (`ros2 param set
... pid_kp 3` sends an int; use `3.0`), NaN/Inf, out of range or int64 overflow,
cross-field violation, unknown name (no undeclared parameters), deletion
(`ros2 param delete` sends NOT_SET), and any edit while armed, arming, or saving.
This includes `arming_mode`, which on the console is allowed while armed; over
ROS it is not. Changing `arming_mode` clears both arming keys and the neutral
latch through the control task as before. Each commit is one atomic
compare-by-revision write; if a console edit lands in between, the ROS edit is
retried (3 attempts) on the fresh settings, so a newer console value is never
overwritten, and after three conflicts the set is refused.

### Reconciliation

The rclc modification callback only validates and commits settings; it never
calls an rclc setter (the library disables them inside callbacks). After each
executor spin the owner task compares the settings revision with its cache and
writes back differing values in bounded chunks (8 descriptors examined, at most 3
server writes per iteration; each write may publish an event). A revision change
mid-pass restarts the pass from a fresh snapshot, so a stale copy is never
written over a newer value. Console, UI and `defaults` changes therefore appear
in `ros2 param get` within a few owner iterations. These internal writes call the
modification callback too; a flag marks them so they are accepted even while
armed, while an external write is still refused. A new session starts with a
fresh server populated from the current settings.

### Low-memory and CLI behavior

Low-memory mode means one parameter per request; atomic batches are not
promised. The pinned header defines `RCLC_PARAMETER_UNSUPORTED_ON_LOW_MEM`; the
rclc implementation source is not part of this repository, so the exact reply
text could not be read from the pinned build and the following is **unverified
until Task 13 runs it against the real server**:

| Command | Expected |
|---|---|
| `ros2 param set /umbreon/pico <name> <v>` | one `SetParameters` entry; accepted or `successful=false` with the reason |
| `ros2 param get /umbreon/pico <name>` | one name per request: works |
| `ros2 param list /umbreon/pico` | `ListParameters`; low-memory listing limits unverified |
| `ros2 param describe /umbreon/pico <name>` | one name; no ranges are advertised (enforced on set instead) |
| `ros2 param delete /umbreon/pico <name>` | refused (`Rejected by server`) |
| `ros2 param load` / `dump` / multi-name get | multiple parameters per request are a low-memory limitation |
| `set_parameters_atomically` | not promised; treat any result other than an explicit success as unsupported |

Host tests cannot link rclc; they pin our side of the contract: creation and
deletion refused, a batch is judged per parameter and is **not** atomic (an
accepted first entry stays applied when a later one is rejected), and the
descriptor table. The event publishes after a console edit and the
heap use of the server are measured in Task 13.

## Settings persistence (`/umbreon/save_settings`)

`std_srvs/Trigger`, reliable, reply confirm bounded to 20 ms like `/umbreon/arm`.
Console `save`, this service and first-boot AP provisioning share one path in
`settings_apply`: take the exclusive maintenance reservation (refused while armed,
arming, or another save/provision holds it, so the static flash buffer is never
shared), wait at most 60 ms for control's neutral acknowledgement, then
encode/write/read back one slot via `flash_safe_execute` from the core-0 task.
Control never touches flash. A refusal or a missing acknowledgement performs no
erase/program; the reservation is always released, so a retry is possible.

Replies (`success` is true only after the write was read back and verified):

| `success` | `message` |
|---|---|
| true | `saved: slot <0\|1> seq <n>` (non-secret; the new sequence) |
| false | `refused: session` (stale or dead session) |
| false | `refused: armed or busy` |
| false | `error: neutral ack timeout` |
| false | `error: flash write failed` (the old blob stays authoritative; no new sequence is reported) |

Reset or power loss at any point of erase/program leaves the previous valid blob
selected (pure blob tests in `test_settings.c`); the next save targets the other slot.

Timing budget of one callback on the core-0 task (the stale core-0 beat limit is
500 ms): neutral ACK wait <= 60 ms, hal wait one control period + 5 ms = 25 ms,
erase+program measured at 5 ms on hardware (Task 6; datasheet worst case for a 4 KiB
NOR sector erase ~400 ms), readback ~1 ms, reply confirm <= 20 ms. Worst case
without a mid-callback beat would be about 505 ms, which does not fit, so the
callback refreshes the core-0 heartbeat once after the neutral ACK and before the
flash work (the owner task itself making bounded progress; nothing else is fed).
That leaves about 450 ms at the datasheet worst case and about 110 ms measured.
The hardware watchdog is widened to 2 s only around the flash write and restored
immediately; control remains the only routine feeder. Do not lengthen the erase
budget or add feeds outside control: redesign instead.

Flash-induced control pauses are counted apart from jitter: the control task records
each wake's lateness (> 5 ms) as a flash pause when a maintenance reservation is
held, otherwise as jitter; catch-up cycles after one stall are not new events.
Both appear in the console `status` line `control:`.
