# Umbreon Pico 2 W — Plan 3: WiFi and micro-ROS

Date: 2026-10-29
Status: implementation in progress. Task1 complete; Task2 coordination implemented,
with real flash/watchdog timing acceptance still open. ROS networking is not live yet.

**Goal:** the existing Pico 2 W firmware joins WiFi or starts its own AP, reconnects
to a Jazzy micro-ROS Agent, publishes the agreed telemetry and accepts ROS
commands/services/parameters through the existing control loop. Network work must
never become responsible for PWM timing or stop-on-silence enforcement.

**Baseline:** branch `plan3-network`, cut from `plan2-drivers` at `ef051fd`.
Plan 2 implementation: `ae23834`; native ELF/UF2, 280 tests, 99.38% core/driver
line coverage. All six ToFs, IMU, battery and encoder reported healthy during the
recorded disarmed smoke. Physical accuracy, audio and motion checks remain
owner-run items; do not silently count them as passed.

**Spec:** `docs/specs/2026-10-29-pico2w-ros2-firmware-design.md`, especially
§4–6 and §8–12. Phase 0 evidence: `docs/spike-results.md`. Preserve its measured
loss/latency limitations; the office network is not a proven low-latency link.

**Execution:** work task by task on this branch. Before each implementation,
write meaningful failing host tests where logic is testable; implement, run the
relevant checks, review the actual diff, then commit. Maintain a task state in
`../.remember/plan3-network-plan.md` after each stage/check/decision.

## Scope and delivery boundaries

Included here:

- Station mode, WPA2 AP fallback with DHCP, stable per-device AP credentials,
  bounded endpoint selection and recovery, `auto` / `sta` / `ap` preferences.
- One production micro-ROS node, all §5 telemetry and command topics, arming and
  save services, numeric/bool parameters, session time synchronization.
- Independent command timeout, real communication-task heartbeat, explicit
  network-loss disarm and reconnect invalidation, including partial arming keys.
- Minimal Jazzy agent and CLI containers needed to verify this firmware.
- Host tests, fake-network integration, safe disarmed smoke and owner checklists.

Plan 4 owns OLED screens and QR rendering. Plan 3 must expose an in-RAM network
snapshot/identity API for that UI. Plan 5 owns the desktop ROS workspace, URDF/TF,
gamepad, ROS safety node, visualization and bag launch files. Plan 6 owns full
moving-car E2E acceptance. Do not build a teleop/autonomy mux in this plan.

## Global constraints

- `firmware/core/` remains pure C11, value semantics, no SDK, RTOS or ROS headers;
  `-Wall -Wextra -Werror`; >=80% core+driver line coverage. Retain existing tests.
- Core 1 control remains the sole servo/ESC PWM writer and the sole routine
  watchdog feeder. Core 0 owns all rcl/rmw/uxr objects, transport and WiFi FSMs.
  No network API, allocation, semaphore wait or flash write in the control path.
- Network startup, session init, reliable services and teardown can fail. Each
  failure path must leave neutral output possible and release partial resources.
- Short shared-state spinlocks only protect copies/queue operations. No SDK,
  RTOS, flash, socket, rcl or printf call while holding a shared spinlock.
- Default command timeout stays 250 ms, configurable 100–1000 ms. Measure the
  control stop at timeout + at most one 20 ms cycle. Do not tune it upward to
  conceal WiFi loss. Local safety clocks stay monotonic, independent of ROS time.
- Preserve console driving as a deliberate offline bench path. A ROS command
  always requires WiFi + a ready agent session; no implicit ROS-to-console fallover.
- No credentials in git, ROS parameters, telemetry, captured logs or test fixtures.
  Use synthetic test passwords. Existing `get` masking remains intact.
- Keep `umb_settings_t`/flash version 1 unchanged unless a demonstrated requirement
  needs a migration. All planned network fields already exist in the struct.
- Hardware scope for the agent: flash/boot and serial `status`, `sensors`, `help`,
  `cal imu` only after confirming disarmed/stationary. Read-only ROS inspection is
  permitted. Never arm, publish drive commands, set `mode none`, edit live settings
  or call save on the board. Moving, AP-switching and persistence scenarios below
  are owner-run. Do not modify host routes, disconnect the owner's AP, or kill
  unrelated containers. Starting/stopping this project's test agent is in scope.
- On code changes run `graphify update .` if installed; keep generated untracked
  graph output separate from commits. A code-only update does not claim docs were
  semantically re-indexed.

## Decisions from the baseline audit

1. **Heartbeat owner changes.** Today `console_task` calls `shared_core0_beat`.
   In Plan 3 the communication task becomes its only writer, at least every 100 ms
   during functioning bounded iterations, even offline. A responsive console must
   not hide a wedged executor. Keep the core-1 500 ms stale-beat protection.
2. **Loss is an event as well as a level.** Existing WiFi/agent stop bits depend
   on `safety.source == UMB_SRC_ROS`. Keep that level gate for offline console use,
   but add a latched network-loss/session-generation event consumed by control,
   independent of last command source. It clears both arming keys, the neutral
   latch, queued arm requests and old commands. A quick down/up before the next
   control tick must still be observed. Initial offline boot is not repeated loss.
3. **Do not inherit the spike's blocking lifecycle.** Use asynchronous STA join,
   bounded session steps and reverse-order partial cleanup. Drop ready/link flags
   and notify control before teardown. Ping is not the 250 ms command watchdog.
4. **MTU and limits need a library rebuild.** The actual generated custom MTU is
   512. Odometry includes two 36-double covariance arrays (576 bytes before its
   other fields), so it cannot fit a best-effort packet at that MTU. Start with
   custom MTU 1200 and test actual serialized sizes, including XRCE overhead.
5. **Use the pinned Jazzy headers.** They define six parameter services/handles,
   not the five mentioned by some older tutorials. They support bool/int/double,
   not string parameters. Credentials stay in console/flash; enum ROS values are
   integers. Do not assume a declared discovery symbol is linked: the present
   custom-transport library does not contain `rmw_uros_discover_agent`.
6. **AP fallback is in scope.** STA uses configured `agent_ip` when nonzero. With
   no IP, try bounded subnet discovery/probing. AP mode selects candidates from
   DHCP leases and validates each by XRCE ping. No unbounded discovery loop or
   automatic assumption that the first phone is an agent. UI QR is Plan 4.
7. **Save needs a reservation.** A check of `!outputs.armed` followed by flash is
   racy with encoder/service arming. Use a shared maintenance reservation plus a
   control neutral acknowledgement; gate all arm sources while it is held.
8. **Report what was measured.** `status` speed is measured, not target. An existing
   `calibrated yes` flag does not prove a new calibration succeeded. Add explicit
   network timing/health diagnostics rather than inferring them from a console
   that happens to respond.

## Runtime and interface contracts

### Scheduling and ownership

Core 1 tasks retain their existing priorities and pinning. New application task
`net_task` is pinned to core 0 at idle+3; console stays idle+2. CYW43 initialization
runs inside `net_task`. Audit generated CYW43 async-context and lwIP `tcpip_thread`
affinity too: they must stay on core 0, rather than relying on the app task alone.
Do not pin the shared FreeRTOS timer service blindly: inspect SDK/RTOS users first.

Normal network iterations target <=10 ms between executor opportunities. One
20 ms ping can extend an iteration; command safety still runs on core 1. Initial
proposed live ping policy: every 100 ms, timeout 20 ms, one attempt, drop session
after three consecutive failures; expected declaration of loss <=400 ms including
loop overhead. Record false-loss frequency on the real link before changing policy.
Entity creation/session-open operations have explicit finite timeouts; during them
`agent_up` remains false. A task that actually stalls must not feed its own beat
from another helper task. No burst publish backlog after reconnect.

Time sync: bounded 20 ms attempts, retry until synchronized, then every 30 s.
Do not publish falsely stamped sensor data before first successful sync. Command
safety and disarm remain available if time sync fails. Expose sync state/age;
keep a previous valid offset during a transient resync failure, invalidate it on
new session/agent generation and report degraded sync.

### Shared state additions (names may be refined in the implementing task)

- `shared_network_t`: desired/active WiFi mode and FSM stage, connected IP/mask,
  AP SSID (no password), endpoint IP/port and selection method, RSSI-valid/RSSI,
  session generation, ready flags, last successful ping, reconnect/error counters,
  sync status/age, network task heartbeat/loop max, heap watermarks.
- `shared_notify_network_loss(reason, generation)`: latch until control consumes;
  repeated events cannot disappear if a producer overwrites a status snapshot.
- Arm requests become bounded request records with origin, request ID and session
  generation; post returns accepted/busy. Disarm has a separate sticky priority
  mailbox and cannot be dropped behind a full arm queue. Completion ACK comes
  from the control task, not from the enqueue operation.
- Settings mutation is a short atomic shared transaction against the latest copy:
  validation + disarmed/no-pending-arm gate + settings revision. Multiple console,
  ROS and future UI producers cannot silently overwrite one another's edits.
- Maintenance reservation inhibits every arm source and all motion command
  acceptance, asks control for neutral, and receives its acknowledgement before
  any flash operation. Release requires a fresh neutral command before motion.
- Reflex transitions use a bounded control-producer/network-consumer event FIFO,
  with sample timestamps and relevant values. Overflow is counted; resynchronize
  the latest full state rather than silently losing the clearing transition.
- Extend sensor/status telemetry with IMU cumulative read errors, I2C failure
  counters, ToF errors, and control loop period/max/overruns. Snapshot readers must
  retain the Plan 2 per-slot expiration behavior and copy no unbounded strings.

### ROS graph and resource budget

Node name `pico`, namespace `/umbreon` (full node `/umbreon/pico`). Topic/service
names below are absolute so the node namespace cannot duplicate `/umbreon`.

| Kind | Interface | Semantics / QoS / target cadence |
|---|---|---|
| Sub | `/umbreon/cmd_vel`, Twist | SPEED, AUTO; linear.x m/s, angular.z normalized steer; best-effort, keep-last 1 |
| Sub | `/umbreon/cmd_teleop`, Twist | SPEED, MANUAL; same fields/QoS |
| Sub | `/umbreon/cmd_raw`, Twist | RAW, MANUAL; linear.x throttle; same QoS |
| Service | `/umbreon/arm`, SetBool | Actual control ACK; false always disarms and clears both keys |
| Service | `/umbreon/save_settings`, Trigger | True only after verified flash commit; refused armed/busy |
| Pub x6 | `/umbreon/range/{hr,fr,r,l,fl,hl}`, Range | best-effort, volatile, 25 Hz scheduling; new sample stamps, no fabricated updates |
| Pub | `/umbreon/imu/data_raw`, Imu | best-effort, 100 Hz, native sensor axes, SI units, no orientation |
| Pub | `/umbreon/odom`, Odometry | best-effort, 50 Hz, frame odom / child base_link, no firmware TF publisher |
| Pub | `/umbreon/battery`, BatteryState | best-effort, 1 Hz, voltage only; unsupported measurements unknown |
| Pub | `/umbreon/reflex`, DiagnosticStatus | best-effort; event per transition, 2 Hz repeat while active |
| Pub | `/umbreon/diagnostics`, DiagnosticArray | best-effort, 1 Hz; bounded summary and diagnostics |
| Parameters | `/umbreon/pico/*` standard parameter services | rclc low-memory mode; one parameter per get/set request |

The parameter service names are node-scoped, e.g.
`/umbreon/pico/get_parameters`; `ros2 param ... /umbreon/pico` addresses them.
Counts: 11 telemetry publishers + optional `/parameter_events` publisher = 12;
3 subscriptions; 6 parameter services + arm + save = 8 services. One 10 ms
scheduler timer gives `3 + 2 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1 = 12`
executor handles with the pinned header. Proposed library capacity: 1 node,
16 publishers, 4 subscriptions, 10 services, 0 clients, history 4, custom MTU1200.
Measure SRAM before accepting this headroom; do not blindly increase heap size.

Use preallocated bounded strings, diagnostic sequences and service responses.
No per-message heap allocation after entity setup. For best-effort messages,
assert measured serialized size + actual XRCE overhead fits the stream packet;
reserve at least 64 bytes until the exact middleware overhead is checked. Diagnostics
must be split/bounded if they exceed the budget, not truncated inside serialized
bytes. A capped list of full, individually valid diagnostic entries is acceptable
if omitted-count/overflow is explicit. Use reliable timeouts <=20 ms for parameter
events/service responses during normal operation; reset error state deliberately.

### Conversion rules

- All three Twist callbacks share pure validation. Reject non-finite consumed
  fields, unsupported nonzero axes and invalid topic IDs; rejected input does not
  refresh `rx_ms` or the neutral latch. Use local receive time, never PC ROS time.
  Clamp finite steering to [-1,1]; apply signed speed/throttle limits through
  existing core logic. Across topics, last callback accepted by the single
  executor wins. Console/ROS latest-value behavior remains explicit and tested.
- Range: mm / 1000; 9999/offline/stale -> positive infinity. Radiation type IR,
  frame `tof_<label>`, nominal FoV 25 degrees in radians. The driver's 8190 mm
  invalid cutoff is not a physical max_range. Use an explicit documented profile
  limit no higher than the sensor's nominal 2 m; high-speed practical range can
  be shorter. Pin the minimum usable distance/profile limit against manufacturer
  evidence during Task8 and record any measurement-dependent limitation. Valid
  samples above advertised max -> +inf, valid samples below min -> -inf (too
  close); do not label such samples as normal finite readings. Core safety retains
  its original raw distances. Preserve real per-sensor timestamps.
- IMU: raw gyro degrees/s -> rad/s, raw acceleration g -> m/s² using 9.80665.
  Frame `imu_link`, identity quaternion with orientation_covariance[0] = -1.
  Raw means the sensor's own axes/bias, consistent with the existing raw snapshot.
  Do not negate raw axes twice using `imu_rotate`; odometry uses the existing
  configured, bias-corrected yaw rate. Missing/stale IMU is diagnosed and not
  published as a fresh healthy sample.
- Odometry: integrate signed measured speed and continuous yaw from the corrected
  yaw rate, not normalized steering. Use monotonic sample intervals, ignore gaps
  >=500 ms and invalid IMU/speed; never fabricate a yaw of zero as a good estimate.
  With IMU invalid, freeze pose integration and advertise high/unknown pose
  uncertainty; signed linear velocity may still be reported if finite. Reset
  pose at firmware boot, retain through an agent reconnect, do not integrate a
  network outage as one giant interval. Use normalized yaw quaternion and
  explicit conservative covariance, with unobservable z/roll/pitch marked large.
- Battery: `present = bat_have`; voltage as measured; unknown current, charge,
  capacity and percentage are NaN, unsupported status/health/technology UNKNOWN,
  empty bounded cell sequences. Do not invent state of charge from voltage.
- Header stamps derive from synchronized epoch offset + monotonic acquisition
  time, not publication time. Reconstruct uint32 sensor-ms ages against a uint64
  monotonic clock so 49-day wrap does not corrupt epoch stamps. ROS time jumps
  cannot affect command expiry, health ages or odometry dt.

### Parameters and persistence

Define one explicit descriptor table mapping ROS scalar names to existing settings
keys/types/ranges. Required groups:

- `arming_mode` integer 0..3; `cmd_timeout_ms`; forward/reverse speed and throttle
  limits; servo reverse and min/max/neutral points; PID gains and slew settings.
- Battery enable/multiplier/low threshold; IMU rotate; wheel diameter/encoder holes;
  tach glitch filter; collision mask, TTC/distance/cap thresholds, stall/impact.
- All twelve `reflex.<name>.<manual|auto>` values as integers 0..3 (off/warn/pico/ros).
- `wifi_mode`, `sta_timeout_s`, `agent_port`, `ros_domain_id` as integers.
  `agent_ip` is an int64 constrained to 0..4294967295, representing existing
  host-order a<<24|b<<16|c<<8|d; zero means discover. The console retains dotted
  IPv4 input. Include examples of both forms in the ROS interface documentation.

`steer_trim` in the design is an alias, not an existing struct field: expose it as
an integer degree offset from 90, mapped to `neutral_point`. Expose `steer_trim`
as the ROS name and keep `neutral_point` as the canonical stored/console field,
not a second writable ROS alias. Console edits must refresh the trim parameter.
Do not add a duplicate flash field.

Credential/string parameters and arbitrary parameter creation/deletion are not
supported. Use `allow_undeclared_parameters=false`; provision enough `max_params`
for the counted descriptor table. Low-memory mode means one parameter per request;
atomic batch changes are not promised. Verify actual pinned behavior for
`set_parameters_atomically` and document any unsupported result.

All ROS setting changes require disarmed/no pending arm and validated ranges;
wrong types, NaN/Inf, overflow and unknown/deleted keys are rejected without changes.
Changing arming mode clears keys and the neutral latch through control. Network,
endpoint or domain changes are applied only after the current service response
has had a bounded send opportunity, then invalidate session generation and
reconnect. Parameter cache must follow console/UI updates too. Avoid recursive
parameter setter calls inside the rclc modification callback; reconcile revisions
from the owning task after the callback returns.

Saving remains explicit for ordinary changes. `Trigger.success=true` means the
settings were actually written and verified, not queued. The one exception is
first-time AP password provisioning required by design §8: generate once from
hardware entropy, reserve maintenance, persist the complete current settings
without overwriting provisioned credentials, verify it, then enable AP. Never
start an open AP if generation or persistence fails. Do not print the password;
Plan 4 reads it via an internal accessor for its local display/QR. No packet or
telemetry copy contains it. The normal masked `get ap_pass` behavior stays intact. Until the Plan4 local
password display exists, AP connection checks use an owner-provisioned, known
`ap_pass` set/saved while disarmed using the existing console. The agent does not
request or capture that value. Test automatic generation/persistence separately
with synthetic entropy/storage; do not claim that an unknown generated password
has been displayed or that QR onboarding already exists.

## File map (proposed)

```text
firmware/core/
  umb_net_fsm.{h,c}          station/AP/session policy and wrap-safe deadlines
  umb_network_identity.{h,c} AP SSID/password formatting; entropy is input
  umb_ros_command.{h,c}     pure Twist-field mapping and validation
  umb_ros_params.{h,c}      scalar schema, range mapping, setting transactions
  umb_telemetry.{h,c}       scalar conversions, timestamps and publish schedule
  umb_odometry.{h,c}        signed speed/yaw integration
  umb_reflex_events.{h,c}   transition/repeat and diagnostic severity decisions
firmware/hal/
  hal_wifi.{h,c}            CYW43/lwIP adapter, no decision policy
  hal_udp_transport.{h,c}   custom XRCE datagram callbacks
  hal_microros_platform.{h,c} allocator, clock_gettime/usleep
  hal_dhcp_server.{h,c}     adapted, licensed DHCP server and lease snapshot
firmware/app/
  net_task.{h,c}            core-0 orchestrator and sole micro-ROS owner
  ros_entities.{h,c}        creation/partial cleanup/executor lifecycle
  ros_messages.{h,c}        bounded ROS messages, telemetry adapters
  ros_services.{h,c}        arm ACK and maintenance/save adapter
  ros_parameters.{h,c}     rclc parameter glue and revision reconciliation
  settings_apply.{h,c}     shared reservation/neutral ACK/flash coordinator
  shared_state.{h,c}       snapshots, sticky loss/disarm, ACKs, event FIFO
  main.c, console_task.*, console_cmds.c, control_task.c, sensors_task.c (modified)
firmware/lwipopts.h
firmware/CMakeLists.txt, FreeRTOSConfig.h
micro_ros/library_generation/colcon.meta, README.md, build.sh (as needed)
tests/host/test_net_fsm.c, test_network_identity.c, test_ros_command.c,
  test_ros_params.c, test_telemetry.c, test_odometry.c, test_reflex_events.c,
  test_shared_state.c, test_control.c, test_safety.c
tests/net/                 socket/ROS lifecycle fakes; bounded integration tests
tools/plan3_readonly_smoke.py
Docker compose agent + ros-cli additions, .env.example
docs/ros-interface.md, docs/hil/plan3-network-checklist.md,
  docs/hil/plan3-validation.md
```

New app modules must have one clear owner. It is acceptable to combine small glue
files if that reduces needless interfaces, while keeping pure policy host-testable.
Do not write into the `umbreon_zephyr` reference project.

## Task 1: Lock resource and build contracts

**Completed:** see `docs/validation/plan3-task1.md`; production runtime entity
heap/stack acceptance remains in later tasks.

**Files:** micro-ROS colcon metadata/build docs, firmware CMake, a build-contract
check script, `docs/ros-interface.md` initial graph/budget.

- [x] Record baseline checks and actual library symbols/headers/package revisions.
- [x] Add `-DUCLIENT_CUSTOM_TRANSPORT_MTU=1200`, proposed RMW capacities above;
  rebuild Jazzy library. Verify generated headers, linked symbols and Cortex-M33
  softfp ABI against SDK objects. Do not change the toolchain to hard-float.
- [x] Fix the literal `-UCLIENT_PROFILE_STREAM_FRAMING=ON` metadata typo to the
  intended `-DUCLIENT_PROFILE_STREAM_FRAMING=ON` if retained; custom UDP registration
  still uses `framing=false`. Check flags against the pinned dependency CMake.
- [x] Fail the firmware configure/build with a clear message if library/required
  message type support is missing or generated MTU/limits violate the contract.
- [x] Add a serialized-size test/measurement for worst-case Odometry and bounded
  diagnostics before selecting final MTU. Include actual XRCE overhead.
- [x] Run micro-ROS rebuild, firmware build, existing host tests/coverage; record
  ELF data/bss, heap budget and artifact size. This is a resource gate.
- [x] Commit `build: configure micro-ROS resources for production telemetry`.

## Task 2: Close shared-state and control safety gaps

**Software status:** see `docs/validation/plan3-task2.md`. The flash latency/margin
item below remains open and must be resolved in Task11.

**Files:** shared_state, control_task, umb_control/safety, console task/handlers,
flash HAL and their host integration tests.

- [x] Write failing tests: a console beat cannot hide a hung net task; ROS stream
  timeout stops at the configured boundary; loss clears partial BOTH keys even
  if the last command was console or no ROS command ever arrived; a down/up within
  one control tick is still consumed; generation N commands/arm requests cannot
  run after reconnect to N+1; disarm cannot be dropped by a full queue.
- [x] Implement sticky loss/disarm and request-ID ACKs. Preserve offline console
  usability and distinguish boot-offline from a real session/mode loss. While a
  loss bit is active, new arm requests must not revive keys behind that bit.
- [x] Make net_task the heartbeat writer at final wiring; temporarily provide a
  deliberate build-time stub only if needed to keep intermediate firmware bootable.
  Never ship two heartbeat writers.
- [x] Add a maintenance reservation with neutral ACK and atomic settings updates.
  Test encoder/ROS arm races, simultaneous console/ROS edits, missing ACK, flash
  failure and cleanup. Reservation releases on every error, neutral latch stays
  cleared; all arm/command paths must obey it, including `mode none` in host tests.
- [ ] Flash coordinator gets the control ACK immediately after neutral PWM and a
  fresh control watchdog update. Bound lockout/flash below the 250 ms watchdog
  period with explicit measured margin; remove the flash HAL's independent
  `watchdog_update()` so control remains the feeder. Do not feed watchdog from
  network code to mask blocked control.
- [x] Run host tests + coverage, firmware compile; commit
  `fix: isolate network liveness and latch loss across command sources`.

## Task 3: Pure WiFi/session FSM and AP identity policy

**Files:** umb_net_fsm, umb_network_identity, host tests.

- [x] Tests first: empty credentials -> AP in AUTO; STA timeout at configured
  seconds -> AP; STA_ONLY never falls back; AP_ONLY never tries STA; AP with no
  session retries STA every 60 s; active/creating session prevents retry; failures
  recover with finite backoff; wrap-safe deadlines; endpoint changes invalidate
  generation exactly once; invalidated DHCP candidate is not retained forever.
- [x] Define input events (link/lease/ping/create results) and output actions; no
  socket, SDK or shared calls inside policy. A bounded action advances one step.
- [x] First attempts: STA immediate; after AP retry failure restore AP with the
  same identity; apply capped retry backoff in STA_ONLY instead of busy looping.
- [x] Identity tests: suffix from device MAC as `umbreon-XXXX`, WPA2-safe generated
  password length >=12 and <=63, distinct supplied entropy changes password,
  existing valid password preserved, invalid configured pass is rejected.
- [x] Run host tests/coverage; commit `feat: network recovery and AP identity policy`.

## Task 4: Production UDP, WiFi and DHCP adapters

**Files:** hal_wifi, hal_udp_transport, hal_dhcp_server, lwipopts, tests/net.

- [ ] Use spike UDP code as reference, not a blind copy. Test with fake socket
  operations: open/bind/close failures; duplicate close; short/error send; timeout
  zero uses nonblocking read; positive timeout obeyed; setsockopt errors handled;
  oversized/truncated packet rejected rather than delivered as valid XRCE bytes.
- [ ] A session endpoint is a fixed IP+port; reject datagrams from a different
  source, maintaining the original read deadline under unrelated packets.
  Endpoint discovery uses a separate bounded probe context and validates XRCE
  ping replies; it must not retarget a live session on arbitrary traffic.
- [ ] STA join starts asynchronously. Initialize CYW43 from the core-0 task,
  disable power save after join, collect link/IP/RSSI via supported locking.
- [ ] Port DHCP from the SDK example, preserving license and pinned source credit.
  Protect all RAW API operations with tcpip-thread execution/core-lock as required
  by NO_SYS=0; callback copies bounded lease metadata only, never calls rcl.
- [ ] Harden DHCP packet/option bounds, PAD/END handling and chained pbuf reads;
  malformed requests cannot read outside the packet. Test offer/ACK and expiry,
  lease replacement, truncated options and repeated requests without heap growth.
- [ ] AP is 192.168.4.1/24; choose a bounded lease pool beginning at .2, record
  candidate IPs in lease order and retry candidates fairly. Do not advertise a
  DNS server unless one is actually implemented. No captive portal required.
- [ ] Prove closing/reopening WiFi modes releases DHCP PCBs/sockets and clears old
  endpoint candidates. Run adapter tests and firmware compile; commit
  `feat: bounded CYW43 UDP transport and AP DHCP adapter`.

## Task 5: Platform hooks and core-0 network skeleton

**Files:** hal_microros_platform, net_task skeleton, CMake, lwipopts, FreeRTOSConfig,
main/task pinning, allocator tests.

- [ ] Install one FreeRTOS-backed rcl allocator before all rcl initialization.
  Test alignment, null/zero realloc semantics, preserved contents, overflow in
  size+header/count*size and failed realloc leaving the old allocation intact.
- [ ] Implement monotonic clock_gettime/usleep glue once; sleep yields appropriately
  after scheduler start. Avoid duplicate libc symbols with SDK/micro-ROS objects.
- [ ] Link `pico_cyw43_arch_lwip_sys_freertos` and static library, carry forward
  measured spike lwIP configuration. Verify TCP/IP/async-context core affinity,
  stack sizes and priorities from actual SDK 2.2.0 sources. Keep core-1 timer pool.
- [ ] Add network task idle+3/core0 and sufficient initial stack from measured
  lifecycle usage. Keep total allocated RTOS heap + static bss within physical
  RAM. Audit libc `malloc` users versus the rcl/FreeRTOS allocator too.
- [ ] A network-disabled/offline stub must maintain safe neutral boot; final
  heartbeat ownership change is atomic with enabling the production loop.
- [ ] Firmware clean build + unit tests; commit
  `feat: micro-ROS platform hooks and core-0 network task`.

## Task 6: WiFi mode orchestration and endpoint selection

**Files:** net_task WiFi stages, settings_apply provisioning, shared_network.

- [ ] Connect the pure FSM to adapters. Offline operation still gives bounded
  heartbeat iterations and a responsive console. No 15-second blocking join loop.
- [ ] On first missing AP password: generate from Pico RNG/hardware entropy,
  acquire maintenance, persist/verify once while neutral, release, and start AP.
  Persistence failure leaves AP disabled and a clear non-secret error; retry with
  backoff, no flash-write loop. Already provisioned devices perform no boot write.
- [ ] STA configured IP takes priority. Zero IP -> bounded subnet-broadcast XRCE
  probe; optional multicast only if verified on this transport/library. After a
  bounded round, remain offline and retry instead of blocking the task.
- [ ] AP mode probes DHCP candidates even if a STA fixed IP exists. Only a valid
  XRCE agent reply selects the endpoint; expired/disconnected lease invalidates
  selection. Never switch WiFi mode while a session is ready or being created.
- [ ] Before mode/endpoint changes, notify loss and clear ready flags; tear down
  old objects/socket before reconfiguration. No pending command crosses sessions.
- [ ] Fake-time end-to-end tests cover STA/AP retry, owner setting changes and a
  phone-first/PC-second DHCP lease sequence. Commit
  `feat: station and AP fallback with bounded agent selection`.

## Task 7: Command mapping and acknowledged arming

**Files:** umb_ros_command, ros_services arm handler, control/shared ACK tests.

- [ ] Failing pure tests cover all three topic mappings, signed reverse, zero
  command, steering clamp, NaN/Inf, unsupported axes and no mailbox write on reject.
- [ ] ROS callbacks stamp receive time locally and post only for the current ready
  generation. History depth1; no replay of cached commands during lifecycle setup.
  Document that Twist has no sender timestamp: keep-last cannot independently
  identify every delayed network packet; it is not an authenticated/freshness
  protocol. No auto-resume because disarm/neutral latch remain mandatory.
- [ ] Arm(true) refuses a dead/stale session or active maintenance. Queue accepted
  requests with IDs and wait for actual control completion with a <=60 ms bound;
  control runs independently. Message differentiates `waiting for car arm`,
  `waiting for ros arm`, armed, refused mode and busy/timeout.
- [ ] Timeout/cancellation cannot leave a supposedly failed arm request queued to
  arm later. Tests include request timing out just before a control tick and
  generation change while waiting. Disarm(false) has priority and clears both keys.
- [ ] Host tests/coverage + firmware compile; commit
  `feat: ROS command mapping and control-acknowledged arming`.

## Task 8: Pure telemetry, timestamp mapping and odometry

**Files:** umb_telemetry, umb_odometry, sensor snapshot extensions and tests.

- [ ] Pin/document Range profile metadata from ST evidence (8190 is a sentinel
  boundary, not a sensor-range claim), and any bench limits still to be measured.
  ST does not give a universal minimum independent of optics/cover glass: record
  a conservative application minimum as a chosen limit, not a datasheet guarantee.
- [ ] Conversion tests: Range finite/9999/stale/offline/fresh OOR/below-min/above-max; Imu SI units and
  unsupported orientation; invalid/nonfinite inputs never create healthy samples;
  battery unknown fields; fixed bounded frame names and per-sensor stamps.
- [ ] Timestamp tests: epoch offset, unsynced suppression, acquisition vs publish
  time, uint32 wrap, future-sample tolerance, failed resync and backwards epoch
  adjustment with monotonic safety/odometry unaffected.
- [ ] Odometry tests: forward/reverse straight motion, turn arc, heading crossing
  +/-180/360, normalization, IMU failure/recovery, long dt ignored, restart pose
  reset and reconnect pose retention. Use measured signed speed, not target.
- [ ] Cadence tests: IMU100/odom50/range25/battery1/diagnostics1; long scheduler gap
  emits current data once, never a catch-up flood. Snapshot acquisition generation
  prevents repeated raw samples being labelled as newly acquired.
- [ ] Host tests/coverage; commit `feat: SI telemetry and monotonic odometry`.

## Task 9: ROS entities, messages and recoverable lifecycle

**Files:** ros_entities, ros_messages, net_task session stages, tests/net.

- [ ] Set unique device client key, ROS domain from settings and exact node/topic
  names/QoS. Initialize service/message storage to known empty states.
- [ ] Build all entity types with a creation bitmap/state tracker. Every init step
  can fail: tests inject failure after each successful predecessor and prove all
  initialized resources are finalized once in reverse dependency order.
- [ ] Use generated message init/fini correctly with preallocated bounded data;
  never let generated fini free a static string as if it were heap-owned. Keep
  allocation ownership explicit and test repeated setup/cleanup.
- [ ] Keep `agent_up=false` until the session and required entities/executor are
  ready; a pingable agent alone is insufficient. Establish synchronized time
  before timestamped telemetry. Commands and services still obey readiness gates.
- [ ] One scheduler timer updates prepared messages; throttle due publications
  with the pure cadence state. Treat executor timeout as normal; repeated actual
  transport/rcl failures trigger loss instead of ignored error accumulation.
- [ ] On loss: clear link/ready + latch loss first; zero destroy-session timeout
  before fini; release buffers/objects/sockets even on dead agent. New generation
  uses fresh subscriptions/executor buffers and never reuses old pending requests.
- [ ] Fake lifecycle + wire-size tests, firmware build; commit
  `feat: production ROS graph and leak-free session recovery`.

## Task 10: Parameter server and settings reconciliation

**Files:** umb_ros_params descriptors, ros_parameters, atomic settings operations,
ROS interface documentation and tests.

- [ ] Count and list all implemented parameters/types/ranges in docs. No silent
  omissions of reverse limits or reflex settings. Use existing settings validators
  without double-to-float/int overflow or locale-dependent string round trips.
- [ ] Initialize low-memory rclc server, event publisher and all six services;
  reserve handles with `RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES`, not a guessed5.
- [ ] Tests: valid scalar edit, bad type/range/NaN/Inf, unknown/delete refused,
  armed/pending-arm/maintenance refusal, alias coherence, conflicting producers,
  and all 12 reflex policies mapped to the right mode/index.
- [ ] Read reflects console updates on the next bounded reconciliation iteration.
  Cache revision cannot overwrite a newer concurrent setting. Internal refresh
  must not be mistaken for a forbidden external write while armed.
- [ ] Network/domain edits commit settings, give response a send opportunity,
  then invalidate/recreate. Confirm no recursive setter in modification callback.
- [ ] Test declared low-memory batch/atomic limitations through a host/fake client
  and document exact ROS CLI behavior. Commit
  `feat: validated ROS parameters backed by shared settings`.

## Task 11: Verified settings save service

**Files:** settings_apply, ros_services save handler, console save integration,
flash adapter tests.

- [ ] Use the same reservation+neutral ACK path for ROS save, console save and
  initial AP provisioning. A global flash transaction prevents concurrent saves
  from sharing the static flash buffer. No flash work from control.
- [ ] Trigger returns success only after encode/write/readback succeeds; include
  non-secret slot/sequence in result. Refused armed/busy/neutral-ACK timeout must
  perform no erase/program. Partial failure does not claim the new sequence.
- [ ] Test concurrent arm request during save, flash timeout/readback failure,
  reset/power-loss slot selection (pure blob tests), retry and reservation release.
- [ ] Bound total callback time (neutral ACK <=60 ms + measured flash margin) and
  count flash-induced control pauses separately from ordinary jitter. If worst
  erase/program cannot safely fit watchdog margin, redesign this path before
  hardware tests; never solve it with an unbounded watchdog feed outside control.
- [ ] Host tests/firmware build; commit
  `feat: disarmed verified settings persistence service`.

## Task 12: Diagnostics, reflex events and network cues

**Files:** umb_reflex_events, ros_messages diagnostics, shared FIFO, control and
sensor counters, umb_buzzer/UI cues, console status additions.

- [ ] Tests: event on rise/action change/clear; 2 Hz repeat while active; WARN for
  warn/delegated, ERROR for Pico intervention, OK for clear. Preserve event order,
  timestamps and affected values, including a rise+clear between publish ticks.
- [ ] Diagnostics include armed/keys/mode, stop bits and command age, active and
  delegated reflex masks, ToF online/recovering/restarts/errors, IMU health,
  WiFi mode/IP/endpoint/RSSI, session generation/reconnects/ping age, time sync,
  control overruns, network loop max, dropped events and heap/stack watermarks.
  Missing counters say unsupported/unavailable, never a fabricated zero.
- [ ] Bound all names/values/sequences and test maximum serialization size at the
  MTU budget. Never include password or complete settings blobs.
- [ ] Network WiFi-up, agent-ready and agent-lost cues are selected by pure logic;
  preserve battery/reflex/disarm cue priority over success tones. No repeated
  connection beep every loop; test transitions and competing cues.
- [ ] `status`/a non-secret network status subsection exposes current network
  stage/endpoint/sync so disarmed HIL is possible without OLED.
- [ ] Host tests/coverage and firmware compile; commit
  `feat: network and reflex diagnostics with transition cues`.

## Task 13: Integration, Jazzy verification tools and disarmed smoke

**Files:** docker/compose.yml agent/ros-cli services, .env.example,
tools/plan3_readonly_smoke.py, tests/net, docs/ros-interface.md,
docs/hil/plan3-validation.md, final main/task wiring.

- [ ] Add only this project's agent and CLI verification services: matching Jazzy,
  host network, shared IPC when required by DDS, explicit ROS_DOMAIN_ID. Keep
  host Humble out of containers. Full desktop/ROS packages remain Plan 5.
  `.env.example` contains no WiFi secrets. Do not hard-code office IPs as defaults.
- [ ] Agent port from configuration must match selected endpoint; log the actual
  command/port/domain. Keep spike UDP relay as an optional documented workaround
  for the observed VPN route, not an unconditional production dependency.
- [ ] Final wiring: one communication heartbeat writer; all network helpers
  core0; control PWM core1; correct generation/loss/maintenance state flow.
- [ ] Run `make fw`, `make test`, `make coverage` and tests/net. If sandbox needs
  approved Docker/USB access, use supported escalation, not guessed host bypasses;
  native checks are fallback evidence, recorded explicitly rather than relabelled
  Docker success. No implementation test is marked passed from a prior build.
- [ ] Run 100 simulated partial-create/loss/reconnect cycles with stable live
  allocation count, bounded stack and cleared queues. Run transport silence and
  false-packet floods; core stop logic remains correct with a live console.
- [ ] Read-only hardware smoke: flash, record status/sensors, start project agent,
  list node/topics/services/parameters, get params, echo each sensor type with
  best-effort QoS and measure topic rates. Observe at least 60 s, record losses,
  sync state, heap/stack/control jitter and firmware hash. No drive publication,
  arm call, parameter change or save from the agent.
- [ ] Stop/restart only the project's agent while disarmed; observe session down,
  reconnect and unchanged neutral outputs/no key restoration. Record 10 hardware
  cycles with heap watermarks; no claim of zero-loss WiFi. AP fallback/mode-change
  tests are owner-run to avoid disconnecting the owner's current network.
- [ ] No stale data advertised healthy; no stable-case project compiler warning,
  stack overflow, malloc failure, watchdog reboot or monotonic heap loss. Record
  actual rates rather than claiming target100/50/25 on a lossy network.
- [ ] Commit `feat: integrate WiFi micro-ROS runtime and verification tools` and
  `docs: record plan 3 disarmed network validation` as appropriate.

## Task 14: Owner network-loss and AP acceptance checklist

**File:** `docs/hil/plan3-network-checklist.md` plus optional owner-only helpers.
Every scenario states preconditions, exact command/expected observation, timeout,
what it proves and what it cannot prove. Helpers default to read-only; no automatic
arming/motion/save hidden in a smoke script.

- [ ] Station normal boot: association, IP, configured endpoint, node/parameter
  discovery, synchronized stamps and telemetry rates; record RSSI and network.
- [ ] Before AP connection tests, owner provisions a known valid `ap_pass` while
  disarmed if Plan4 password display is not available. Keep it out of logs. Check
  generated-once behavior separately; no fake claim that a password was displayed.
- [ ] Router unavailable/wrong credentials: AUTO -> AP after sta_timeout, DHCP
  gives PC .2 or later, agent discovered by validated lease candidate, stable
  protected SSID/password across reboot. Phone-first/PC-second case works.
- [ ] AP without session -> retry STA at60s; AP with ready/creating session does
  not switch. STA_ONLY/AP_ONLY obey their preference. Persistence-error path
  never exposes an open AP and never repeatedly erases flash.
- [ ] On a stand, owner explicitly arms and commands a small speed. Command
  silence -> neutral by configured timeout + one control cycle (default <=270ms),
  fresh neutral required to resume. Owner, not agent, verifies physical stop.
- [ ] Kill project agent -> timeout stops motion independently of detection;
  ready/session loss declared <=400ms target then keys cleared. Repeat while a
  console command was last and with only one BOTH key latched; loss still clears
  it. A live USB console cannot mask a suspended/hung net task (>500ms stale
  heartbeat + one cycle). Host fault simulation covers cases unsafe on hardware.
- [ ] Router off/on, AP-client disconnect and wrong agent route -> stop/disarm;
  network restore never resumes an old nonzero command or restores old keys.
  Test reverse/brake sequence interruption on stand. Do not disable the owner's
  shared network as part of an automated test.
- [ ] Arm service responses match actual keys/mode and no late arm after a timed
  out/refused request. Wrong/NaN commands do not refresh command age. Save/param
  changes refused while armed; successful owner save survives reboot.
- [ ] Measure RTT on the intended router or car AP; compare with Phase0's marginal
  office link. Report p50/p95/max/loss, false session losses and control jitter
  independently. This is not a promise of a 50ms tail on every AP.
- [ ] Record outstanding Plan2 physical/audio checks and Plan4/5 dependencies;
  bag/RViz/full teleop E2E acceptance remains Plan6. Finish disarmed and neutral.
- [ ] Commit `docs: plan 3 station AP and link-loss acceptance checklist`.

## Review focus and finish criteria

1. Agent disappears during the third entity create, or returns mid-fini: no leak,
   double free, stale rcl handle or revival of an old generation.
2. Console works but net_task is hung: core1 stops/disarms; console beat cannot
   hide it. No link level/event is lost due to the last source being console.
3. Delayed session data arrives after reconnect: fresh subscription state and
   neutral/arming gates keep outputs neutral; invalid packets do not refresh age.
4. Large Odometry/diagnostics: valid whole messages reach the PC over best-effort,
   no silent MTU failure and no unbounded string/sequence allocation.
5. Set parameter then get via console, and edit console then get parameter: one
   coherent settings state; no recursive callback, old-copy overwrite or flash
   side effect. Save success means verified storage under arm inhibition.
6. WiFi AP has multiple clients: only a ping-validated endpoint becomes agent;
   DHCP/RAW thread context correct, no blocking discovery, no leaked AP password.
7. Firmware stays within SRAM and reconnect watermarks stabilize. The host harness
   cannot substitute for observed hardware timing, nor can reported PWM substitute
   for a physical ESC stop test.

Software completion means all implemented paths and counts are documented, all
required build/host/fake-network checks pass, and safe disarmed hardware evidence
is recorded or explicitly marked unavailable. Plan 3 owner acceptance remains open
until the motion/AP/loss checklist is returned. Do not mark later physical phases
complete on the strength of a topic list alone.

## Primary references and version caveats

Local code and generated headers take precedence over generic examples:

- `spike/{main.c,udp_transport.c,lwipopts.h}` and `docs/spike-results.md` are measured
  RP2350/CYW43/FreeRTOS evidence, not production lifecycle code.
- `micro_ros/libmicroros/include/rclc_parameter/rclc_parameter.h` defines the actual
  scalar types, six handles and callback restrictions for this build. Pinned rclc
  revision is listed in `micro_ros/built_packages`.
- [micro-ROS parameter server source documentation](https://github.com/micro-ROS/micro-ros.github.io/blob/master/_docs/tutorials/programming_rcl_rclc/parameters/parameters.md):
  API/background; inspect the pinned header for resource counts and batch support.
- [Micro-XRCE-DDS Client CMake options](https://github.com/eProsima/Micro-XRCE-DDS-Client/blob/83f129a80770a09aac9e823896ecbf6a0eddf0fc/CMakeLists.txt):
  verify `UCLIENT_CUSTOM_TRANSPORT_MTU` against the pinned revision when rebuilding.
- [Raspberry Pi Pico SDK documentation](https://www.raspberrypi.com/documentation/pico-sdk/networking.html):
  CYW43 asynchronous station and AP APIs. The local SDK2.2.0 async-context,
  FreeRTOS sys_arch and flash-safe sources govern task/lock/flash behavior.
- [ST VL53L0X datasheet](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
  documents nominal reach/FoV; the 20ms profile still needs its own honest limits.
  [ST technical moderator on minimum distance](https://community.st.com/mems-sensors-48/minimum-range-vl053l0-54599)
  explains its dependence on the optical assembly rather than one universal value.
- [ROS Jazzy Range message contract](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Range.html)
  defines metadata/units; use the Jazzy generated fields, not Rolling-only additions.
- SDK examples' `pico_w/wifi/access_point/dhcpserver` are a porting reference;
  preserve their license and harden parsing/context use for the production build.

This plan does not promise upstream discovery that the current archive does not
contain, ROS string parameters that rclc does not support, a sensor range that has
not been calibrated, or real-time network delivery on the measured office link.
