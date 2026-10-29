# Umbreon Pico 2 W ROS2 Firmware — Design

Date: 2026-10-29
Status: approved by owner 2026-10-29

Legend: **[user]** = decided by the owner in conversation. **[my call]** = chosen by the
designer, open to change. Everything else is context or left to the builder.

## 1. Intent

The Umbreon roborace car today runs `umbreon_zephyr`: standalone autonomous firmware on a
Raspberry Pi Pico 2 that reads 6 ToF sensors + IMU + tachometer, makes its own driving
decisions, and talks to the outside world through an ESP8266 bridge.

The owner wants a **new firmware for a Raspberry Pi Pico 2 W** that turns the car into a
**ROS2 robot driven from a PC over WiFi**. The firmware makes no driving decisions; it
receives commands and transmits sensor data. All intelligence lives on the PC. **[user]**

The PC workflow it must support **[user]**:
- teleop with a gamepad,
- the owner's own autonomous driving node,
- rviz visualisation,
- recording data (rosbag) for later analysis and tuning.

## 2. Goals, non-goals, anti-goals

### Goals
1. Same car, same sensors, same wiring; only the Pico 2 + ESP8266 are replaced by a
   Pico 2 W (on-board CYW43439 WiFi). **[user]**
2. ROS2 integration via **micro-ROS** (agent on the PC, UDP over WiFi). **[user]**
3. Built on the **Raspberry Pi Pico SDK**, using **both RP2350 cores**. **[user]**
4. **All existing hardware keeps working**: 6× VL53L0X, MPU-6050, tachometer, servo, ESC,
   battery ADC, SSD1306 OLED, rotary encoder + button, piezo buzzer, persistent settings.
   **[user]**
5. **OLED UI and encoder usage are refined** for the ROS2 role (status/telemetry focus).
   **[user]**
6. **Fail-safe: if the link drops, the car stops.** **[user]**
7. **Arming is configurable.** **[user]** (modes in §6.)
8. PC side runs **ROS2 Jazzy (LTS) in Docker with all tools**. **[user]**
9. **Safety reflexes can run on the Pico** (faster than a PC round trip) **or on the ROS
   side**, chosen per reflex and per driving mode. **[user]** (§6.3)

### Non-goals
- Porting the standalone autonomous algorithm (`control.c`: PID wall-following,
  wrong-direction detection, manoeuvres) into firmware. The firmware never steers or
  drives toward a goal by itself; it may only **limit or override** PC commands through the
  safety reflexes in §6.3. **[my call]** — porting it to a PC-side ROS2 node may be a later project.
- Supporting the ESP8266 or the original (non-W) Pico 2.
- Modifying `umbreon_zephyr`. It stays as-is and is used as reference/source of driver logic.
- OTA firmware updates (UF2 over USB is enough for now).

### Anti-goals (failure even if it "works")
- The car moves when nobody intends it: after boot, after reconnect, after a crash of the
  PC node, or after a firmware reset.
- WiFi / micro-ROS stalls delay the stop-on-silence watchdog or servo/ESC updates.
- WiFi credentials or other secrets committed to source.
- An OLED/encoder UI that is harder to use than today's.

## 3. Constraints and facts

- **Host PC**: Ubuntu 22.04, ROS2 Humble installed natively, Docker 29 + Compose v2,
  Pico SDK at `~/pico/pico-sdk` (plus picotool, openocd, debugprobe).
- **ROS2 distro**: **Jazzy Jalisco** (LTS, supported to 2029, fully supported by micro-ROS,
  official `microros/micro-ros-agent:jazzy` image) **[user]**. Firmware library and agent
  must use the same distro. Moving to Lyrical later is a rebuild, not a redesign.
- **micro-ROS on RP2350**: the official `micro_ros_raspberrypi_pico_sdk` targets RP2040
  (Cortex-M0+); RP2350 support is an open upstream request. The static library must be
  rebuilt for Cortex-M33 (hard-float) using the micro-ROS static library builder. A custom
  UDP transport over the CYW43 lwIP stack is required (upstream port is serial/USB).
- **Pin compatibility**: Pico 2 W uses GP23/24/25/29 internally for the CYW43439. The car
  wiring uses none of them; no hardware change needed. Note: GP25 is no longer the user LED
  (LED is behind the CYW43, `cyw43_arch_gpio_put`).

### Pinout (unchanged from `umbreon_zephyr/pinout.md`, ESP8266 removed)

| GPIO | Function | Notes |
|------|----------|-------|
| GP0/GP1 | I2C0 SDA/SCL 400 kHz | MPU-6050 `0x68`, SSD1306 `0x3C` |
| GP2/GP3 | I2C1 SDA/SCL 100 kHz | 6× VL53L0X, re-addressed `0x30–0x35` |
| GP6,7,8,9,14,15 | XSHUT sensors 0–5 | HR, FR, R, L, FL, HL |
| GP10 | PWM5A 50 Hz | Steering servo |
| GP11 | PWM5B 50 Hz | ESC, neutral 1500 µs |
| GP12 / GP22 / GP19 | Encoder DT / CLK / button | active low |
| GP13 | Tachometer | rising-edge IRQ |
| GP16/GP17 | UART0 115200 | debug console |
| GP18 | Piezo buzzer | GPIO square wave |
| GP26 | ADC0 | battery, 18k/10k divider |
| GP4/GP5 | **free** | were ESP8266 UART1 |

## 4. Architecture

```
 PC (Docker, ROS2 Jazzy, --net=host)                    Pico 2 W
 ┌──────────────────────────────┐   WiFi / UDP   ┌────────────────────────────────┐
 │ micro_ros_agent udp4 :8888   │◄──────────────►│ Core 0: CYW43 + lwIP, micro-ROS │
 │ teleop (joy + umbreon_teleop)│                │  executor, pubs/subs/services  │
 │ autonomous node (owner's)    │                ├──────── shared state ──────────┤
 │ rviz2, rqt, ros2 bag         │                │ Core 1: control loop, watchdog, │
 │ robot_state_publisher (URDF) │                │  arming FSM, servo/ESC, sensors,│
 └──────────────────────────────┘                │  IMU, tach, battery, OLED, enc, │
                                                 │  buzzer                         │
                                                 └────────────────────────────────┘
```

### 4.1 Firmware runtime **[my call]**
- **FreeRTOS SMP** (FreeRTOS-Kernel RP2350 port shipped with pico-sdk) with core affinity:
  - **Core 0**: WiFi (`pico_cyw43_arch_lwip_sys_freertos`), micro-ROS executor task,
    console task.
  - **Core 1**: control task (highest priority, 50 Hz), ToF scan task, IMU task, battery
    task, UI task (OLED + encoder + buzzer).
- Rationale: ToF scanning on 100 kHz I2C and OLED rendering are slow and blocking; separate
  tasks keep the control/watchdog loop deterministic, and pinning networking to core 0
  isolates WiFi jitter from actuation (anti-goal).
- Builder may choose bare-metal multicore (`pico_multicore` + cooperative loops) instead if
  FreeRTOS + micro-ROS proves problematic in Phase 0, as long as the isolation property
  holds.

### 4.2 Data flow between cores
- Single-writer snapshots (sensor data, status) and a latest-value command mailbox, protected
  by short critical sections / spinlocks. No blocking waits across cores in the control path.
- The control task is the **only** writer of servo/ESC PWM.

### 4.3 Code reuse from `umbreon_zephyr`
Port logic, rewrite platform plumbing (Zephyr device/sensor/NVS APIs → pico-sdk
`hardware_i2c/pwm/adc/gpio/flash`):
- `sensors.c` + `modules/vl53l0x_enhanced` → VL53L0X driver (XSHUT sequencing,
  re-addressing, recovery logic and statistics).
- `imu.c` → MPU-6050 (gyro bias calibration, accel calibration).
- `tachometer.c` → GPIO IRQ with glitch filter, speed estimate.
- `battery.c`, `buzzer.c`, `encoder.c` (event decoding: CW/CCW/click/double/hold/long-hold).
- `car.c` → servo/ESC mapping, servo reverse/trim, ESC calibration, speed PID.
- `display*.c` + u8g2 → OLED (u8g2 with pico-sdk I2C byte callback).
- `settings*.c` → settings struct, validation, versioned storage (re-implemented on raw flash).

## 5. ROS2 interface **[my call]**

Namespace `/umbreon`. Frames: `base_link`, `imu_link`, `tof_hr|fr|r|l|fl|hl` (static TFs
come from a URDF on the PC, not the firmware). Only standard message packages, so the
micro-ROS library needs no custom interfaces.

### Inputs (PC → car)
| Name | Type | Notes |
|------|------|-------|
| `/umbreon/cmd_vel` | `geometry_msgs/Twist` | `linear.x` = target speed m/s (closed loop via tach PID); `angular.z` interpreted as **normalised steering** −1…+1 (left…right), not yaw rate — documented, simple for teleop and owner node. Best-effort QoS. |
| `/umbreon/cmd_teleop` | `geometry_msgs/Twist` | Same meaning as `cmd_vel`. Published by `umbreon_teleop` (gamepad). Commands here put the car in **manual mode**, commands on `cmd_vel` in **autonomous mode** (§6.3). **[user: manual vs autonomous behaviour; my call: mode by topic]** |
| `/umbreon/cmd_raw` | `geometry_msgs/Twist` | Open loop: `linear.x` = throttle −1…+1, `angular.z` = steering −1…+1. For tuning/ESC testing. |
| `/umbreon/arm` | `std_srvs/SetBool` service | ROS-side arm/disarm. |
| parameters | rclc parameter server | `arming_mode`, `cmd_timeout_ms`, `max_speed_ms`, `max_throttle`, `steer_trim`, `servo_reverse`, PID gains, battery thresholds. Persisted on `save` (or `/umbreon/save_settings` `std_srvs/Trigger`). |

Whichever command topic received the most recent message wins; mixing is not supported.
`cmd_raw` counts as manual mode.

**Reverse driving from ROS is in scope** **[user]**. Handling **[my call]**:
- Negative `linear.x` on `cmd_vel` or `cmd_raw` means reverse. The firmware runs the ESC's
  brake → neutral → reverse sequence itself (timings/pulses from the existing
  `reverse_brake_cmd/ms`, `reverse_drive_cmd/ms` settings), so PC nodes just send signed
  values.
- The tachometer has no direction, so the firmware tracks a **gear state** (forward / braking /
  reverse) and signs the measured speed from it. Closed-loop PID runs on speed magnitude in
  both directions; reverse speed is capped by a separate `max_reverse_speed_ms` setting.
- Direction changes only go through the brake sequence when the measured speed is near zero
  or after the brake time. Any stop condition during the sequence goes straight to neutral.
- `odom` uses the signed speed.

Teleop and the autonomous node **do not run at the same time for now** **[user]**; running
them together (gamepad supervising autonomy) is a later feature **[user]**. Design for it
without building it: manual and autonomous commands already arrive on separate topics, so a
later "gamepad overrides autonomy" rule (manual wins while teleop is active) can be added in
the firmware's command selection or a PC-side mux without changing topics. Teleop and autonomy launch files are separate. **[my call]**

### Outputs (car → PC)
| Name | Type | Rate |
|------|------|------|
| `/umbreon/range/{hr,fr,r,l,fl,hl}` | `sensor_msgs/Range` (IR, min/max from driver, invalid → +inf) | per ToF scan (~20–30 Hz) |
| `/umbreon/imu/data_raw` | `sensor_msgs/Imu` (gyro + accel, no orientation) | 100 Hz |
| `/umbreon/odom` | `nav_msgs/Odometry` (tach speed + IMU yaw dead reckoning) | 50 Hz |
| `/umbreon/battery` | `sensor_msgs/BatteryState` | 1 Hz |
| `/umbreon/reflex` | `diagnostic_msgs/DiagnosticStatus` — one message per reflex state change (name, level WARN/ERROR, action taken: `warn` / `pico-intervened` / `ros-delegated`, key values such as distance, time-to-collision, speed) | on change, plus 2 Hz repeat while active |
| `/umbreon/diagnostics` | `diagnostic_msgs/DiagnosticArray` (armed state, arming mode, drive mode, active reflexes, link, cmd age, sensors online, ToF restarts, I2C errors, loop overruns, WiFi RSSI, reset reason) | 1 Hz |

- Timestamps in PC time via micro-ROS session time sync (`rmw_uros_sync_session`), re-synced
  periodically.
- Sensor topics best-effort QoS.

## 6. Safety and arming

### 6.1 Stop conditions (always active, independent of arming mode)
The control task (core 1) forces **ESC neutral (1500 µs) and steering centre** when any of:
1. no valid command for `cmd_timeout_ms` (default **250 ms**, configurable 100–1000);
2. micro-ROS agent unreachable (ping fails; core 0 sets a flag);
3. WiFi disconnected;
4. battery below `bat_low` (if enabled);
5. core 0 heartbeat stale (core 0 hung) — checked by core 1;
6. disarm request from any source.

Loss conditions 2, 3, 5 and low battery also **disarm** (where arming is required), so the
car never resumes motion by itself when the link returns. Condition 1 alone only stops.
**[my call]**

- Hardware watchdog (`watchdog_enable`) fed only by the control task; on reset PWM outputs
  are idle until init writes neutral. Builder must verify the ESC treats "no pulses" as stop
  and document it.
- Boot always starts **disarmed** with neutral outputs, regardless of mode.

### 6.2 Arming modes **[user: configurable]** — selected on OLED settings and as ROS parameter, persisted
| Mode | Car accepts motion commands when |
|------|---------------------------------|
| `none` | always (stop conditions still apply) |
| `encoder` | armed by encoder long-press on the car |
| `ros` | armed via `/umbreon/arm` (gamepad button through `umbreon_teleop`) |
| `both` | armed on the car **and** via ROS |

Default: `both` **[my call]**. Disarm: encoder click on the status screen, ROS `arm=false`,
or any loss condition. Armed/disarmed is always visible on the OLED and signalled by the
buzzer.

Arming FSM details **[my call]**:
- `both` is a **two-key rule**: car key (encoder long-press) and ROS key (`/umbreon/arm`)
  are independent latches, set in either order, no time window. The car is ARMED only when
  both are set. `/umbreon/arm(true)` with the car key unset succeeds with
  `message="waiting for car arm"`; the OLED shows which key is missing.
- **Any disarm clears both keys** (from any source or loss condition), so re-arming always
  needs both again.
- **Re-enable latch in every mode, including `none`**: after boot, any loss condition, or any
  disarm, motion commands are ignored until a **neutral command** (|throttle| and |speed| ≈ 0)
  has been received. In `none` this replaces arming, so the car never resumes motion on its
  own when a stale non-zero command stream comes back (anti-goal). `none` can be set over
  ROS and the OLED; the OLED shows a "NO ARMING" warning on the status screen.

### 6.3 Safety reflexes **[user: both Pico and ROS options; defaults agreed]**

Reflexes are separate from the always-on link protections in §6.1 (WiFi, agent, command
silence, core-0 hang), which are not configurable and always run on the Pico.

**Driving mode** comes from the command topic: `cmd_teleop`/`cmd_raw` = **manual**,
`cmd_vel` = **autonomous**.

Each reflex has a setting **per driving mode**: `off`, `warn`, `pico` or `ros`. Settings are
editable on the OLED and as ROS parameters, and persisted in flash.

| Setting | Behaviour |
|---------|-----------|
| `off` | not evaluated |
| `warn` | Pico detects and reports (`/umbreon/reflex`, OLED, buzzer); does not touch the command |
| `pico` | Pico detects, intervenes within the same 20 ms control cycle, and reports |
| `ros` | Pico detects and reports with action `ros-delegated`; the PC node `umbreon_safety` (§10) decides and filters or stops commands |

| Reflex | What it detects | Pico intervention | Manual default | Auto default |
|--------|-----------------|-------------------|----------------|--------------|
| `battery` | voltage below `bat_low` | stop + disarm (= §6.1 #4) | `pico` | `pico` |
| `collision` | moving forward and time-to-collision on the ToF sensors in `collision_sensor_mask` (default FR + FL, so side sensors near walls don't trigger it) below `ttc_ms` (default 400 ms), or distance below `min_front_mm` (default 150) | brake, then neutral; forward commands blocked while the condition holds, reverse allowed | `warn` | `pico` |
| `speed_cap` | obstacle near: allowed speed falls linearly from `max_speed_ms` at `cap_far_mm` (default 1000) to `cap_min_ms` (default 0.2) at `cap_near_mm` (default 300) | clamp target speed (SPEED) or throttle (RAW) | `warn` | `pico` |
| `stall` | ESC forward/reverse above `stall_min_us` offset (default 60 µs) with no tach pulse for `stall_ms` (default 700) | neutral until a neutral command | `warn` | `pico` |
| `impact` | accel magnitude above `impact_g` (default 4 g) or car upside down (accel Z < −0.5 g for 500 ms) | stop + disarm | `warn` | `pico` |
| `sensor_fault` | ToF online count below 6 or ToF in recovery, or IMU not OK | ToF: speed capped to `cap_min_ms`; IMU fault: `impact` unavailable, warn only | `warn` | `pico` |

Thresholds and `collision_sensor_mask` are settings (defaults above) **[my call]**. `speed_cap` uses the same sensor mask. A Pico intervention
always sends a `/umbreon/reflex` event and is shown on the OLED (reflex name) and buzzer
(short double beep). In manual mode with `warn`, `umbreon_teleop` may also rumble the gamepad
if it supports force feedback **[my call, optional]**.

## 7. OLED and encoder UI (refined) **[user: keep + refine]**

Screens (rotate to switch, click/hold to act) **[my call]**:
1. **Status** (default): WiFi RSSI + IP, agent connected, ARMED/DISARMED (large), arming
   mode, battery V, last command age, speed.
2. **Sensors**: 6 ToF distances in car-layout diagram, IMU yaw rate, tach speed.
3. **Network**: SSID, IP, agent IP:port, ROS distro, domain ID, reconnect count.
4. **Settings**: edit with encoder; saved to flash. Includes arming mode, cmd timeout,
   speed/throttle limits, steering trim/limits/reverse, PID, wheel diameter, encoder holes,
   battery multiplier/low, tach glitch filter, agent IP/port.
5. **Tools**: ESC calibration, IMU gyro/accel calibration, servo sweep, ToF recovery, reboot.

Tools that move actuators require disarmed state + confirm screen, and are refused while a
ROS client is commanding.

Buzzer cues: boot, WiFi up, agent connected, agent lost, armed, disarmed, low battery.

## 8. WiFi modes

The car **joins a configured WiFi network (station mode) and falls back to its own access
point** when it can't **[user]**. Details **[my call]**:
- Boot: try station mode with stored credentials for `sta_timeout_s` (default 15 s). On
  failure, or with no credentials, start AP mode.
- AP mode: SSID `umbreon-XXXX` (last bytes of the MAC), WPA2 password generated once per
  device and stored in flash, Pico at `192.168.4.1` with a small DHCP server (pico-sdk
  example). The OLED Network screen shows SSID, password and a **WiFi QR code** (reuse
  `qrcodegen`) so the PC/phone can join quickly.
- While in AP mode with no agent session, retry station mode every 60 s. Never switch modes
  while an agent session is active.
- **Finding the agent**: if a fixed agent IP is configured, use it. Otherwise (and always in
  AP mode) use micro-ROS **agent discovery** (`rmw_uros_discover_agent`, UDP multicast), so
  the PC's address never has to be typed in. The Phase 0 spike checks that discovery works
  over CYW43/lwIP. If it doesn't, AP mode falls back to the first DHCP client's address.
- Mode switches and WiFi loss are stop + disarm conditions (§6.1).
- WiFi mode, SSID and agent are shown on the OLED Status and Network screens; mode
  preference (`auto` / `sta only` / `ap only`) is a setting.

## 9. Configuration and secrets

- Persistent settings in the last flash sectors: magic + version + CRC, two-slot write for
  power-loss safety. Flash writes happen only while disarmed (core 0 must be paused or run
  from RAM during erase/program — use `flash_safe_execute`).
- **WiFi SSID/password and agent IP/port**: build-time defaults from a git-ignored
  `config/secrets.cmake` (template `secrets.example.cmake` committed); overridable at runtime
  via the UART0/USB console (`wifi set <ssid> <pass>`, `agent set <ip> <port>`) and stored in
  flash. Agent IP/port also editable on the OLED. No secrets in source. **[my call]**
- Console on UART0 (GP16/17) and USB CDC: logs, `status`, `get/set` settings, `save`, `reboot`.

## 10. PC side (Docker) **[user: Jazzy LTS in Docker, all tools]**

Directory `umbreon_ROS2/umbreon_pico_ros2/docker/` (compose project):
- `agent`: `microros/micro-ros-agent:jazzy`,
  `udp4 -p 8888`.
- `ros`: `osrf/ros:jazzy-desktop-full` + `joy`, `teleop_twist_joy`, `rviz2`, `rqt`,
  `ros2bag` (mcap), `robot_state_publisher`, `foxglove_bridge` **[my call]**; X11/Wayland
  passthrough, `/dev/input` for gamepad, `./bags` volume, workspace volume.
- `fw-build`: pinned pico-sdk + ARM GCC + micro-ROS static library builder for Cortex-M33;
  produces `.uf2`.
- All services `network_mode: host`, shared `ROS_DOMAIN_ID` from `.env`.
- Host Humble must not be sourced in containers; mixing host Humble nodes with the container
  distro is unsupported.

ROS2 workspace `umbreon_pico_ros2/ros2_ws/src/`:
- `umbreon_description`: URDF with sensor frames/poses.
- `umbreon_bringup`: launch files `teleop.launch.py`, `viz.launch.py` (rviz config),
  `record.launch.py` (bag of all `/umbreon/*`), `all.launch.py`.
- `umbreon_teleop`: joy → `/umbreon/cmd_teleop`, deadman button, arm/disarm buttons via
  service.
- `umbreon_safety`: ROS-side reflexes for any reflex set to `ros`. It sits between the
  command sources and the car: it subscribes to `/umbreon/cmd_vel_in` and `/umbreon/cmd_teleop_in`,
  ranges, imu, odom and `/umbreon/reflex`, and republishes filtered commands to
  `/umbreon/cmd_vel` / `/umbreon/cmd_teleop`. Same reflex logic and thresholds as the
  Pico (read from the car's parameters), in Python so it can be changed without reflashing.
  When no reflex is set to `ros` the launch files bypass it.
- `umbreon_autonomy_template`: minimal Python node (subscribes ranges/odom, publishes
  cmd_vel) as a starting point for the owner's node.

## 11. Delivery phases

0. **Spike (throwaway)**: micro-ROS (Jazzy) static lib for RP2350, UDP transport over
   CYW43/lwIP, FreeRTOS SMP; publish a counter and echo a subscriber at 50 Hz; measure
   round-trip latency and reconnect behaviour. Also check agent discovery over
   CYW43/lwIP. **Gate**: confirm FreeRTOS vs bare-metal.
1. Firmware skeleton: build system, both cores, console, settings storage, watchdog,
   arming FSM, servo/ESC with stop logic (no ROS yet, commands via console).
2. Drivers: ToF, IMU, tach, battery, buzzer, encoder.
3. micro-ROS node: topics, service, parameters, time sync, link-loss handling.
4. OLED UI.
5. Docker + ROS2 packages.
6. HIL / E2E verification on the car.

## 12. Testing **[owner's global rules: TDD, 80% coverage, unit + integration + E2E]**

- **Unit (host, CMake + Unity or the `umbreon_zephyr/tests` runner style)**: arming FSM,
  stop-condition logic, command mapping/limits, PID, settings validation + serialization +
  CRC/slot selection, tach speed math, encoder decoding, ToF data conversion. Hardware
  access behind thin HAL interfaces so logic compiles on host. Coverage ≥ 80% on logic
  modules.
- **Integration**: firmware ↔ agent in Docker: topics present, rates, QoS, service and
  parameters work (`launch_testing`, car on a stand, wheels off the ground).
- **Who runs hardware tests [my call]**: the builder flashes and runs what it can from this
  PC (Pico 2 W over USB / debug probe). Anything that needs the car on a stand (ESC
  "no pulses = stop", latency, link-loss scenarios) is delivered as a scripted checklist
  plus helper scripts that the owner runs and reports back; phase gates wait on those
  results.
- **E2E / HIL**: scripted scenarios on the stand — teleop drive, cmd timeout stop, kill
  agent → stop + disarm, WiFi AP off → stop, reconnect stays disarmed, low-battery stop,
  bag record/playback in rviz.

## 13. Left to the builder
- Task priorities, stack sizes, micro-ROS memory limits (`colcon.meta` publisher/subscriber
  counts), exact QoS depths.
- u8g2 integration details and screen layouts within §7.
- Flash layout addresses, settings version scheme.
- ToF timing budget (keep existing defaults unless measurements say otherwise).
- File layout inside `umbreon_pico_ros2/` (suggested: `firmware/`, `docker/`, `ros2_ws/`,
  `tests/`, `docs/`).
