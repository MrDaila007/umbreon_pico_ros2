# Umbreon Pico 2 W — Plan 1: Foundation, micro-ROS spike, safety core

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A reproducible build and test setup, a proven micro-ROS-over-WiFi path on the Pico 2 W, and firmware that drives the servo/ESC safely from console commands, with arming, the stop-on-silence watchdog, reverse, speed PID and the on-board safety reflexes logic (wired to real sensors in Plan 2).

**Architecture:** All decision logic (settings, actuation mapping, gear/reverse, PID, arming, safety, reflexes, control step, console parsing) lives in `firmware/core/` as pure C with value-semantics state (`state' = f(state, input)`), with no pico-sdk includes. It is unit-tested on the host. Thin target code in `firmware/app/` and `firmware/hal/` runs it on FreeRTOS SMP: core 1 hosts the 50 Hz control task, core 0 hosts the console (and the network in Plan 3).

**Tech Stack:** C11, Raspberry Pi Pico SDK 2.2.0 (board `pico2_w`), FreeRTOS-Kernel SMP (RP2350_ARM_NTZ port), micro-ROS Jazzy (static library for Cortex-M33), lwIP + CYW43, CMake/Ninja, Docker, host tests with `test_runner.h` (copied from `umbreon_zephyr/tests`), gcovr coverage.

**Spec:** `/home/user/Documents/Robots/roborace/umbreon_ROS2/docs/specs/2026-10-29-pico2w-ros2-firmware-design.md`

## Roadmap (this plan is 1 of 6)

| Plan | Delivers | Spec sections |
|------|----------|---------------|
| **1 (this)** | build/test infra, micro-ROS lib + spike (gate), core safety/actuation/reflex logic, console-driven firmware, tachometer | §3, §4.1–4.2, §5 command semantics, §6.1–6.3, §9 storage/console, §11 Phase 0–1 |
| 2 | Drivers: VL53L0X ×6, MPU-6050, battery, buzzer, encoder (ported from `umbreon_zephyr`); feed reflex sensor inputs | §4.3, §6.3, §11 Phase 2 |
| 3 | micro-ROS node: WiFi STA/AP fallback, agent discovery, topics, `/umbreon/arm`, parameters, time sync, link-loss inputs | §5, §8, §11 Phase 3 |
| 4 | OLED UI (u8g2) + encoder navigation, buzzer cues, Tools screen | §7, §11 Phase 4 |
| 5 | Docker PC stack + ROS2 packages (description, bringup, teleop, `umbreon_safety`, autonomy template) | §10, §11 Phase 5 |
| 6 | HIL/E2E scenarios on the car | §12, §11 Phase 6 |

Each later plan gets written when the previous one is done, using what was learned.

## Global Constraints

- Project root: `/home/user/Documents/Robots/roborace/umbreon_ROS2/umbreon_pico_ros2/` (new git repo). Do not modify `umbreon_zephyr/`.
- Board `pico2_w` (RP2350A, Cortex-M33). Pico SDK **2.2.0**. ROS2 / micro-ROS **Jazzy**.
- `firmware/core/*.c` must compile with plain host `gcc -std=c11 -Wall -Wextra -Werror` and include no pico-sdk/FreeRTOS headers.
- Core state machines use value semantics: functions take state by value or const pointer and return the new state. No hidden globals in `core/`.
- ESC neutral **1500 µs**, ESC limits 1000–2000 µs; servo pulse = `700 + angle*1600/180` µs, angle 0–180°. Both at 50 Hz PWM (GP10 servo, GP11 ESC).
- Command timeout default **250 ms**, range **100–1000 ms**. Control loop **50 Hz** (20 ms).
- Driving mode comes from the command source: manual (`cmd_teleop`, `cmd_raw`, console `raw`/`speed`) or autonomous (`cmd_vel`, console `auto`). Reflex actions `off|warn|pico|ros` are set per reflex and per mode; only `pico` changes outputs on the car. The link protections (timeout, WiFi, agent, core 0) are not reflexes and are always on.
- Boot always disarmed with neutral outputs. Any disarm clears both keys. After boot, any loss condition or any disarm, motion needs a neutral command first, in every arming mode.
- No secrets in git: `config/secrets.cmake` is git-ignored; `config/secrets.example.cmake` is committed.
- Coverage of `firmware/core/` ≥ **80 %** lines (gcovr).
- Commit messages: `<type>: <description>` (feat, fix, refactor, docs, test, chore, perf, ci). No attribution lines.
- After code changes, run `graphify update .` in the project root if graphify is installed (skip silently if not).

## Review Focus

1. **Non-finite or out-of-range command values** (NaN/inf/±5 from ROS or console): treat as neutral, never as full throttle. → Task 10 test `test_control_nan_command_is_neutral`.
2. **Millisecond counter wrap** (`uint32_t` ms wraps after ~49.7 days, or after a host-test start near `UINT32_MAX`): timeouts must still fire on time. → Task 8 test `test_safety_timeout_across_wrap`.
3. **Joystick jitter around zero** (alternating +0.02/−0.02): must not start brake/reverse sequences. → Task 6 test `test_gear_deadband_jitter_no_brake`.
4. **Corrupt, half-written or old-version flash settings**: use the other valid slot or the defaults, and still boot disarmed. → Task 4 tests `test_pick_slot_*`.
5. **Arming mode changed while armed** (console now, ROS/OLED later): disarms immediately. → Task 7 test `test_arming_mode_change_disarms`.

(Reflexes: side ToF sensors close to a wall must not trigger collision braking. → Task 9 test `test_reflex_collision_ignores_side_sensors`.)

---

## File map

```
umbreon_pico_ros2/
  .gitignore  Makefile  README.md
  config/secrets.example.cmake            # WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT
  docker/compose.yml                      # service fw-build (Plan 5 adds agent/ros)
  docker/fw-build/Dockerfile
  micro_ros/library_generation/           # toolchain.cmake, colcon.meta, library_generation.sh
  micro_ros/build.sh                      # → micro_ros/libmicroros/{libmicroros.a,include/}
  spike/                                  # THROWAWAY Phase 0 app (deleted at end of Plan 3)
  firmware/
    CMakeLists.txt  pico_sdk_import.cmake  FreeRTOSConfig.h
    core/  umb_types.h
           umb_settings.{h,c}   umb_actuation.{h,c}  umb_gear.{h,c}  umb_pid.{h,c}
           umb_arming.{h,c}     umb_safety.{h,c}     umb_reflex.{h,c}   umb_control.{h,c}
           umb_console_parse.{h,c}  umb_crc32.{h,c}
    hal/   hal_pwm.{h,c}  hal_flash_store.{h,c}  hal_tach.{h,c}  hal_time.{h,c}
    app/   main.c  control_task.{h,c}  console_task.{h,c}  shared_state.{h,c}
  tests/host/  CMakeLists.txt  test_runner.h  test_*.c
  docs/hil/plan1-bench-checklist.md
```

---

### Task 1: Repo scaffold, Docker build image, host test harness

**Files:**
- Create: `.gitignore`, `Makefile`, `README.md`, `config/secrets.example.cmake`, `docker/compose.yml`, `docker/fw-build/Dockerfile`, `tests/host/CMakeLists.txt`, `tests/host/test_runner.h` (copy of `umbreon_zephyr/tests/test_runner.h`), `tests/host/test_smoke.c`

**Interfaces:**
- Produces: `make test` (build + run all host tests in the container), `make coverage` (gcovr over `firmware/core/`, fails under 80 %), `make fw` (firmware UF2, used from Task 12), `make microros` (Task 2). Host test CMake adds one executable per `test_*.c`, linking all `firmware/core/*.c`, compiled with `-std=c11 -Wall -Wextra -Werror --coverage`.

- [ ] **Step 1:** `git init` in `umbreon_pico_ros2/`. `.gitignore` covers `build/`, `config/secrets.cmake`, `micro_ros/libmicroros/`, `micro_ros/build_tmp/`, `*.uf2`.
- [ ] **Step 2:** Write the Dockerfile: `ubuntu:24.04`, apt `gcc-arm-none-eabi libnewlib-arm-none-eabi cmake ninja-build gcc g++ python3 python3-pip git gcovr`; clone `raspberrypi/pico-sdk` tag `2.2.0` with submodules to `/opt/pico-sdk`; clone `FreeRTOS/FreeRTOS-Kernel` tag `V11.2.0` to `/opt/FreeRTOS-Kernel`. Verify that `portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ` exists in that tag; if not, use `raspberrypi/FreeRTOS-Kernel` `main` pinned to a commit hash and record the hash in the Dockerfile. Set `ENV PICO_SDK_PATH=/opt/pico-sdk FREERTOS_KERNEL_PATH=/opt/FreeRTOS-Kernel`. Build picotool 2.2.0 into the image.
- [ ] **Step 3:** `compose.yml` service `fw-build`: build `fw-build/`, volume `..:/work`, `working_dir: /work`, `user: "${UID:-1000}:${GID:-1000}"`.
- [ ] **Step 4:** Write `test_smoke.c` with `TEST(test_harness_runs){ ASSERT_EQ(1+1, 2); }`, and the Makefile targets above (each runs `docker compose -f docker/compose.yml run --rm fw-build …`).
- [ ] **Step 5:** Run `make test`. Expected: `1/1 tests passed`, exit 0.
- [ ] **Step 6:** Commit `chore: scaffold project, docker build image and host test harness`.

### Task 2: micro-ROS Jazzy static library for Cortex-M33

**Files:**
- Create: `micro_ros/library_generation/{toolchain.cmake,colcon.meta,library_generation.sh}` (start from `micro-ROS/micro_ros_raspberrypi_pico_sdk` branch `jazzy`, folder `microros_static_library/library_generation/`), `micro_ros/build.sh`

**Interfaces:**
- Produces: `micro_ros/libmicroros/libmicroros.a` and `micro_ros/libmicroros/include/`, consumed by `spike/` and Plan 3.

- [ ] **Step 1:** In `toolchain.cmake`, copy the compile flags pico-sdk 2.2.0 uses for RP2350 ARM (source of truth: `$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake`, including `-mcpu=cortex-m33`, the `-march`, and the exact `-mfloat-abi`). The library ABI must match the firmware ABI exactly.
- [ ] **Step 2:** In `colcon.meta`, set `rmw_microxrcedds` to `-DRMW_UXRCE_TRANSPORT=custom -DRMW_UXRCE_MAX_NODES=1 -DRMW_UXRCE_MAX_PUBLISHERS=12 -DRMW_UXRCE_MAX_SUBSCRIPTIONS=4 -DRMW_UXRCE_MAX_SERVICES=8 -DRMW_UXRCE_MAX_CLIENTS=1 -DRMW_UXRCE_MAX_HISTORY=4`. Keep the upstream message packages (they include `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `diagnostic_msgs`, `std_srvs`); check each one is present in the generated `include/`.
- [ ] **Step 3:** `build.sh` runs `docker run --rm -v "$PWD/micro_ros:/project" --env MICROROS_LIBRARY_FOLDER=library_generation microros/micro_ros_static_library_builder:jazzy` and moves the output to `micro_ros/libmicroros/`. Wire it to `make microros`.
- [ ] **Step 4:** Run `make microros`, then `arm-none-eabi-readelf -A micro_ros/libmicroros/libmicroros.a | grep -m3 -E "Tag_CPU_arch|Tag_ABI_VFP_args|Tag_FP_arch"`. Expected: `Tag_CPU_arch: v8-M.mainline`, with VFP/float-ABI tags matching a pico-sdk `pico2_w` object (compare against `readelf -A` of any object from Task 3's build).
- [ ] **Step 5:** Commit `build: micro-ROS jazzy static library for RP2350 Cortex-M33`.

### Task 3: Phase 0 spike (throwaway) — micro-ROS over WiFi, the gate

Throwaway code: no tests, not reviewed for quality. Its output is a results document.

**Files:**
- Create: `spike/CMakeLists.txt`, `spike/main.c`, `spike/udp_transport.c`, `spike/FreeRTOSConfig.h`, `spike/lwipopts.h`, `docs/spike-results.md`

- [ ] **Step 1:** Build an app for `PICO_BOARD=pico2_w` on FreeRTOS SMP (`configNUMBER_OF_CORES 2`), linking `pico_cyw43_arch_lwip_sys_freertos` and `libmicroros.a`. WiFi credentials come from `config/secrets.cmake` as compile definitions. The micro-ROS custom transport (`rmw_uros_set_custom_transport`, framing `false`) uses lwIP sockets for UDP to `AGENT_IP:AGENT_PORT`. Allocators use `pvPortMalloc`/`vPortFree`, and `clock_gettime` comes from `time_us_64()`.
- [ ] **Step 2:** The node `umbreon_spike` publishes `std_msgs/Int32` `/spike/count` at 50 Hz, subscribes `/spike/echo_in` (Int32) and republishes each value on `/spike/echo_out`. It runs on a task pinned to core 0. A core-1 task toggles a GPIO (spare GP4) at 50 Hz and logs its worst-case period jitter every 5 s over USB stdio.
- [ ] **Step 3:** Also try `rmw_uros_discover_agent()` when `AGENT_IP` is empty, and log whether it finds the agent.
- [ ] **Step 4:** Hardware run. This step needs the Pico 2 W on USB and the agent running: `docker run --rm --net=host microros/micro-ros-agent:jazzy udp4 -p 8888 -v4`. If you cannot reach the hardware, hand Steps 4–5 to the owner as a checklist and stop here. Measure and record:
  - `ros2 topic hz /spike/count` (expect ≈ 50 Hz);
  - echo round-trip latency (a host script sends 200 stamped values and reports p50/p95/max);
  - behaviour when the agent is killed and restarted (does the session recover, and how long does it take);
  - behaviour when the WiFi AP goes away and comes back;
  - core-1 jitter while all of the above happen;
  - whether discovery works.
- [ ] **Step 5:** Write `docs/spike-results.md` with the numbers and a **gate decision**: FreeRTOS SMP confirmed, or bare-metal `pico_multicore` (justify). Note any lwIP/CYW43 settings needed. If p95 echo latency is above 50 ms or the session does not recover after an agent restart, stop and report to the owner before Plan 3.
- [ ] **Step 6:** Commit `chore: phase 0 micro-ROS WiFi spike and results`.

### Task 4: Shared types, CRC32, settings model and two-slot storage format

**Files:**
- Create: `firmware/core/umb_types.h`, `firmware/core/umb_crc32.{h,c}`, `firmware/core/umb_settings.{h,c}`
- Test: `tests/host/test_settings.c`

**Interfaces:**
- Produces (in `umb_types.h`):
  ```c
  typedef enum { UMB_ARM_NONE = 0, UMB_ARM_ENCODER = 1, UMB_ARM_ROS = 2, UMB_ARM_BOTH = 3 } umb_arm_mode_t;
  typedef enum { UMB_WIFI_AUTO = 0, UMB_WIFI_STA_ONLY = 1, UMB_WIFI_AP_ONLY = 2 } umb_wifi_mode_t;
  typedef enum { UMB_SRC_CONSOLE = 0, UMB_SRC_ROS = 1 } umb_cmd_source_t;
  typedef enum { UMB_DRIVE_MANUAL = 0, UMB_DRIVE_AUTO = 1 } umb_drive_mode_t;
  typedef enum { UMB_RX_OFF = 0, UMB_RX_WARN = 1, UMB_RX_PICO = 2, UMB_RX_ROS = 3 } umb_reflex_action_t;
  typedef enum { UMB_REFLEX_BATTERY = 0, UMB_REFLEX_COLLISION, UMB_REFLEX_SPEED_CAP, UMB_REFLEX_STALL,
                 UMB_REFLEX_IMPACT, UMB_REFLEX_SENSOR_FAULT, UMB_REFLEX_COUNT } umb_reflex_id_t;
  typedef enum { UMB_CMD_NONE = 0, UMB_CMD_SPEED = 1, UMB_CMD_RAW = 2 } umb_cmd_kind_t;
  typedef struct { umb_cmd_kind_t kind; umb_cmd_source_t source; umb_drive_mode_t mode;
                   float steer;   /* -1 left … +1 right */
                   float value;   /* SPEED: m/s signed; RAW: throttle -1…+1 */
                   uint32_t rx_ms; } umb_cmd_t;
  ```
- Produces (`umb_settings.h`): `umb_settings_t` with fields and defaults:

  | field | type | default | range |
  |---|---|---|---|
  | arming_mode | uint8 (umb_arm_mode_t) | BOTH | 0–3 |
  | cmd_timeout_ms | uint16 | 250 | 100–1000 |
  | max_speed_ms / max_reverse_speed_ms | float | 1.0 / 0.5 | 0–5 |
  | max_throttle / max_throttle_rev | float | 1.0 / 0.38 | 0–1 |
  | min_speed / max_speed / min_bspeed | int16 µs | 1540 / 1600 / 1460 | 1500–2000 / 1500–2000 / 1000–1500 |
  | min_point / max_point / neutral_point | int16 deg | 60 / 120 / 90 | 0–180, min ≤ neutral ≤ max |
  | servo_reverse | bool | false | |
  | pid_kp / pid_ki / pid_kd | float | 66.4 / 243.5 / 4.16 | 0–5000 / 0–10000 / 0–2000 |
  | spd_slew | float m/s² | 0.85 | 0–20 |
  | kick_pct / kick_ms / corner_kick_us | float / int16 / int16 | 18 / 300 / 30 | 0–100 / 0–2000 / 0–200 |
  | reverse_brake_cmd / reverse_brake_ms | int16 | −250 / 650 | −1000–0 / 0–5000 |
  | encoder_holes / wheel_diam_m | int16 / float | 2 / 0.060 | 1–2000 / 0.01–1.0 |
  | tach_glitch_filter_us | uint16 | 500 | 1–500 |
  | bat_enabled / bat_multiplier / bat_low | bool / float / float | false / 4.85 / 6.0 | / 0.1–20 / 0.1–20 |
  | imu_rotate | bool | true | |
  | wifi_mode | uint8 | AUTO | 0–2 |
  | wifi_ssid[33] / wifi_pass[64] / ap_pass[64] | char | from secrets / "" / "" | |
  | agent_ip (uint32, 0 = discover) / agent_port | | from secrets / 8888 | port 1–65535 |
  | sta_timeout_s / ros_domain_id | uint16 / uint8 | 15 / 0 | 5–120 / 0–232 |
  | calibrated | bool | false | |
  | reflex_manual[UMB_REFLEX_COUNT] (uint8 actions) | | BATTERY=PICO, all others WARN | 0–3 |
  | reflex_auto[UMB_REFLEX_COUNT] | | all PICO | 0–3 |
  | ttc_ms / min_front_mm | uint16 | 400 / 150 | 100–3000 / 30–1000 |
  | cap_far_mm / cap_near_mm / cap_min_ms | uint16 / uint16 / float | 1000 / 300 / 0.2 | 100–2000 / 50–1500, near < far / 0–5 |
  | collision_sensor_mask | uint8 (bit i = ToF index i) | `(1<<1)\|(1<<4)` (FR+FL) | 1–63 |
  | stall_min_us / stall_ms | uint16 | 60 / 700 | 10–500 / 200–5000 |
  | impact_g | float | 4.0 | 1.5–16 |

  ```c
  umb_settings_t umb_settings_defaults(void);
  umb_settings_t umb_settings_sanitize(umb_settings_t s);   /* clamp to ranges, non-finite floats → default */
  bool umb_settings_set_kv(const umb_settings_t *in, const char *key, const char *value, umb_settings_t *out);
  bool umb_settings_get_kv(const umb_settings_t *s, const char *key, char *buf, size_t cap);
  #define UMB_SETTINGS_SLOT_SIZE 4096u
  size_t umb_settings_encode(const umb_settings_t *s, uint32_t seq, uint8_t *buf, size_t cap);
  bool   umb_settings_decode(const uint8_t *buf, size_t len, umb_settings_t *out, uint32_t *seq);
  int    umb_settings_pick_slot(const uint8_t *slot0, const uint8_t *slot1, size_t len,
                                umb_settings_t *out, uint32_t *seq);  /* 0/1, or -1 → defaults */
  uint32_t umb_crc32(const uint8_t *data, size_t len);  /* IEEE 802.3, reflected, init/xorout 0xFFFFFFFF */
  ```
  Blob layout: `magic u32 = 0x554D4250 ("UMBP")`, `version u16 = 1`, `payload_len u16`, `seq u32`, payload = raw `umb_settings_t` bytes, then `crc32 u32` over everything before it. `set_kv` keys are the field names; reflex actions use keys `reflex.<battery|collision|speed_cap|stall|impact|sensor_fault>.<manual|auto>` with values `off|warn|pico|ros`; it validates with `sanitize` semantics and rejects (returns false, `*out` untouched) out-of-range or unparsable values rather than clamping. `get_kv` never prints `wifi_pass`/`ap_pass` (prints `***`).

- [ ] **Step 1: Write failing tests** in `test_settings.c`:
  - `test_crc32_known_vector`: `umb_crc32("123456789", 9) == 0xCBF43926`
  - `test_defaults_match_table`: spot-check `cmd_timeout_ms == 250`, `arming_mode == UMB_ARM_BOTH`, `min_speed == 1540`, `neutral_point == 90`, `reverse_brake_ms == 650`
  - `test_sanitize_clamps_and_fixes_nan`: `cmd_timeout_ms = 5` → 100; `pid_kp = NAN` → 66.4; `neutral_point = 150` with max 120 → 120
  - `test_set_kv_rejects_out_of_range`: `set_kv(&d, "cmd_timeout_ms", "2000", &o)` is false; `"300"` is true and `o.cmd_timeout_ms == 300`, `d` unchanged
  - `test_set_kv_unknown_key_false`, `test_get_kv_masks_password`
  - `test_reflex_defaults`: `reflex_manual[UMB_REFLEX_BATTERY] == UMB_RX_PICO`, `reflex_manual[UMB_REFLEX_COLLISION] == UMB_RX_WARN`, every `reflex_auto[i] == UMB_RX_PICO`
  - `test_set_kv_reflex_action`: `"reflex.collision.manual" = "ros"` → `UMB_RX_ROS`; value `"maybe"` → false
  - `test_sanitize_cap_near_below_far`: `cap_near_mm = 1200`, `cap_far_mm = 1000` → `cap_near_mm < cap_far_mm` after sanitize
  - `test_encode_decode_roundtrip`: encode with seq 7, decode gives equal struct (memcmp) and seq 7
  - `test_decode_rejects_bad_crc`, `test_decode_rejects_bad_magic`, `test_decode_rejects_other_version`
  - `test_pick_slot_newest_valid_wins`: slot0 seq 3, slot1 seq 4 → 1
  - `test_pick_slot_falls_back_when_newest_corrupt`: flip one byte in slot1 → 0
  - `test_pick_slot_erased_flash_gives_defaults`: both all-`0xFF` → −1 and `*out` equals defaults
  - `test_pick_slot_seq_wrap`: slot0 seq `0xFFFFFFFF`, slot1 seq 0 → 1 (newer by serial-number arithmetic `(int32_t)(b-a) > 0`)
- [ ] **Step 2:** Run `make test`. Expected: link errors or FAILs for these tests.
- [ ] **Step 3:** Implement the functions above.
- [ ] **Step 4:** Run `make test`. Expected: all pass.
- [ ] **Step 5:** Commit `feat: settings model, sanitization and two-slot flash format`.

### Task 5: Actuation mapping (steer → servo µs, throttle → ESC µs)

**Files:**
- Create: `firmware/core/umb_actuation.{h,c}`
- Test: `tests/host/test_actuation.c`

**Interfaces:**
- Consumes: `umb_settings_t`.
- Produces:
  ```c
  #define UMB_ESC_NEUTRAL_US 1500
  uint16_t umb_servo_us(float steer, const umb_settings_t *c);      /* applies servo_reverse, clamps -1…1 */
  uint16_t umb_esc_us_from_throttle(float t, const umb_settings_t *c); /* open loop, applies max_throttle(_rev) */
  uint16_t umb_esc_reverse_limit_us(const umb_settings_t *c);       /* = umb_esc_us_from_throttle(-max_throttle_rev) */
  ```
  Mapping (ported from `umbreon_zephyr/src/car.c`): `s = lroundf(t*1000)` after clamping |t| to the max_throttle limit; `s>0 → map(s,1,1000,min_speed,max_speed)`; `s<0 → map(s,-1000,-1,1000,min_bspeed)`; `s==0 → 1500`. Steering: `s<0 → angle = map(s,-1000,0,min_point,neutral_point)`, else `map(s,0,1000,neutral_point,max_point)`, `us = 700 + angle*1600/180`, with integer `map` as in car.c. Non-finite input → neutral (1500 / centre).

- [ ] **Step 1: Write failing tests** with defaults:
  - `test_servo_centre`: `umb_servo_us(0) == 1500`
  - `test_servo_full_left_right`: `-1 → 1233`, `+1 → 1766`
  - `test_servo_reverse_flips`: with `servo_reverse = true`, `+1 → 1233`
  - `test_servo_clamps`: `5.0 → 1766`; `NAN → 1500`
  - `test_esc_zero_is_neutral`: `0 → 1500`
  - `test_esc_forward_range`: `0.001 → 1540`, `1.0 → 1600`
  - `test_esc_reverse_limited`: `-1.0 → umb_esc_reverse_limit_us(&c) == 1285`; `-0.001 → 1460`
  - `test_esc_nan_inf_neutral`: `NAN → 1500`, `INFINITY → 1500`
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: servo and ESC actuation mapping`.

### Task 6: Gear state machine (forward / brake / reverse)

**Files:**
- Create: `firmware/core/umb_gear.{h,c}`
- Test: `tests/host/test_gear.c`

**Interfaces:**
- Consumes: `umb_settings_t` (`reverse_brake_cmd`, `reverse_brake_ms`).
- Produces:
  ```c
  typedef enum { UMB_GEAR_NEUTRAL, UMB_GEAR_FORWARD, UMB_GEAR_WAIT_STOP, UMB_GEAR_BRAKE,
                 UMB_GEAR_GAP, UMB_GEAR_REVERSE } umb_gear_phase_t;
  typedef struct { umb_gear_phase_t phase; int8_t target_dir; uint32_t deadline_ms; } umb_gear_t;
  #define UMB_DIR_DEADBAND 0.05f        /* |request| below this = neutral request */
  #define UMB_GEAR_STOP_SPEED 0.10f     /* m/s, "stopped" */
  #define UMB_GEAR_WAIT_STOP_MS 1000u
  #define UMB_GEAR_GAP_MS 80u
  umb_gear_t umb_gear_init(void);
  umb_gear_t umb_gear_step(umb_gear_t g, float request, float speed_abs, uint32_t now_ms,
                           const umb_settings_t *c);
  int8_t umb_gear_drive_dir(umb_gear_t g);     /* +1 FORWARD, -1 REVERSE, 0 otherwise (outputs neutral/brake) */
  bool   umb_gear_braking(umb_gear_t g);       /* true in BRAKE: ESC gets reverse_brake_cmd */
  float  umb_gear_speed_sign(umb_gear_t g);    /* sign applied to tach speed for odometry: -1 only in REVERSE/GAP-from-reverse */
  umb_gear_t umb_gear_emergency_brake(umb_gear_t g, uint32_t now_ms, const umb_settings_t *c);
      /* FORWARD → BRAKE with target_dir 0: brake for reverse_brake_ms, then GAP, then NEUTRAL (never REVERSE).
         Other phases unchanged. */
  ```
  Sequence (ported from `control.c` MNV_BACK_*): a reverse request from NEUTRAL/FORWARD → WAIT_STOP (until `speed_abs < 0.10` or 1000 ms) → BRAKE (ESC `reverse_brake_cmd` for `reverse_brake_ms`) → GAP (neutral, 80 ms) → REVERSE. Forward request while in REVERSE → WAIT_STOP (neutral) → FORWARD. A request inside the deadband while driving → NEUTRAL directly (no brake). A same-direction request continues. Use wrap-safe time comparison `(int32_t)(now - deadline) >= 0`.

- [ ] **Step 1: Write failing tests:**
  - `test_gear_forward_immediate`: from init, request +0.5 → FORWARD, `drive_dir == 1`
  - `test_gear_reverse_full_sequence`: request −0.5 at speed 0.5 → WAIT_STOP; at speed 0.05 → BRAKE; after 650 ms → GAP; after 80 ms → REVERSE, `drive_dir == -1`
  - `test_gear_wait_stop_times_out`: speed stays 1.0; after 1000 ms → BRAKE
  - `test_gear_reverse_to_forward_waits_stop`: REVERSE, request +0.5, speed 0.3 → WAIT_STOP, `drive_dir == 0`; speed 0.05 → FORWARD
  - `test_gear_deadband_jitter_no_brake`: alternate +0.02/−0.02 for 50 steps → never BRAKE
  - `test_gear_cancel_reverse_during_brake`: in BRAKE, request +0.5 → no REVERSE is ever reached; ends FORWARD after wait-stop
  - `test_gear_deadline_across_wrap`: start the sequence at `now = UINT32_MAX - 100`; the brake ends at the right time
  - `test_gear_speed_sign_reverse`: in REVERSE, `umb_gear_speed_sign == -1.0f`
  - `test_gear_emergency_brake_ends_neutral`: FORWARD → `emergency_brake` → BRAKE; after 650 + 80 ms → NEUTRAL, even if the request stays +0.5 during the brake
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: gear state machine with ESC brake-reverse sequence`.

### Task 7: Arming state machine

**Files:**
- Create: `firmware/core/umb_arming.{h,c}`
- Test: `tests/host/test_arming.c`

**Interfaces:**
- Consumes: `umb_arm_mode_t`, `umb_cmd_t`.
- Produces:
  ```c
  typedef struct { umb_arm_mode_t mode; bool car_key; bool ros_key; bool neutral_seen; } umb_arming_t;
  typedef enum { UMB_ARM_OK, UMB_ARM_WAITING_CAR, UMB_ARM_WAITING_ROS, UMB_ARM_REJECTED_MODE } umb_arm_result_t;
  umb_arming_t umb_arming_init(umb_arm_mode_t mode);                 /* all false */
  umb_arming_t umb_arming_car_key(umb_arming_t a, umb_arm_result_t *res);
  umb_arming_t umb_arming_ros(umb_arming_t a, bool arm, umb_arm_result_t *res);
  umb_arming_t umb_arming_disarm(umb_arming_t a);                    /* clears both keys and neutral_seen */
  umb_arming_t umb_arming_set_mode(umb_arming_t a, umb_arm_mode_t m);/* = disarm + new mode */
  umb_arming_t umb_arming_on_command(umb_arming_t a, const umb_cmd_t *cmd); /* neutral cmd sets neutral_seen */
  bool umb_arming_is_armed(umb_arming_t a);        /* keys satisfy mode; NONE → true */
  bool umb_arming_motion_allowed(umb_arming_t a);  /* is_armed && neutral_seen */
  ```
  Neutral command: `kind != NONE` and `|value| < UMB_DIR_DEADBAND`. A car key in ROS mode → `REJECTED_MODE`. ROS `arm=true` in ENCODER mode → `REJECTED_MODE`. ROS `arm=false` always disarms (`OK`). In BOTH mode, `ros(true)` without the car key → `WAITING_CAR`; `car_key` without the ROS key → `WAITING_ROS`. A key press only sets its own key; it never sets `neutral_seen`.

- [ ] **Step 1: Write failing tests:**
  - `test_arming_boot_disarmed_all_modes`: for each mode except NONE, `!is_armed`; in every mode, `!motion_allowed`
  - `test_arming_none_needs_neutral`: NONE, command value 0.5 → `!motion_allowed`; then value 0 → `motion_allowed`
  - `test_arming_both_two_keys_any_order`: ros → WAITING_CAR, car → OK, armed. Repeat in reverse order
  - `test_arming_both_needs_neutral_after_keys`
  - `test_arming_disarm_clears_both`: after disarm, `ros(true)` alone → WAITING_CAR
  - `test_arming_ros_false_always_disarms`
  - `test_arming_encoder_mode_rejects_ros_arm`, `test_arming_ros_mode_rejects_car_key`
  - `test_arming_mode_change_disarms`: armed in BOTH, set mode BOTH again → `!is_armed`
  - `test_arming_nan_command_not_neutral`: value NAN does not set `neutral_seen`
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: configurable arming state machine with re-enable latch`.

### Task 8: Safety evaluation (stop and disarm conditions)

**Files:**
- Create: `firmware/core/umb_safety.{h,c}`
- Test: `tests/host/test_safety.c`

**Interfaces:**
- Produces:
  ```c
  enum { UMB_STOP_CMD_TIMEOUT = 1u<<0, UMB_STOP_AGENT = 1u<<1, UMB_STOP_WIFI = 1u<<2,
         UMB_STOP_REFLEX = 1u<<3 /* set by umb_control from umb_reflex */, UMB_STOP_CORE0 = 1u<<4,
         UMB_STOP_NO_CMD = 1u<<5 };
  #define UMB_STOP_DISARM_MASK (UMB_STOP_AGENT | UMB_STOP_WIFI | UMB_STOP_CORE0)
  #define UMB_CORE0_STALE_MS 500u
  typedef struct { uint32_t now_ms; bool have_cmd; uint32_t last_cmd_ms; umb_cmd_source_t source;
                   bool wifi_up; bool agent_up; uint32_t core0_beat_ms; } umb_safety_in_t;
  uint32_t umb_safety_evaluate(const umb_safety_in_t *in, const umb_settings_t *c);
  ```
  WIFI/AGENT bits apply only when `source == UMB_SRC_ROS`. Battery is a reflex (Task 9), not a link protection. Use wrap-safe elapsed time `now - then` in `uint32_t`.

- [ ] **Step 1: Write failing tests:**
  - `test_safety_no_cmd_stops`: `have_cmd=false` → `UMB_STOP_NO_CMD`
  - `test_safety_timeout_boundary`: elapsed 249 → no timeout bit; 250 → bit set
  - `test_safety_timeout_across_wrap`: `last = UINT32_MAX - 10`, `now = 300` → bit set; `now = 200` → clear
  - `test_safety_link_only_for_ros_source`: wifi down + CONSOLE source → no WIFI bit; ROS → WIFI bit
  - `test_safety_core0_stale`: beat 501 ms ago → CORE0 bit
  - `test_safety_disarm_mask_excludes_timeout`: `(UMB_STOP_CMD_TIMEOUT & UMB_STOP_DISARM_MASK) == 0`
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: safety stop and disarm condition evaluation`.

### Task 9: Safety reflexes

**Files:**
- Create: `firmware/core/umb_reflex.{h,c}`
- Test: `tests/host/test_reflex.c`

**Interfaces:**
- Consumes: `umb_settings_t` reflex fields, `umb_drive_mode_t`, `umb_reflex_id_t`, `umb_reflex_action_t`.
- Produces:
  ```c
  typedef struct { bool tof_present; int16_t tof_mm[6]; uint8_t tof_online; bool tof_recovering;
                   bool imu_present; bool imu_ok; float accel_g[3];
                   bool bat_present; float battery_v; } umb_sensor_in_t;   /* Plan 1 target: all *_present = false */
  typedef struct { uint32_t stall_since_ms; uint32_t upside_since_ms; bool stall_latched;
                   uint32_t active_mask; } umb_reflex_t;                   /* bit i = reflex i detected */
  typedef struct { uint32_t detected_mask, intervene_mask, delegated_mask; /* PICO / ROS actions */
                   bool stop, disarm, emergency_brake, block_forward;
                   float speed_cap_ms;    /* +inf = no cap */
                   float throttle_cap;    /* 1.0 = no cap */
                   float ttc_ms, nearest_mm; } umb_reflex_out_t;
  umb_reflex_t umb_reflex_init(void);
  umb_reflex_t umb_reflex_step(umb_reflex_t r, umb_drive_mode_t mode, const umb_sensor_in_t *sens,
                               float speed_signed, int esc_offset_us, uint32_t tach_age_us,
                               uint32_t now_ms, const umb_settings_t *c, umb_reflex_out_t *out);
  const char *umb_reflex_name(umb_reflex_id_t id);   /* "battery", "collision", … as in set_kv keys */
  ```
  Rules (spec §6.3). Detection always runs unless the action is OFF; `out` effects apply **only for PICO**; ROS sets `delegated_mask`, WARN sets only `detected_mask`.
  - **battery**: `bat_present && bat_enabled && !(battery_v >= bat_low)` (NaN counts as low) → stop + disarm.
  - **collision**: `nearest_mm` = minimum valid `tof_mm` over `collision_sensor_mask`. If `speed_signed > 0.05`, `ttc_ms = nearest_mm / (speed_signed*1000) * 1000`. Detected when moving forward and (`ttc_ms < ttc_ms setting` or `nearest_mm < min_front_mm`). PICO → `emergency_brake = true`, `block_forward = true`. `block_forward` also stays true while `nearest_mm < min_front_mm` with the car stopped.
  - **speed_cap**: linear from `max_speed_ms` at `cap_far_mm` to `cap_min_ms` at `cap_near_mm`, flat below near. Detected when the cap is below `max_speed_ms`. PICO → `speed_cap_ms` = cap and `throttle_cap = cap / max_speed_ms`.
  - **stall**: `|esc_offset_us| ≥ stall_min_us` and `tach_age_us ≥ stall_ms*1000` → detected, latched until the offset falls below `stall_min_us` (i.e. a neutral command). PICO → stop while latched.
  - **impact**: `imu_present && imu_ok` and (`|accel| > impact_g`, or `accel_g[2] < -0.5` held for 500 ms) → PICO stop + disarm.
  - **sensor_fault**: `tof_present && (tof_online < 6 || tof_recovering)`, or `imu_present && !imu_ok` → detected. PICO on a ToF fault → `speed_cap_ms = cap_min_ms`. An IMU fault only reports, and impact detection is skipped.
  - A sensor whose `*_present` is false never triggers its reflex.
  - All elapsed times use wrap-safe `uint32_t` subtraction.

- [ ] **Step 1: Write failing tests:**
  - `test_reflex_nothing_present_no_effect`: all `*_present = false` → `detected_mask == 0`, `!stop`, `speed_cap_ms` is inf
  - `test_reflex_collision_auto_brakes`: AUTO, FR = 300 mm, speed 1.0 m/s (ttc 300 < 400) → `emergency_brake && block_forward`, collision bit in `intervene_mask`
  - `test_reflex_collision_manual_warns_only`: same inputs, MANUAL (default WARN) → bit in `detected_mask`, `!emergency_brake`, not in `intervene_mask`
  - `test_reflex_collision_ros_delegates`: `reflex_auto[COLLISION] = ROS` → bit in `delegated_mask`, `!emergency_brake`
  - `test_reflex_collision_ignores_side_sensors`: R (index 2) = 50 mm, FR/FL = 2000 → not detected
  - `test_reflex_reverse_not_blocked`: speed −0.5 with FR = 100 → `!emergency_brake`
  - `test_reflex_speed_cap_linear`: nearest 650 mm (midpoint of 300…1000) → cap ≈ `(1.0+0.2)/2 = 0.6` (eps 0.01); 200 mm → 0.2; 1500 mm → not detected
  - `test_reflex_stall_latch`: offset 100 µs, tach age 800 ms → stop; offset stays 100 µs with fresh tach → still stopped; offset 0 → released
  - `test_reflex_impact_disarms_auto`: accel (0, 0, 5) g → stop + disarm
  - `test_reflex_upside_down_needs_500ms`: accel Z −1 g for 400 ms → nothing; at 500 ms → detected
  - `test_reflex_battery_manual_default_acts`: MANUAL, `bat_enabled`, 5.5 V < 6.0 → stop + disarm (default PICO in both modes)
  - `test_reflex_battery_nan_is_low`
  - `test_reflex_tof_fault_caps_speed`: AUTO, `tof_online = 5` → `speed_cap_ms == 0.2`
  - `test_reflex_off_not_evaluated`: COLLISION OFF → not in `detected_mask`
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: configurable safety reflexes with pico/ros/warn actions`.

### Task 10: Speed PID and control step (ties it together)

**Files:**
- Create: `firmware/core/umb_pid.{h,c}`, `firmware/core/umb_control.{h,c}`
- Test: `tests/host/test_pid.c`, `tests/host/test_control.c`

**Interfaces:**
- Consumes: Tasks 4–9.
- Produces:
  ```c
  /* umb_pid.h — port of car_pid_control(): EMA 0.7/0.3, setpoint slew, kick, corner kick (forward only),
     derivative on measurement, integral clamp ±50, feed-forward past dead zone. Works on speed magnitude;
     returns ESC offset in µs ≥ 0 from neutral. */
  typedef struct { float filtered, prev_filtered, integral, ref_applied, kick_prev_ref;
                   uint32_t prev_count, prev_ms, kick_until_ms; bool started; } umb_pid_t;
  umb_pid_t umb_pid_reset(uint32_t tach_count);
  umb_pid_t umb_pid_step(umb_pid_t p, float target_abs, int8_t dir, float steer_abs,
                         uint32_t tach_count, uint32_t tach_age_us, uint32_t now_ms,
                         const umb_settings_t *c, float *speed_abs_out, int *offset_us_out);
  /* umb_control.h */
  typedef struct { umb_arming_t arming; umb_gear_t gear; umb_pid_t pid; umb_reflex_t reflex;
                   uint32_t stop_mask; float last_speed_signed; int last_esc_offset_us; } umb_control_t;
  typedef struct { umb_cmd_t cmd;            /* valid when safety.have_cmd */
                   umb_safety_in_t safety; umb_sensor_in_t sensors;
                   uint32_t tach_count; uint32_t tach_age_us; } umb_control_in_t;
  typedef struct { uint16_t servo_us, esc_us; uint32_t stop_mask; bool armed, motion_allowed;
                   float speed_signed; umb_gear_phase_t gear; umb_drive_mode_t mode;
                   umb_reflex_out_t reflex; } umb_outputs_t;
  umb_control_t umb_control_init(const umb_settings_t *c, uint32_t tach_count);
  umb_control_t umb_control_step(umb_control_t s, const umb_control_in_t *in,
                                 const umb_settings_t *c, umb_outputs_t *out);
  ```
  `umb_control_step` order:
  1. Evaluate safety. Run `umb_reflex_step` with the command's drive mode, the previous step's `speed_signed` and ESC offset. If the reflex says `stop`, set `UMB_STOP_REFLEX`.
  2. If any `UMB_STOP_DISARM_MASK` bit is newly set, or the reflex says `disarm`, disarm.
  3. If any stop bit is set, clear `neutral_seen`: the car stops and needs a neutral command again.
  4. Feed the command to arming.
  5. If `!motion_allowed` or any stop bit: outputs are servo centre and ESC 1500, the gear resets to NEUTRAL and the PID resets.
  6. Otherwise:
     - Non-finite `steer`/`value` → treat as 0.
     - Reflex effects: `emergency_brake` → `umb_gear_emergency_brake`. `block_forward` → positive requests become 0. SPEED targets are clamped to `speed_cap_ms`, RAW throttle to `throttle_cap`.
     - SPEED: clamp to `max_speed_ms` / `max_reverse_speed_ms`; the gear steps on the sign; in FORWARD/REVERSE the PID runs on |target|; ESC = `1500 + offset` clamped to `[1500, max_speed]`, or `1500 - offset` clamped to `[umb_esc_reverse_limit_us, 1500]`; in BRAKE, ESC = `umb_esc_us_from_throttle(reverse_brake_cmd/1000)`; in other phases, 1500.
     - RAW: the gear steps on the throttle; the ESC uses `umb_esc_us_from_throttle` only in FORWARD/REVERSE, the brake value in BRAKE, else 1500.

  `speed_signed = speed_abs * umb_gear_speed_sign`. Speed is measured every step, even when stopped, so odometry stays correct.

- [ ] **Step 1: Write failing tests** (`test_pid.c`):
  - `test_pid_zero_target_zero_offset`
  - `test_pid_feedforward_past_deadzone`: target 0.5, stopped, after 2 steps 20 ms apart → `offset ≥ min_speed-1500 (40)`
  - `test_pid_speed_from_count`: 2 holes, wheel 0.060 m, +2 counts in 100 ms → raw 1.885 m/s; `speed_abs` after the EMA ≈ `0.7*1.885` (eps 0.01)
  - `test_pid_stale_tach_zeroes_speed`: `tach_age_us = 600000` → speed 0
  - `test_pid_kick_forward_only`: `dir = -1` gives no kick contribution (compare against the same inputs with `kick_pct = 0`)
- [ ] **Step 2: Write failing tests** (`test_control.c`), using a helper that builds a fresh command at `now` so there's no timeout:
  - `test_control_boot_outputs_neutral`: `esc_us == 1500 && servo_us == 1500 && !armed`
  - `test_control_raw_drive_after_arm_and_neutral`: BOTH mode, car+ros keys, neutral command, then RAW 1.0 → `esc_us == 1600`
  - `test_control_timeout_stops_without_disarm`: after the timeout → ESC 1500, `armed == true`; a non-zero command resumes nothing until a neutral command; after neutral + 0.5 → moves
  - `test_control_agent_loss_disarms`: ROS source, `agent_up=false` → `!armed`, ESC 1500
  - `test_control_nan_command_is_neutral`: armed, RAW NAN → ESC 1500
  - `test_control_raw_reverse_goes_through_brake`: armed, RAW −1 while stopped → first ESC = brake value `umb_esc_us_from_throttle(-0.25)`; after 650 + 80 ms → 1285
  - `test_control_speed_mode_clamps_target`: SPEED 9.0 m/s behaves like `max_speed_ms` (compare offsets)
  - `test_control_steer_passthrough`: armed, steer −1 → `servo_us == 1233`; when stopped, servo 1500
  - `test_control_auto_collision_brakes`: armed, AUTO mode, forward at 1.0 m/s (simulated tach), FR 300 mm → ESC equals the brake value, then 1500; a further forward command stays at 1500 while FR < `min_front_mm`
  - `test_control_manual_collision_passes`: same, MANUAL → ESC keeps the forward value, `out.reflex.detected_mask` has the collision bit
  - `test_control_battery_reflex_disarms`: `bat_present`, 5.0 V → `!armed`, ESC 1500
- [ ] **Step 3:** `make test` → FAIL.
- [ ] **Step 4:** Implement `umb_pid.c`, then `umb_control.c`.
- [ ] **Step 5:** `make test` → PASS.
- [ ] **Step 6:** Run `make coverage`. Expected: `firmware/core` line coverage ≥ 80 %. Add tests for any uncovered branch in `core/` until it is.
- [ ] **Step 7:** Commit `feat: speed PID and control step integrating arming, safety and gear`.

### Task 11: Console command parser

**Files:**
- Create: `firmware/core/umb_console_parse.{h,c}`
- Test: `tests/host/test_console_parse.c`

**Interfaces:**
- Produces:
  ```c
  typedef enum { UMB_CC_INVALID, UMB_CC_HELP, UMB_CC_STATUS, UMB_CC_RAW, UMB_CC_SPEED, UMB_CC_AUTO, UMB_CC_STOP,
                 UMB_CC_ARM_CAR, UMB_CC_ARM_ROS, UMB_CC_DISARM, UMB_CC_MODE, UMB_CC_GET, UMB_CC_SET,
                 UMB_CC_SAVE, UMB_CC_DEFAULTS, UMB_CC_REBOOT, UMB_CC_WIFI_SET, UMB_CC_AGENT_SET } umb_cc_kind_t;
  typedef struct { umb_cc_kind_t kind; float steer, value; uint16_t duration_ms;
                   umb_arm_mode_t mode; char key[32]; char arg1[65]; char arg2[65]; } umb_console_cmd_t;
  bool umb_console_parse(const char *line, umb_console_cmd_t *out);
  ```
  Grammar (lowercase, whitespace-separated): `help`, `status`, `raw <steer> <thr> [ms]`, `speed <steer> <mps> [ms]` (both MANUAL), `auto <steer> <mps> [ms]` (SPEED in AUTO mode, for bench-testing reflexes), `stop`, `arm car`, `arm ros`, `disarm`, `mode none|encoder|ros|both`, `get <key>`, `set <key> <value>`, `save`, `defaults`, `reboot`, `wifi set <ssid> <pass>`, `agent set <ip|discover> <port>`. Default `ms` is 1000, max 3000; the console task repeats the command every 20 ms for that long. Steering and throttle values outside −1…1 → INVALID. `wifi set` with an SSID over 32 or a password over 63 chars → INVALID. In Plan 1, `arm ros` stands in for the ROS service and `arm car` for the encoder long-press.

- [ ] **Step 1: Write failing tests:**
  - `test_parse_raw_default_duration` (`raw 0.2 0.3` → 1000 ms)
  - `test_parse_raw_duration_capped` (`raw 0 0.1 9000` → 3000)
  - `test_parse_raw_rejects_out_of_range` (`raw 0 1.5` → false)
  - `test_parse_mode_values`, `test_parse_set_key_value`, `test_parse_wifi_set_lengths`
  - `test_parse_auto_command` (`auto 0 0.4` → `UMB_CC_AUTO`, value 0.4, 1000 ms)
  - `test_parse_agent_discover` (`agent set discover 8888` → arg1 `"discover"`)
  - `test_parse_garbage_invalid`, `test_parse_empty_line_invalid`, `test_parse_trailing_newline_ok`
- [ ] **Step 2:** `make test` → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** `make test` → PASS.
- [ ] **Step 5:** Commit `feat: console command parser`.

### Task 12: Target firmware — HALs, FreeRTOS tasks, console-driven control

**Files:**
- Create: `firmware/CMakeLists.txt`, `firmware/pico_sdk_import.cmake`, `firmware/FreeRTOSConfig.h`, `firmware/hal/*`, `firmware/app/*`

**Interfaces:**
- Consumes: all `core/` APIs.
- Produces (used by Plans 2–4):
  ```c
  /* hal_pwm.h */   void hal_pwm_init(void); void hal_pwm_write(uint16_t servo_us, uint16_t esc_us);
  /* hal_tach.h */  void hal_tach_init(uint16_t glitch_us); uint32_t hal_tach_count(void); uint32_t hal_tach_age_us(void);
  /* hal_flash_store.h */ bool hal_settings_load(umb_settings_t *out); bool hal_settings_save(const umb_settings_t *s);
  /* hal_time.h */  uint32_t hal_now_ms(void);
  /* shared_state.h — cross-core mailbox, spinlock-protected copies, never blocks */
  void shared_init(const umb_settings_t *s);
  void shared_post_cmd(const umb_cmd_t *cmd);          bool shared_take_cmd(umb_cmd_t *out);
  void shared_post_arm_request(int kind /* 0 car, 1 ros-arm, 2 ros-disarm, 3 disarm */);
  void shared_set_link(bool wifi_up, bool agent_up);   void shared_core0_beat(uint32_t now_ms);
  void shared_set_settings(const umb_settings_t *s);   void shared_get_settings(umb_settings_t *out);
  void shared_publish_outputs(const umb_outputs_t *o); void shared_get_outputs(umb_outputs_t *out);
  bool shared_settings_write_allowed(void);            /* true only while !armed */
  ```
  - PWM: GP10/GP11 on slice 5, 50 Hz, 1 µs resolution (clkdiv so wrap = 19999). `hal_pwm_init` writes 1500/1500 before enabling the slice.
  - Tach: GP13 rising-edge IRQ with a glitch filter (port of `umbreon_zephyr/src/tachometer.c`).
  - Flash slots: the last two 4 KiB sectors of the 4 MB flash (`PICO_FLASH_SIZE_BYTES - 8192` and `- 4096`). Writes go through `flash_safe_execute` and write the slot not holding the current newest blob, with `seq+1`.
  - Control task: pinned to core 1, highest priority, `vTaskDelayUntil` every 20 ms. It reads mailbox/arm requests/settings, calls `umb_control_step`, calls `hal_pwm_write`, and feeds the hardware watchdog (`watchdog_enable(250, true)` after the first step). Only this task calls `hal_pwm_write`.
  - Console task: pinned to core 0. It reads lines from USB CDC and UART0 (GP16/17, 115200), parses them, repeats `raw`/`speed` commands every 20 ms for the duration, and beats the core-0 heartbeat every 100 ms. `status` prints the outputs, stop bits, arming keys and mode, gear, settings source (slot or defaults) and reset reason, plus a `NO ARMING` warning line when the mode is `none`. `save` is refused while armed (message `refused: armed`). Changing `mode` calls `umb_arming_set_mode` through an arm request.
  - In Plan 1 the sensor inputs are all `*_present = false`, so only the stall reflex can act (it uses ESC + tach). Plan 2 fills them in. `status` lists detected/intervening reflexes by name.
  - In Plan 1 the link is always `wifi_up=false, agent_up=false`. This is harmless because those bits only apply to the ROS source.
  - `make fw` builds `build/fw/umbreon_pico_ros2.uf2` with `PICO_BOARD=pico2_w`, FreeRTOS SMP, `configNUMBER_OF_CORES 2`, and `configUSE_CORE_AFFINITY 1`, plus whatever the Task 3 gate decided.

- [ ] **Step 1:** Implement the HALs and `shared_state.c` (a spinlock from `hardware/sync.h`; copies only, with no FreeRTOS calls inside locks).
- [ ] **Step 2:** Implement `control_task.c`, `console_task.c` and `main.c`:
  1. Load settings (or use the defaults).
  2. Initialise PWM to neutral, then the tachometer.
  3. Initialise shared state.
  4. Create the tasks with affinity.
  5. Start the scheduler.
- [ ] **Step 3:** Run `make fw`. Expected: builds with no warnings, and the UF2 exists.
- [ ] **Step 4:** Run `make test && make coverage`. Expected: still green, ≥ 80 %.
- [ ] **Step 5:** Commit `feat: console-driven firmware on FreeRTOS SMP with core-1 control loop`.

### Task 13: Bench checklist for the owner (HIL, car on a stand)

**Files:**
- Create: `docs/hil/plan1-bench-checklist.md`, `tools/console_drive.py` (pyserial: sends `raw`/`speed` lines and reads `status`; `--stop-after` to simulate the PC going silent)

Each checklist item states the exact console input and the expected observation. Include:

- [ ] Flash with `picotool load -x build/fw/umbreon_pico_ros2.uf2` (or BOOTSEL drag-and-drop).
- [ ] At boot, `status` shows `armed=0`, ESC 1500, servo 1500, and the wheels don't move.
- [ ] Mode BOTH: `raw 0 0.3` does nothing; `arm car`, `arm ros`, `raw 0 0` and then `raw 0 0.3 2000` turn the wheels for about 2 s, then they stop on their own (timeout).
- [ ] `raw 0 0.3 2000` again, without a neutral first: the wheels do **not** move (latch).
- [ ] `raw 0 -0.5 3000`: the brake → pause → reverse sequence is audible/visible and the wheels turn backwards.
- [ ] Unplug USB/UART mid-run (`console_drive.py --stop-after 1`): the car stops within 250 ms.
- [ ] **ESC with no pulses**: hold BOOTSEL and reset while the throttle is applied, then confirm the ESC stops and stays stopped. Record the ESC model and result.
- [ ] `speed 0 0.5 3000`: steady wheel speed and `status` speed ≈ 0.5 m/s; record the tach readings.
- [ ] Steering: `raw -1 0` / `raw 1 0` hit the mechanical limits without binding. Adjust `min_point`/`max_point`/`neutral_point` and `save`, then power-cycle and confirm the values persist.
- [ ] **Stall reflex**: hold a wheel still, then `auto 0 0` and `auto 0 0.5 3000` → the ESC drops to neutral after about 0.7 s, and `status` shows `stall` intervened. Repeat with `raw 0 0` and `raw 0 0.3 3000` (manual, default WARN) → the motor keeps pushing and `status` shows `stall` detected only. Release the wheel immediately.
- [ ] `mode none`, `raw 0 0`, `raw 0 0.3`: the wheels move without keys, and a `status` warning shows `NO ARMING`.

- [ ] **Step 1:** Write the checklist and the script.
- [ ] **Step 2:** Run `python3 tools/console_drive.py --help`. Expected: prints usage.
- [ ] **Step 3:** Commit `docs: plan 1 bench checklist and console drive helper`.
