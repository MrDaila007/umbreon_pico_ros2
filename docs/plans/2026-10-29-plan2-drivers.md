# Umbreon Pico 2 W — Plan 2: Sensor and UI drivers

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** All car hardware except the OLED works on the Pico 2 W: 6× VL53L0X, MPU-6050, battery ADC, buzzer and rotary encoder. Their data feeds the Plan 1 safety reflexes (real `umb_sensor_in_t`) and a shared sensor snapshot that Plan 3 will publish to ROS.

**Architecture:** Same split as Plan 1.
- Chip drivers (`firmware/drivers/`) talk to a small I2C interface (`umb_i2c_t`), so they are host-tested against a fake register-map bus.
- Decision and filter logic (`firmware/core/`) stays pure C with value semantics.
- Thin pico-sdk HALs plus FreeRTOS tasks on core 1 run the drivers and publish snapshots through `shared_state`.
- The control task stays the only PWM writer for servo and ESC. The buzzer uses its own PWM slice.

**Tech Stack:** C11, pico-sdk 2.2.0, FreeRTOS SMP, host tests with `tests/host/test_runner.h` and gcovr.

**Spec:** `docs/specs/2026-10-29-pico2w-ros2-firmware-design.md` (§3 pinout, §4.3 reuse, §5 outputs, §6.3 reflexes, §7 buzzer cues)
**Builds on:** Plan 1, branch `plan1-foundation`. Work on a new branch `plan2-drivers` cut from `plan1-foundation`.
**Reference sources (read-only):** `../umbreon_zephyr/src/{sensors,imu,battery,buzzer,encoder}.c` and `../umbreon_zephyr/modules/vl53l0x_enhanced/`.

## Global Constraints

- Plan 1 Global Constraints still apply:
  - `firmware/core/` stays pure C11 and builds on the host with `-Wall -Wextra -Werror`;
  - value semantics;
  - ≥ 80 % line coverage;
  - commit message format;
  - no secrets in git;
  - only the control task writes servo/ESC PWM.
- `firmware/drivers/*.c` must also build on the host (no pico-sdk/FreeRTOS includes). Add them to the host test build, and extend the coverage filter to `firmware/(core|drivers)/`.
- Pins (spec §3):
  - I2C0 GP0/GP1 at 400 kHz: MPU-6050 `0x68` (the SSD1306 `0x3C` is added in Plan 4).
  - I2C1 GP2/GP3 at 100 kHz: VL53L0X ×6.
  - XSHUT GP6, 7, 8, 9, 14, 15 for sensor index 0–5 (HR, FR, R, L, FL, HL).
  - Battery ADC0 on GP26. Buzzer GP18. Encoder CLK GP22, DT GP12, button GP19 (all active low, pull-ups on).
- ToF addresses: sensor *i* is moved from `0x29` to `0x30 + i`. Distances are reported in mm. The invalid sentinel is **9999** (matches `umb_reflex`).
- I2C0 will be shared with the OLED in Plan 4. Every I2C bus access goes through one FreeRTOS mutex per bus.
- No new decision logic in `firmware/app/` or `firmware/hal/`.
- Hardware rule for agents: the board may be wired to the ESC. On hardware, agents may only flash, boot, and run `status`, `sensors` and `help`, plus `cal imu` while disarmed. They never arm, never send motion, never `mode none`, never `save`.
- After code changes, run `graphify update .` if graphify is installed (skip silently if not).

## Review Focus

1. **A ToF sensor missing or dead at boot** (unplugged cable, 5 of 6 answer): the other five still work, `online == 5`, and `sensor_fault` reports it. Boot is not blocked. → Task 5 `test_tof_array_one_missing_rest_online`.
2. **An I2C read that hangs or fails mid-run** (cable glitch): the reading goes stale and turns into 9999 after 1500 ms, a restart is attempted at most every 3000 ms, and the control loop never blocks. → Task 5 `test_tof_array_stale_then_restart_cooldown`.
3. **IMU returns garbage** (NaN, all zeros, all `0xFFFF`, wrong WHO_AM_I): `imu_ok = false` and impact detection is skipped, never a false impact disarm. → Task 2 `test_mpu6050_bad_whoami_not_ok`, Task 3 `test_imu_all_ones_frame_is_fault`.
4. **IMU calibration while the car is moving** (`cal imu` typed while driving): refused. Bias stays unchanged. → Task 10 console gating, plus Task 3 `test_imu_bias_rejects_motion` (sample spread above threshold → calibration rejected).
5. **A fast encoder spin or switch bounce** (glitchy transitions): no phantom click and no phantom arm. Invalid quadrature transitions are dropped. → Task 7 `test_encoder_bounce_no_click`, `test_encoder_invalid_transition_ignored`.

---

## File map

```
firmware/drivers/  umb_i2c.h  vl53l0x.{h,c}  mpu6050.{h,c}  LICENSE-vl53l0x.txt
firmware/core/     umb_imu_filter.{h,c}  umb_tof_array.{h,c}  umb_battery.{h,c}
                   umb_encoder.{h,c}  umb_buzzer.{h,c}
                   umb_reflex.c (modified, Task 1)
firmware/hal/      hal_i2c.{h,c}  hal_gpio_misc.{h,c} (XSHUT)  hal_adc.{h,c}  hal_buzzer.{h,c}  hal_encoder.{h,c}
firmware/app/      sensors_task.{h,c}  ui_task.{h,c}  (modified) shared_state.{h,c}, control_task.c, console_cmds.c, main.c
tests/host/        fake_i2c.{h,c}  test_vl53l0x.c  test_mpu6050.c  test_imu_filter.c  test_tof_array.c
                   test_battery.c  test_encoder.c  test_buzzer.c
docs/hil/          plan2-sensor-checklist.md
```

---

### Task 1: Branch, host build for drivers, IMU NaN fix in reflexes

**Files:**
- Modify: `tests/host/CMakeLists.txt` (also compile `firmware/drivers/*.c`, and `tests/host/fake_*.c` into each test), `Makefile` (coverage filter `firmware/(core|drivers)/.*\.c`)
- Modify: `firmware/core/umb_reflex.c`
- Test: `tests/host/test_reflex.c`

**Interfaces:**
- Produces: the host build picks up `firmware/drivers/*.c` and `tests/host/fake_*.c` automatically, and coverage covers drivers.

- [ ] **Step 1:** Branch `plan2-drivers` already exists (cut from `plan1-foundation`, holds this plan) — work on it.
- [ ] **Step 2: Write the failing test** `test_reflex_nan_accel_is_imu_fault`: AUTO, `imu_present = imu_ok = true`, `accel_g = {NAN, 0, 1}` → `UMB_REFLEX_SENSOR_FAULT` is in `detected_mask`, impact is not detected, `!stop`, `!disarm`. Same for `INFINITY`.
- [ ] **Step 3:** Run `make test`. Expected: FAIL.
- [ ] **Step 4:** In `umb_reflex_step`, a non-finite accel component counts as an IMU fault: sensor_fault is detected, and impact/upside checks are skipped for that step.
- [ ] **Step 5:** Run `make test && make coverage`. Expected: PASS, ≥ 80 %. With no driver files yet, the coverage filter change is a no-op.
- [ ] **Step 6:** Commit `fix: non-finite accel counts as IMU fault in reflexes` (and the build change as `build: host build and coverage include firmware/drivers`).

### Task 2: I2C interface, fake bus, MPU-6050 driver

**Files:**
- Create: `firmware/drivers/umb_i2c.h`, `firmware/drivers/mpu6050.{h,c}`, `tests/host/fake_i2c.{h,c}`
- Test: `tests/host/test_mpu6050.c`

**Interfaces:**
- Produces:
  ```c
  /* umb_i2c.h — 7-bit addresses; returns 0 on success, negative on error/NACK/timeout */
  typedef struct umb_i2c {
      int (*write)(void *ctx, uint8_t addr, const uint8_t *buf, size_t len, bool nostop);
      int (*read)(void *ctx, uint8_t addr, uint8_t *buf, size_t len);
      void (*delay_us)(void *ctx, uint32_t us);
      void *ctx;
  } umb_i2c_t;
  int umb_i2c_write_reg8(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val);
  int umb_i2c_read_regs(const umb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t *out, size_t n);
  /* fake_i2c.h — 256-byte register file per address, auto-increment on multi-byte reads/writes,
     optional NACK per address, write log for assertions */
  typedef struct { uint8_t regs[128][256]; bool present[128]; bool nack_all; int writes; /* … */ } fake_i2c_t;
  umb_i2c_t fake_i2c_bus(fake_i2c_t *f);
  /* mpu6050.h */
  typedef struct { float accel_g[3]; float gyro_dps[3]; float temp_c; } mpu6050_sample_t;
  int  mpu6050_init(const umb_i2c_t *b, uint8_t addr);            /* 0 ok, <0 error */
  int  mpu6050_read(const umb_i2c_t *b, uint8_t addr, mpu6050_sample_t *out);
  ```
  Register values (decided):
  - WHO_AM_I `0x75` must read one of `0x68, 0x70, 0x72, 0x98` (common clones).
  - Init writes: `PWR_MGMT_1 0x6B ← 0x01` (PLL on gyro X), `SMPLRT_DIV 0x19 ← 9` (100 Hz), `CONFIG 0x1A ← 0x03` (DLPF ~42 Hz), `GYRO_CONFIG 0x1B ← 0x08` (±500 °/s, 65.5 LSB per °/s), `ACCEL_CONFIG 0x1C ← 0x10` (±8 g, 4096 LSB per g).
  - Read 14 bytes from `0x3B`, big-endian: ax, ay, az, temp, gx, gy, gz. Temperature is `raw/340 + 36.53`.
  - A frame that is entirely `0x00` or entirely `0xFF` is returned as error `-2`.

- [ ] **Step 1: Write failing tests:**
  - `test_mpu6050_init_writes_config`: after init, fake regs `0x6B == 0x01`, `0x19 == 9`, `0x1A == 3`, `0x1B == 0x08`, `0x1C == 0x10`.
  - `test_mpu6050_bad_whoami_not_ok`: WHO_AM_I `0x00` → init returns < 0.
  - `test_mpu6050_absent_nack`: no device → init < 0.
  - `test_mpu6050_scaling`: az raw 4096 → 1.0 g; gz raw −131 → −2.0 °/s (eps 0.01); temp raw 0 → 36.53.
  - `test_mpu6050_all_ff_frame_error` and `test_mpu6050_all_zero_frame_error`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement `umb_i2c` helpers, the fake bus and the driver.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: I2C interface, fake bus and MPU-6050 driver`.

### Task 3: IMU filter (bias calibration, yaw rate, heading, health)

**Files:**
- Create: `firmware/core/umb_imu_filter.{h,c}`
- Test: `tests/host/test_imu_filter.c`

**Interfaces:**
- Consumes: plain floats only. Core must not include drivers, so define a local struct with the same fields as `mpu6050_sample_t`.
- Produces:
  ```c
  typedef struct { float accel_g[3]; float gyro_dps[3]; } umb_imu_raw_t;
  typedef struct { float gyro_bias_dps[3]; float yaw_rate_dps; float heading_deg;
                   uint32_t last_ok_ms; bool have_sample; bool calibrated; } umb_imu_state_t;
  typedef struct { int n; float sum[3]; float min_z, max_z; } umb_imu_cal_t;
  #define UMB_IMU_EMA 0.3f
  #define UMB_IMU_DEADZONE_DPS 0.4f
  #define UMB_IMU_CAL_SAMPLES 200
  #define UMB_IMU_CAL_MAX_SPREAD_DPS 3.0f   /* gyro Z spread above this = car moved → reject */
  #define UMB_IMU_STALE_MS 100
  umb_imu_state_t umb_imu_init(void);
  umb_imu_cal_t   umb_imu_cal_add(umb_imu_cal_t c, const umb_imu_raw_t *s);
  bool            umb_imu_cal_finish(const umb_imu_cal_t *c, umb_imu_state_t *st); /* false if n<200 or spread too big */
  umb_imu_state_t umb_imu_update(umb_imu_state_t st, const umb_imu_raw_t *s, bool read_ok,
                                 uint32_t now_ms, bool imu_rotate);
  bool            umb_imu_is_ok(const umb_imu_state_t *st, uint32_t now_ms);  /* sample within 100 ms */
  ```
  Ported from `imu.c`:
  - Yaw rate = gyro Z minus bias, negated when `imu_rotate`, zeroed below 0.4 °/s, then EMA 0.3.
  - Heading integrates yaw rate when 0 < dt < 0.5 s, wrapped with `fmodf(…, 360)`.
  - A non-finite raw value counts as a failed read.

- [ ] **Step 1: Write failing tests:**
  - `test_imu_cal_bias`: 200 samples at gz = 1.5 → bias 1.5, `calibrated`.
  - `test_imu_bias_rejects_motion`: samples alternating 0 and 10 °/s → finish returns false, bias unchanged.
  - `test_imu_cal_too_few_samples`.
  - `test_imu_deadzone`: gz − bias = 0.3 → yaw rate 0.
  - `test_imu_ema_and_rotate`: rotate negates the sign.
  - `test_imu_heading_integrates`: 10 °/s for 1 s in 10 ms steps → ≈ 10° (eps 1.0).
  - `test_imu_dt_gap_not_integrated`: a 600 ms gap does not add heading.
  - `test_imu_stale_not_ok`: last ok 150 ms ago → `!ok`.
  - `test_imu_all_ones_frame_is_fault`: `read_ok = false` → `last_ok_ms` unchanged.
  - `test_imu_nan_sample_is_failed_read`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: IMU bias calibration, yaw-rate filter and health`.

### Task 4: VL53L0X driver (register-level)

**Files:**
- Create: `firmware/drivers/vl53l0x.{h,c}`, `firmware/drivers/LICENSE-vl53l0x.txt`
- Test: `tests/host/test_vl53l0x.c`

**Interfaces:**
- Consumes: `umb_i2c_t`, `fake_i2c_t`.
- Produces:
  ```c
  #define VL53L0X_DEFAULT_ADDR 0x29
  #define VL53L0X_MAX_VALID_MM 8190      /* ≥ this = out of range */
  typedef struct { uint8_t addr; uint8_t stop_variable; uint32_t timing_budget_us; bool ok; } vl53l0x_t;
  int  vl53l0x_init(vl53l0x_t *d, const umb_i2c_t *b, uint8_t addr);            /* full init at addr */
  int  vl53l0x_set_address(vl53l0x_t *d, const umb_i2c_t *b, uint8_t new_addr); /* reg 0x8A */
  int  vl53l0x_set_timing_budget(vl53l0x_t *d, const umb_i2c_t *b, uint32_t us);
  int  vl53l0x_start_continuous(vl53l0x_t *d, const umb_i2c_t *b);              /* back-to-back */
  int  vl53l0x_poll_range(vl53l0x_t *d, const umb_i2c_t *b, uint16_t *mm, uint8_t *status);
       /* 1 = new reading, 0 = not ready, <0 = I2C error; clears the interrupt after reading */
  ```
  - Port the init and continuous-ranging sequence of Pololu's `vl53l0x-arduino` (`init`, `setSignalRateLimit`, `setMeasurementTimingBudget`, `startContinuous`, `readRangeContinuousMillimeters`) to C over `umb_i2c_t`. Copy its license text into `LICENSE-vl53l0x.txt` and credit it in the file header.
  - Profile (spec "high speed", same as umbreon_zephyr): timing budget **20000 µs**, signal rate limit **0.25 MCPS**, default VCSEL periods.
  - `IDENTIFICATION_MODEL_ID 0xC0` must read `0xEE`.
  - Ready means `RESULT_INTERRUPT_STATUS 0x13 & 0x07 != 0`. The range is 16-bit at `0x14 + 10`, the status is `(reg 0x14 >> 3) & 0x0F`, and the interrupt is cleared with `SYSTEM_INTERRUPT_CLEAR 0x0B ← 0x01`.
  - Each driver call is bounded: polling loops inside init (e.g. waiting for reference calibration) give up after 100 ms (counted via `delay_us`) and return −3.

- [ ] **Step 1: Write failing tests** (fake device preloaded with model ID `0xEE` and a register state that lets init complete; the fake flags reference-calibration completion through `0x13`):
  - `test_vl53l0x_init_ok_and_model_check`, and `test_vl53l0x_wrong_model_fails` (`0xC0 = 0x00` → < 0).
  - `test_vl53l0x_set_address_writes_8a`: reg `0x8A == 0x30 & 0x7F`; later calls use the new address.
  - `test_vl53l0x_poll_not_ready_returns_0`.
  - `test_vl53l0x_poll_reads_range_and_clears`: range regs = 523 → mm 523, returns 1, `0x0B` written to 1.
  - `test_vl53l0x_init_timeout_bounded`: the fake never signals calibration done → returns −3, with total fake delay ≤ 110 ms.
  - `test_vl53l0x_nack_returns_error`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: register-level VL53L0X driver (Pololu port)`.

### Task 5: ToF array manager (pure)

**Files:**
- Create: `firmware/core/umb_tof_array.{h,c}`
- Test: `tests/host/test_tof_array.c`

**Interfaces:**
- Produces:
  ```c
  #define UMB_TOF_COUNT 6
  #define UMB_TOF_INVALID_MM 9999
  #define UMB_TOF_STALE_MS 1500
  #define UMB_TOF_RESTART_COOLDOWN_MS 3000
  #define UMB_TOF_BASE_ADDR 0x30
  typedef enum { UMB_TOF_OFFLINE, UMB_TOF_ONLINE } umb_tof_link_t;
  typedef struct { umb_tof_link_t link; uint16_t mm; uint32_t last_ok_ms, last_restart_ms;
                   uint32_t errors, restarts; bool have_reading; } umb_tof_slot_t;
  typedef struct { umb_tof_slot_t s[UMB_TOF_COUNT]; } umb_tof_array_t;
  typedef enum { UMB_TOF_ACT_NONE, UMB_TOF_ACT_RESTART } umb_tof_action_t;
  umb_tof_array_t  umb_tof_init(void);
  uint8_t          umb_tof_addr(int i);                        /* 0x30 + i */
  umb_tof_array_t  umb_tof_on_init_result(umb_tof_array_t a, int i, bool ok, uint32_t now_ms);
  umb_tof_array_t  umb_tof_on_poll(umb_tof_array_t a, int i, int rc, uint16_t mm, uint32_t now_ms);
       /* rc: 1 new, 0 not ready, <0 error */
  umb_tof_action_t umb_tof_next_action(const umb_tof_array_t *a, int i, uint32_t now_ms);
  void             umb_tof_distances(const umb_tof_array_t *a, uint32_t now_ms, int16_t out_mm[6]);
       /* invalid/stale/offline → 9999 */
  uint8_t          umb_tof_online(const umb_tof_array_t *a, uint32_t now_ms); /* online and fresh */
  bool             umb_tof_recovering(const umb_tof_array_t *a, uint32_t now_ms); /* any restart in last 3 s */
  ```
  Rules:
  - `mm == 0` or `mm ≥ 8190` is a valid poll that reports 9999. It is out of range, not an error.
  - An error increments `errors` and leaves `last_ok_ms` unchanged.
  - A slot is stale when online and `now − last_ok_ms > 1500`.
  - A restart is requested for an offline or stale slot when `now − last_restart_ms ≥ 3000`. The caller reports the outcome through `on_init_result`, which sets `last_restart_ms = now` and increments `restarts`.
  - All times are wrap-safe.
  - Not ported from umbreon_zephyr: the "semantic out-of-range" recovery heuristic. YAGNI until seen on the car.

- [ ] **Step 1: Write failing tests:**
  - `test_tof_addr_plan`: `umb_tof_addr(0) == 0x30`, `umb_tof_addr(5) == 0x35`.
  - `test_tof_array_one_missing_rest_online`: init ok for 0–4 and fail for 5, then fresh polls → `online == 5`, `distances[5] == 9999`.
  - `test_tof_out_of_range_reports_9999`: mm 8190 → 9999, and it still counts as fresh.
  - `test_tof_array_stale_then_restart_cooldown`:
    - At 1600 ms with no ok → stale → `RESTART`.
    - After `on_init_result(fail)` at t → `NONE` until t + 3000.
  - `test_tof_recovering_window`.
  - `test_tof_error_counts`.
  - `test_tof_wrap_safe`: times near `UINT32_MAX`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: ToF array health, staleness and restart policy`.

### Task 6: Battery filter (pure)

**Files:**
- Create: `firmware/core/umb_battery.{h,c}`
- Test: `tests/host/test_battery.c`

**Interfaces:**
- Produces:
  ```c
  #define UMB_BAT_EMA 0.05f
  typedef struct { float v_filtered, v_raw, v_min; bool have; } umb_battery_t;
  umb_battery_t umb_battery_init(void);
  umb_battery_t umb_battery_add(umb_battery_t b, uint16_t adc12, float multiplier);
       /* v = adc12 * 3.3/4095 * multiplier; the first sample seeds the EMA; adc > 4095 or a non-finite multiplier → sample ignored */
  ```
- [ ] **Step 1: Write failing tests:**
  - `test_battery_first_sample_seeds`: adc 2048, ×4.85 → ≈ 8.10 V (eps 0.02).
  - `test_battery_ema`: then adc 0 → filtered drops by 5 % of the step.
  - `test_battery_min_tracks`.
  - `test_battery_rejects_out_of_range_adc`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: battery voltage filter`.

### Task 7: Encoder decoder (pure)

**Files:**
- Create: `firmware/core/umb_encoder.{h,c}`
- Test: `tests/host/test_encoder.c`

**Interfaces:**
- Produces:
  ```c
  #define UMB_ENC_EVT_CW 0x01
  #define UMB_ENC_EVT_CCW 0x02
  #define UMB_ENC_EVT_CLICK 0x04
  #define UMB_ENC_EVT_DOUBLE 0x08
  #define UMB_ENC_EVT_HOLD 0x10
  #define UMB_ENC_EVT_LONG_HOLD 0x20
  #define UMB_ENC_DEBOUNCE_MS 50
  #define UMB_ENC_CLICK_MS 500
  #define UMB_ENC_HOLD_MS 600
  #define UMB_ENC_LONG_HOLD_MS 3000
  typedef struct { uint8_t ab_prev; int8_t sub; bool btn_stable, btn_raw; uint32_t btn_change_ms,
                   press_ms, last_click_ms; bool hold_sent, long_sent, click_pending; } umb_encoder_t;
  umb_encoder_t umb_encoder_init(bool clk, bool dt, bool btn_pressed, uint32_t now_ms);
  umb_encoder_t umb_encoder_step(umb_encoder_t e, bool clk, bool dt, bool btn_pressed,
                                 uint32_t now_ms, uint8_t *events, int *rotation);
       /* called every 1 ms; events OR-ed in; rotation += detents */
  ```
  Rules (port `encoder.c`):
  - Gray-code transition table, 4 valid transitions per detent. An invalid jump (both bits changed) is ignored.
  - The button is debounced for 50 ms.
  - Release within 500 ms → CLICK, unless a second click lands within 500 ms → DOUBLE instead (the first click is held back until the window passes).
  - Pressed ≥ 600 ms → HOLD, once per press. Pressed ≥ 3000 ms → LONG_HOLD, once per press.
  - Releasing after HOLD → no CLICK.

- [ ] **Step 1: Write failing tests:**
  - `test_encoder_cw_one_detent`, and `test_encoder_ccw_one_detent`.
  - `test_encoder_invalid_transition_ignored`: 00 → 11 → rotation 0.
  - `test_encoder_bounce_no_click`: button toggling every 5 ms for 40 ms, then released → no events.
  - `test_encoder_click`.
  - `test_encoder_double`.
  - `test_encoder_hold_no_click_on_release`.
  - `test_encoder_long_hold_once`.
  - `test_encoder_wrap_safe`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: rotary encoder and button decoder`.

### Task 8: Buzzer melodies and cue selection (pure)

**Files:**
- Create: `firmware/core/umb_buzzer.{h,c}`
- Test: `tests/host/test_buzzer.c`

**Interfaces:**
- Consumes: `umb_outputs_t` (`armed`, `reflex.intervene_mask`, `reflex.detected_mask`) from `umb_control.h`.
- Produces:
  ```c
  typedef enum { UMB_CUE_NONE, UMB_CUE_BOOT, UMB_CUE_ARMED, UMB_CUE_DISARMED, UMB_CUE_CAL_DONE,
                 UMB_CUE_ERROR, UMB_CUE_REFLEX, UMB_CUE_LOW_BATTERY } umb_cue_t;
  typedef struct { uint16_t freq_hz; uint16_t ms; } umb_note_t;   /* freq 0 = rest */
  typedef struct { const umb_note_t *notes; uint8_t n, idx; uint32_t note_end_ms; bool playing; } umb_player_t;
  const umb_note_t *umb_cue_notes(umb_cue_t cue, uint8_t *n);
  umb_player_t umb_player_start(umb_cue_t cue, uint32_t now_ms);
  umb_player_t umb_player_step(umb_player_t p, uint32_t now_ms, uint16_t *freq_out); /* freq 0 = silent */
  umb_cue_t    umb_cue_from_outputs(const umb_outputs_t *prev, const umb_outputs_t *cur);
  ```
  Melodies (from `buzzer.c`):
  - BOOT: 1047/55, 1319/55, 1568/75.
  - ARMED (= RUN_START): 784/55, 1047/70.
  - DISARMED (= STOP): 1319/45, 1047/75.
  - CAL_DONE: 1047/45, 1319/45, 1568/45, 2093/90.
  - ERROR: 392/130, 0/60, 392/130.

  New cues (spec §6.3, §7):
  - REFLEX (short double beep): 2093/40, 0/40, 2093/40.
  - LOW_BATTERY: 523/150, 0/80, 523/150, 0/80, 523/150.

  Cue priority when several transitions happen in one step: LOW_BATTERY (battery bit newly in `detected_mask`) > REFLEX (any bit newly in `intervene_mask`) > DISARMED (armed true → false) > ARMED (false → true).

- [ ] **Step 1: Write failing tests:**
  - `test_cue_notes_tables`: BOOT has 3 notes, the first is 1047/55.
  - `test_player_sequence`: frequencies change at note boundaries, then the player stops and is silent.
  - `test_cue_arm_disarm_transitions`.
  - `test_cue_reflex_newly_intervening_only`: a bit already set → NONE.
  - `test_cue_priority_low_battery_over_disarm`.
- [ ] **Step 2:** Run `make test`. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run `make test`. Expected: PASS.
- [ ] **Step 5:** Commit `feat: buzzer melodies and cue selection`.

### Task 9: HALs for I2C, XSHUT, ADC, buzzer PWM, encoder sampling

**Files:**
- Create: `firmware/hal/hal_i2c.{h,c}`, `hal_gpio_misc.{h,c}`, `hal_adc.{h,c}`, `hal_buzzer.{h,c}`, `hal_encoder.{h,c}`; modify `firmware/CMakeLists.txt`

**Interfaces:**
- Consumes: `umb_i2c_t`, `umb_encoder_*`.
- Produces:
  ```c
  /* hal_i2c.h — two buses; each wraps pico i2c_*_blocking_until with a 5 ms per-transfer timeout and a FreeRTOS mutex */
  typedef enum { HAL_I2C0 = 0, HAL_I2C1 = 1 } hal_i2c_bus_t;
  void hal_i2c_init(void);                                   /* I2C0 GP0/1 400 kHz, I2C1 GP2/3 100 kHz, pull-ups */
  const umb_i2c_t *hal_i2c_bus(hal_i2c_bus_t b);             /* the umb_i2c_t takes and releases the bus mutex per call */
  /* hal_gpio_misc.h */  void hal_xshut_init(void); void hal_xshut_set(int i, bool on);  /* GP6,7,8,9,14,15 */
  /* hal_adc.h */        void hal_adc_init(void); uint16_t hal_adc_battery_raw(void);    /* ADC0 GP26 */
  /* hal_buzzer.h */     void hal_buzzer_init(void); void hal_buzzer_tone(uint16_t freq_hz); /* GP18 PWM, 50 % duty, 0 = off */
  /* hal_encoder.h */    void hal_encoder_init(void);  /* 1 ms repeating timer on core 1 runs umb_encoder_step */
                         uint8_t hal_encoder_take(int *rotation);  /* atomically take accumulated events/detents */
  ```
  Implementation constraints:
  - The buzzer PWM slice (GP18 = slice 1A) must not touch slice 5 (servo/ESC).
  - The encoder timer callback only reads three GPIOs, runs the pure step, and ORs into a volatile accumulator under a critical section. No FreeRTOS calls in the callback.
  - `hal_i2c` mutexes are created in `hal_i2c_init`, before the scheduler starts.

- [ ] **Step 1:** Implement the HALs and add them to the firmware target.
- [ ] **Step 2:** Run `make fw`. Expected: clean build, 0 warnings.
- [ ] **Step 3:** Run `make test && make coverage`. Expected: green, ≥ 80 %.
- [ ] **Step 4:** Commit `feat: HALs for I2C buses, XSHUT, ADC, buzzer and encoder`.

### Task 10: Sensor and UI tasks, wiring into control, console

**Files:**
- Create: `firmware/app/sensors_task.{h,c}`, `firmware/app/ui_task.{h,c}`
- Modify: `firmware/app/shared_state.{h,c}`, `control_task.c`, `console_cmds.c`, `main.c`

**Interfaces:**
- Consumes: Tasks 2–9.
- Produces (Plan 3 publishes these):
  ```c
  /* shared_state.h additions */
  typedef struct { int16_t tof_mm[6]; uint32_t tof_stamp_ms[6]; uint8_t tof_online; bool tof_recovering;
                   uint32_t tof_restarts;
                   float accel_g[3], gyro_dps[3], yaw_rate_dps, heading_deg; bool imu_ok, imu_calibrated;
                   uint32_t imu_stamp_ms;
                   float bat_v, bat_v_min; bool bat_have; } shared_sensors_t;
  void shared_publish_sensors(const shared_sensors_t *s);    /* producers: sensor tasks (core 1) */
  void shared_get_sensors(shared_sensors_t *out);            /* readers: control task, console, Plan 3 */
  void shared_request_imu_cal(void);   bool shared_take_imu_cal(void);
  ```
  Tasks (all pinned to core 1, priority below the control task):
  - **ToF task**:
    - At boot: all XSHUT low; then for i = 0..5, raise XSHUT, `vl53l0x_init` at 0x29, `set_address(0x30+i)`, timing budget 20000, `start_continuous`. Report the result via `umb_tof_on_init_result`.
    - Every 10 ms: poll each online sensor and apply `umb_tof_next_action`.
    - Restart of one sensor: XSHUT low for 2 ms, high, re-init at 0x29 and re-address.
    - Publish the snapshot after each scan.
  - **IMU task**:
    - Init, then boot calibration: 200 samples at 5 ms.
    - If calibration is rejected (car moving), run uncalibrated with bias 0, set `imu_calibrated = false`, and play the ERROR cue.
    - Then 100 Hz updates through `umb_imu_update` (with the `imu_rotate` setting).
    - Re-calibrate when `shared_take_imu_cal()` is set. Re-init after 10 consecutive read failures, at most once per second.
  - **Battery**: sample every 50 ms in the IMU task loop (cheap ADC read) through `umb_battery_add` with `bat_multiplier`.
  - **UI task**, every 10 ms:
    - Take encoder events. HOLD → `shared_post_arm_request(SHARED_ARM_CAR)`. CLICK → `shared_post_arm_request(SHARED_ARM_DISARM)`. Plan 4 remaps these inside menus.
    - Run the cue player. Cues come from `umb_cue_from_outputs` between consecutive `shared_get_outputs` snapshots, plus BOOT at start and CAL_DONE after a successful calibration.
  - **Control task**: `gather_inputs` fills `umb_sensor_in_t` from `shared_get_sensors`:
    - `tof_present = true` and `tof_mm` from the snapshot;
    - `imu_present = true`, `imu_ok` from the snapshot (stale → false), `accel_g`;
    - `bat_present = bat_have`, `battery_v = bat_v`.
  - **Console**:
    - `sensors` prints the 6 distances with labels HR, FR, R, L, FL, HL, then online/recovering/restarts, accel/gyro/yaw rate/heading, imu ok/calibrated, battery V and min.
    - `cal imu` requests a calibration. Refused (`refused: armed`) unless disarmed and speed < 0.05.
    - Add both to the parser and to `help`. The parser change needs a host test in `test_console_parse.c`: `test_parse_sensors_and_cal_imu`.

- [ ] **Step 1:** Write the parser test, run it to see it fail, extend `umb_console_parse`, run it to see it pass.
- [ ] **Step 2:** Implement the shared snapshot, the tasks, the control wiring and the console handlers.
- [ ] **Step 3:** Run `make fw && make test && make coverage`. Expected: all green, 0 warnings, ≥ 80 %.
- [ ] **Step 4: Hardware smoke**, within the hardware rule:
  - Flash, then run `status`, `sensors` and `cal imu` while disarmed.
  - Record the transcript. If the board is not in the car, sensors show offline; record that, it is not a failure.
  - Expected: the BOOT melody plays if a buzzer is fitted, `status` stays disarmed, and the ESC stays at 1500.
- [ ] **Step 5:** Commit `feat: sensor and UI tasks feed reflexes and shared sensor snapshot`.

### Task 11: Owner checklist for sensors, reflexes, buzzer and encoder

**Files:**
- Create: `docs/hil/plan2-sensor-checklist.md`

Each item gives the exact console input and the expected observation, using the real `sensors`/`status` wording from Task 10. Wheels off the ground. Include:

- [ ] All 6 ToF read plausibly. A hand at 10 cm in front of FR shows ≈ 100 mm. Covering each sensor in turn identifies its label (catches swapped XSHUT wiring).
- [ ] Unplug one ToF cable while running: within 1.5 s it shows 9999, `online 5`, `recovering`; reconnect → it comes back after a restart (≤ 3 s cooldown).
- [ ] IMU:
  - Car still → gyro ≈ 0, `accel_g` ≈ (0, 0, +1). Record the actual axis signs.
  - Lift and tilt → accel changes.
  - Upside down → Z ≈ −1, and `status` shows impact `detected` (manual mode, warn).
- [ ] `cal imu` while disarmed and still → CAL_DONE melody; tapping the car during calibration → rejected (ERROR melody).
- [ ] Battery:
  - `set bat_enabled 1` → `sensors` shows battery V within 0.2 V of a multimeter.
  - Adjust `bat_multiplier` if needed; `save` only after disarming.
- [ ] Encoder:
  - HOLD 0.6 s → car key set (`status` arming keys). BOTH mode with `arm ros` → armed, ARMED melody.
  - CLICK → disarmed, DISARMED melody.
- [ ] Collision reflex on the stand:
  - Wheels spinning slowly forward in auto, `auto 0 0.4 3000`.
  - A hand closer than 150 mm in front of FR/FL → brake, then neutral, REFLEX double beep, `status` shows collision intervening.
  - The same with `speed 0 0.4 3000` (manual) → only `detected`; the wheels keep turning.
- [ ] Speed cap: in auto, a hand at ≈ 650 mm → `status` speed target capped ≈ 0.6 m/s.

- [ ] **Step 1:** Write the checklist from the real console output format.
- [ ] **Step 2:** Commit `docs: plan 2 sensor and reflex bench checklist`.
