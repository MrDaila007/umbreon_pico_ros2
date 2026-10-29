# Plan 2 sensor and reflex bench checklist (owner-run)

Pico 2 W, Plan 2 firmware, console over USB CDC at 115200 baud. This checklist is
for the owner to execute on the car. Motion tests below have not been run by the
agent. Record date, firmware commit, wiring and results under each section.

## Preparation and boot

Keep the wheels off the ground on a stable stand, with clear access to ESC power.
Keep hands away from the wheels and servo linkage. Cut ESC power if an observation
is unexpected. `stop` requests neutral; `disarm` also clears both arming keys.
Driving commands expire after their duration and the command timeout. Closing the
USB port also cancels USB command repetition. See the Plan 1 bench checklist for
ESC setup and neutral-before-motion behavior.

- [ ] Build with `make fw`; flash `build/fw/umbreon_pico_ros2.uf2` using BOOTSEL or
  `picotool load -x -f build/fw/umbreon_pico_ros2.uf2`.
- [ ] Open the console and enter `help`. It lists `sensors` and `cal imu`.
- [ ] Enter `status`. With default settings, expect:
  ```text
  outputs: servo 1500 us, esc 1500 us, motion no, speed 0.00 m/s
  armed: no (mode both, car key no, ros key no, neutral seen no)
  ...
  encoder: ok
  ```
  `stop:` can show `no_cmd` or later `cmd_timeout`. Missing sensors may put
  `sensor_fault` in `reflex: detected ...`. Plan 2 has no WiFi task, so
  `link: wifi down, agent down` is expected. A saved servo trim can change its
  neutral pulse. ESC must remain at 1500 us. Do not proceed if already armed.
- [ ] A fitted buzzer plays BOOT (three rising notes), then CAL_DONE on successful
  boot IMU calibration. Keep the car still for the first two seconds. A moving or
  absent IMU gives ERROR instead. `encoder: FAILED (keys disabled)` is a failure.

Result:

## ToF labels, measurements and recovery

- [ ] Enter `sensors`. The format is:
  ```text
  tof mm: HR <mm> FR <mm> R <mm> L <mm> FL <mm> HL <mm>
  tof: online <n>/6, recovering <yes|no>, restarts <count>
  accel g: <x> <y> <z>
  gyro dps: <x> <y> <z>
  yaw rate: <rate> dps, heading: <heading> deg
  imu: ok <yes|no>, calibrated <yes|no>
  battery: <voltage> V (min <minimum> V)
  stack free (words): control <n> console <n> imu <n> tof <n> ui <n>
  ```
  Before the first ADC sample the battery line is `battery: no sample`.
- [ ] With all six ToFs connected, wait for `online 6/6, recovering no`. Put a hand
  about 10 cm in front of FR: its value should be about 100 mm. Cover each sensor
  in turn and record its label, catching swapped XSHUT lines. Address order is
  HR=0x30, FR=0x31, R=0x32, L=0x33, FL=0x34, HL=0x35; XSHUT is GP6,7,8,9,14,15.
  `9999` means offline, stale, or out of range. A fresh out-of-range reading still
  counts as online.
- [ ] While disarmed, unplug one ToF cable. Repeated `sensors` should show its
  distance as 9999 within about 1.5 s (possibly earlier after consecutive NACKs),
  `online 5/6`; the other five keep updating. `status` detects `sensor_fault`.
  `recovering yes` appears after a recovery attempt; `restarts` increases.
- [ ] Reconnect. A retry is eligible every 3000 ms after the previous attempt;
  allow that cooldown plus initialization and the next measurement. The sensor
  returns to online and `recovering` clears about three seconds after the last
  attempt. A dead sensor stays offline without blocking boot or the control loop.

Actual label mapping / distances / recovery time:

## IMU and calibration

- [ ] Keep the car level and still; enter `sensors`. Expect `imu: ok yes,
  calibrated yes`, raw gyro near its small hardware bias, filtered yaw rate near
  zero, and acceleration roughly (0, 0, +1) g. The printed gyro is raw; calibration
  subtracts bias from the filtered yaw rate. Record actual axis signs below.
- [ ] Lift and tilt the disarmed car: `accel g:` changes. Hold it upside down for
  over 500 ms, then enter `status`. In manual mode with default WARN policy,
  `reflex: detected ... impact ..., intervening ...` contains `impact` only in
  detected. Set the car level again.
- [ ] While disarmed and stationary enter `cal imu`. Expect
  `ok: calibrating, keep still`; after about one second, CAL_DONE and
  `imu: ok yes, calibrated yes` in `sensors`.
- [ ] Repeat `cal imu`, rotating/tapping the car during sample collection so the
  gyro-Z spread exceeds 3 dps: expect ERROR. Previous bias is retained; a rejected
  boot calibration stays `calibrated no`. The flag does not change to `no` when
  an already calibrated IMU rejects a later calibration.
- [ ] Owner-only, on the stand: after arming as below, `cal imu` replies
  `refused: armed`. Disarm afterward. If unarmed but still rolling at >=0.05 m/s,
  the reply is `refused: moving`. A queued arm request also prevents calibration;
  arming or motion during collection cancels it without installing a new bias.

Actual accel signs / raw gyro / calibration result:

## Battery

- [ ] With the voltage divider connected to GP26 and the car disarmed, enter
  `set bat_enabled 1`. Reply: `ok (not saved; run save)`. Compare `sensors` voltage
  with a multimeter after the filter settles (about 3–5 s). Expect within 0.2 V.
- [ ] If needed, read `get bat_multiplier`, then use
  `set bat_multiplier <corrected-value>` while disarmed. Corrected multiplier is
  current multiplier * multimeter voltage / reported voltage. Recheck after
  settling. The minimum is the lowest filtered reading since boot; it does not
  reset when the multiplier changes. Battery readings exist even when the reflex
  is disabled. Enable the reflex only with a real divider/voltage source fitted.
- [ ] Owner-only: if storing calibrated settings, enter `disarm`, then `save`.
  Expect `ok: saved to slot <n> (seq <n>)`. Never save while armed.

Multimeter / reported voltage / multiplier:

## Encoder and buzzer (owner-only arming)

- [ ] With default BOTH arming, hold the encoder button for at least 0.6 s and
  release. `status` shows `car key yes`, `ros key no`, `armed: no`.
- [ ] Enter `arm ros`: `armed: yes (car key yes, ros key yes; motion needs a
  neutral command first)`. Hear ARMED (two rising notes). No motion occurs without
  a fresh neutral command followed by a driving command.
- [ ] Click and release the encoder button. Allow about 0.5 s for the double-click
  window, then enter `status`: `armed: no`, both keys `no`, ESC 1500 us. Hear
  DISARMED (two falling notes). Releasing a hold must not produce a stray click.

Result:

## Collision and speed cap (owner-only motion)

Use default policies: manual reflexes WARN, autonomous reflexes PICO. Verify any
saved overrides before testing. FR and FL are the default collision sensor mask.
The battery reflex must not be triggered, and the IMU must be healthy/upright.

- [ ] Arm with an encoder hold then `arm ros`. Send `raw 0 0` to satisfy the neutral
  latch, then promptly `auto 0 0.4 3000`. Wheels turn slowly forward. During the
  three-second run, approach FR/FL with a hand closer than 150 mm. Expect brake
  then neutral, REFLEX double beep, and `status` containing `collision` in
  `reflex: ... intervening ...`. A distant hand can already produce a speed-cap
  cue; collision intervention should produce a new cue. Do not touch the wheels.
- [ ] Clear the obstacle, send fresh neutral `raw 0 0`, then
  `speed 0 0.4 3000`. Approach FR/FL again. In manual mode, `status` contains
  `collision` in detected but not intervening; wheels continue for the command's
  duration. Finish with `stop`, then `disarm`.
- [ ] Speed cap: re-arm, send `raw 0 0`, then `auto 0 1.0 3000` with a hand about
  650 mm from FR/FL. With default near=300 mm, far=1000 mm, max=1.0 m/s and
  minimum=0.2 m/s, the target cap is about 0.6 m/s. `status` shows `speed_cap`
  intervening and measured `speed` should settle toward the cap. The console
  prints measured speed, not the numeric target/cap; it does not prove an exact
  0.6 target. This step needs a working tachometer and a suitable low-speed PID.
  Finish with `stop`, then `disarm`.

Result:

## Finish

- [ ] `status`: `armed: no`, ESC 1500 us. Record any failed item and the complete
  `status`/`sensors` output before changing wiring or settings.

Date / firmware commit / overall result:
