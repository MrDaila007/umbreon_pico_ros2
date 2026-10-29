# Plan 2 Task 10 validation — 2026-10-29

Resumed from Task 9 commit `e6dc74f` on `plan2-drivers`. The interrupted Task 10
implementation was present in the working tree; it has been reviewed and finished.

## Changes found during integration review

- Require successful MPU-6050 identity/configuration before accepting samples.
  Wrong identity or a failed configuration cannot become `imu_ok` through a later
  successful sample read. Initialization retries remain at least one second apart.
- Recheck calibration eligibility while collecting samples, including queued arm
  requests, arming and speed >=0.05 m/s. Abort without replacing the previous bias.
  The console rejects queued arm requests before acknowledging calibration.
- Expire shared ToF and IMU data when read, so a stalled producer cannot leave
  health true forever. A per-sensor online mask distinguishes fresh out-of-range
  readings (9999, still online) from offline slots. The snapshot structure is
  internal firmware data, not a serialized wire format.
- Add host integration coverage of the actual IMU task/driver/filter with fake
  clock/I2C and shared-state reader expiry, including timestamp wraparound.

## Checks executed

- `make fw` and `make coverage`: blocked before build by denied access to
  `/var/run/docker.sock` in this execution environment.
- Native host build: `cmake -S tests/host -B build/host-native -G Ninja`,
  `cmake --build build/host-native`,
  `ctest --test-dir build/host-native --output-on-failure`.
  Result: **19/19 executables, 280 individual tests pass**, compiled with
  `-Wall -Wextra -Werror`.
- Coverage: GCC/gcov 11 JSON for the native build, aggregating executable source
  lines in `firmware/core/` and `firmware/drivers/`: **1279/1287 = 99.38%**.
  Local details: `build/coverage-native/summary.txt`. This is direct gcov evidence;
  the Docker/gcovr command was unavailable. App/HAL coverage is outside that metric.
- Native firmware build: Pico SDK **2.2.0**, FreeRTOS Kernel **V11.2.0** from the
  prior session's local scratchpad, system ARM compiler and installed picotool.
  ELF and UF2 produced successfully with project sources built under
  `-Wall -Wextra -Werror`. Artifacts:
  `build/fw-native/umbreon_pico_ros2.{elf,uf2}`.

The native build used the SDK at `/home/user/.pico-sdk/sdk/2.2.0` and FreeRTOS at
`/tmp/claude-1000/-home-user-Documents-Robots-roborace-umbreon-ROS2/1b77250d-b2e5-41c0-9dd1-d4e1565a5914/scratchpad/up`.
These local paths are verification context, not a replacement for the repository's
Docker build. Rebuild with `make fw` in the normal environment.

## Hardware smoke status

**Not executed in this resumed session.** No `/dev/ttyACM*` or USB serial-by-id
port was visible; `lsusb` could not initialize libusb in this environment. No flash,
arm, motion, settings save or console command was sent. There is no new hardware
transcript and no claim that the current image was tested on the car.

Remaining Task 10 acceptance: flash the current image and capture `status`,
`sensors`, then `cal imu` only after confirming disarmed/stationary; wait for
calibration, capture `sensors` and `status` again. ESC must remain 1500 us. If the
board is disconnected from car sensors, record offline/uncalibrated readings.
The wider owner-run checks are in `plan2-sensor-checklist.md`.
