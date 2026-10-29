# Plan3 Task8 SI telemetry, time mapping and odometry

Pure C11 in `firmware/core/` (`umb_telemetry`, `umb_odometry`), no rcl/message/SDK
headers. Task9 copies these plain structs into the ROS messages. Host-tested only; no
firmware flashed, nothing run on the car.

## Range profile (chosen, not guaranteed)
- max 2.0 m: nominal ST long-range figure; practical high-speed range may be shorter.
  The driver's 8190 mm cutoff is only an invalid sentinel boundary.
- min 0.03 m: conservative CHOSEN application limit. ST gives no universal minimum
  (depends on optics/cover glass). Bench measurement per mounted sensor is still owed.
- FoV 25 deg nominal, radiation INFRARED, frames `tof_hr|fr|r|l|fl|hl`.
- mm 0 / >=8190 / 9999 / offline / stale (>1500 ms) -> +inf; below min -> -inf; above
  max -> +inf. Core safety still uses the raw distances.
- An offline/stale sample is stamped at publish time (it is a status event, not an
  acquisition) and is emitted once on the transition, not repeated.

## Other conversions
- IMU: deg/s -> rad/s, g -> m/s^2 (9.80665), frame `imu_link`, orientation
  covariance[0] = -1. Not ok or any non-finite input -> `healthy=false` (not published).
- Battery: voltage only; current/charge/capacity/percentage NaN; status/health/
  technology 0 (UNKNOWN); non-finite or negative voltage -> present=false, voltage NaN.
  The snapshot has no battery acquisition stamp, so it is stamped at publish time.

## Time
ROS stamp = monotonic ms * 1e6 + offset. Unsynced returns false (telemetry suppressed;
never a fake epoch). A failed resync keeps the last offset; `invalidate` suppresses.
Backwards epoch adjustment only changes later stamps; odometry/safety use the uint32
monotonic clock only. `umb_mono_extend` widens to uint64 (49-day wrap);
`umb_sensor_stamp64` reconstructs acquisition time (future up to 5 ms clamped to now,
further or pre-boot rejected).

## Odometry
Measured signed speed (`outputs.speed_signed`) and bias-corrected IMU yaw rate,
midpoint integration, yaw in (-pi, pi]. dt <= 0 or > 200 ms (also the first call)
re-seeds without integrating (this deliberately follows the controller decision of
200 ms rather than the plan text's 500 ms). IMU not ok or non-finite rate: position
integrates along the last good yaw, yaw unchanged, yaw covariance 1e6 and xy covariance
1.0, wz invalid. Non-finite speed: no integration. Pose resets only at `umb_odom_init`
(boot); an agent outage appears as one long dt and is skipped, retaining the pose.
Covariances are chosen conservative values, not identified.

## Cadence
IMU 100 / odom 50 / range 25 / battery 1 / diagnostics 1 Hz. `umb_sched_take` is
due && has_new; not-new leaves the stream due. After a gap >= one period it emits once
and reschedules from now. New-sample detection uses the snapshot acquisition stamps
(`tof_stamp_ms[i]`, `imu_stamp_ms`, which only advance on a successful acquisition), so
no `shared_sensors_t` change was needed.

## Verified
`make test`/`make coverage`: test_telemetry 30/30, test_odometry 16/16, full suite
passing, umb_telemetry.c and umb_odometry.c 100% line coverage, total 99%.
`make fw` builds the uf2 with -Wall -Wextra -Werror.
