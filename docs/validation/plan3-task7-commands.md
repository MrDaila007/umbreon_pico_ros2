# Plan3 Task7 command mapping checkpoint

Partial Task7: mapping and mailbox adapter verified; acknowledged ROS arm service
and binding these functions to live rcl entities remain pending. The production
network task is still staged offline. No Plan3 firmware flashed or arm requested.

`umb_ros_command` maps Twist fields consistently:

| Topic | Kind | Mode | Value |
| --- | --- | --- | --- |
| `/umbreon/cmd_vel` | SPEED | AUTO | signed linear.x m/s |
| `/umbreon/cmd_teleop` | SPEED | MANUAL | signed linear.x m/s |
| `/umbreon/cmd_raw` | RAW | MANUAL | signed linear.x throttle |

All six axes must be finite. Unused linear.y/z and angular.x/y must be zero.
Finite angular.z clamps to [-1,1]. Huge finite double values saturate to finite
float representation; existing control logic applies actual signed limits.
Rejected input leaves the output unchanged. Receive timestamps use local
monotonic time, not a sender-provided stamp.

`ros_commands_post` validates first, then uses the real shared-state atomic
session-generation/readiness/maintenance/disarm gate. Rejection never updates the
mailbox or receive timestamp. The latest accepted callback wins across the three
topics; console and ROS share the existing explicit latest-command mailbox.
Twist has no sender timestamp: keep-last depth1 cannot identify every delayed
packet or provide authentication. Session invalidation and control's disarm and
neutral latch remain necessary.

Six tests cover all mappings, signed reverse/zero, steering saturation, all-axis
NaN/Inf, unsupported axes, invalid topics/null pointers, double overflow, unchanged
mailbox timestamp on rejection, replaced session generation, sticky loss disarm,
maintenance/offline refusal and console/ROS ordering. Mailbox tests compile the
actual shared-state implementation, including control disarm commit/publication.

Docker ARM build and coverage suite pass: 356 C tests in 32 executables plus
10 Python contract tests. Core/driver coverage is 1627/1637 lines (99.39%).
ASan/UBSan and float-cast-overflow checks pass for DHCP, endpoint selection and
pure command mapping (17 tests, including DHCP deterministic mutations).
Evidence: ignored `build/plan3-task7-{fw,coverage,sanitizers}.log`.
