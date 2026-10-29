# Plan3 Task7 command mapping and acknowledged arming

Task7 host logic is complete: Twist mapping, the callback mailbox adapter and the
`/umbreon/arm` (std_srvs/SetBool) handler. Binding these functions to live rcl
subscriptions/service and supplying the production clock (`hal_now_ms` +
`vTaskDelay`) is Task9. The production network task is still staged offline.
No Plan3 firmware flashed and no arm requested on hardware.

## Command mapping

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

## Callback adapter

`ros_commands_post` validates first, stamps `hal_now_ms()` locally, then uses the
real shared-state atomic session-generation/readiness/maintenance/disarm gate.
Rejection never updates the mailbox or receive timestamp. The latest accepted
callback wins across the three topics; console and ROS share the existing
explicit latest-command mailbox.

Binding contract for Task9 (`ros_commands.h`): best-effort, keep-last
`ROS_COMMANDS_HISTORY_DEPTH` = 1. Callbacks carry
`ROS_COMMANDS_SETUP_GENERATION` (0) until the session is declared ready; 0 never
matches shared state, so a sample cached during lifecycle setup is dropped rather
than replayed.

Twist has no sender timestamp: keep-last depth1 cannot independently identify
every delayed network packet, and this is not an authenticated or freshness
protocol. Session invalidation and control's disarm and neutral latch remain
mandatory, so there is no auto-resume after loss.

## `/umbreon/arm` handler

Pure decisions live in `firmware/core/umb_arm_reply.{h,c}`; `firmware/app/ros_services.{h,c}`
is thin glue over `shared_submit_arm` / `shared_get_arm_ack` / `shared_cancel_arm`
with an injected clock/sleep. Exact replies:

| Message | success | When |
| --- | --- | --- |
| `armed` | true | control committed, keys satisfy mode |
| `waiting for car arm` | true | ROS key accepted in `both`, car key missing |
| `waiting for ros arm` | true | mapping for control's WAITING_ROS result (not produced by a ROS request) |
| `disarmed` | true | any arm(false) |
| `refused: mode` | false | control rejected the ROS key (mode `none`/`encoder`) |
| `refused: session` | false | generation 0/stale, link not ready, or generation changed while waiting |
| `refused: maintenance` | false | maintenance reservation held |
| `busy: timeout` | false | queue full, no committed ACK within 60 ms, or cancelled by a racing disarm |

arm(true): refused before submitting (no request ID consumed) for a dead/stale
session or maintenance. Otherwise it submits a request with ID and session
generation, then polls the actual control ACK every <=5 ms, never sleeping past
60 ms after submit (the shared request deadline is the same 60 ms). Control runs
independently on core 1. On timeout, stale session or maintenance the handler
calls `shared_cancel_arm` and re-reads the ACK: if control already committed,
the committed ACK is reported (no "failed but later armed" outcome). The ACK is
read before each time sample so a commit racing the timeout is still seen. A
cancelled request cannot commit: `shared_take_arm_record` skips it and
`shared_commit_arm_batch` discards the whole candidate.

arm(false) is never refused (stale session, generation 0, maintenance) and never
cancelled. It submits the sticky priority disarm, which drops queued arm requests
and clears both keys in control; the handler waits up to 60 ms for its ACK and
reports `disarmed`. If control is stalled the disarm stays queued and applies on
the next tick (core-1 watchdog covers a wedged control loop).

## Verification

`test_arm_reply` (10 pure tests): exact messages/success flags, result mapping,
refusal precedence, poll/cancel/reply decisions at and beyond the bound, committed
ACK winning over timeout/session loss, disarm never refused or cancelled, sleep
never passing the bound.

`test_ros_services` (15 tests, real `shared_state.c`, fake clock, simulated
control tick in the same order as `control_task.c`): ros-mode armed, `both`
waiting for car / armed after car key, wrong mode refused, generation 0/stale/dead
session refused without submit or sleep, maintenance refused without submit,
full queue busy without waiting, stalled control times out at exactly 60 ms and a
later tick whose clock sample lags the timeout does not arm, commit just before
the timeout reported as `armed` via failed cancel, record taken but uncommitted at
timeout cancelled and the commit discarded, generation change while waiting
refused and cancelled, disarm with priority clears both keys and drops a queued
car key, disarm accepted for stale session/generation 0/maintenance, disarm with
stalled control stays sticky, missing clock refuses arm but still disarms.

`test_ros_commands` adds setup generation 0 never posting (4 tests total).
Mutation checks: removing the post-cancel ACK re-read, the cancel call, or the
pre-submit session/maintenance check each fails at least one test.

Docker host suite: 382 C tests in 34 executables plus 10 Python contract tests,
all passing, no warnings. Core/driver coverage 1677/1687 lines (99%);
`umb_arm_reply.c` 50/50. Docker ARM firmware build passes (`ros_services.c`
compiled into the image). Evidence: ignored
`build/plan3-task7b-{red,green,coverage,fw}.log`; earlier mapping evidence in
`build/plan3-task7-{fw,coverage,sanitizers}.log`.
