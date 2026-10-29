# Plan 3 Task 2 — safety and shared-state integration

Date: 2026-10-29. Task 1 baseline `20139f7`.

## Implemented

- Priority disarm has a sticky mailbox, clears queued arm requests/commands and
  cannot be lost when the arm queue is full. It is processed before new arm keys.
- Falling WiFi/agent levels latch a loss event and increment the session generation.
  Down/up between control ticks is preserved. Control consumes loss regardless of
  last command source; ordinary offline console operation remains available.
- ROS command/request producers supply the ready session generation. A stale
  generation is rejected. Command epochs invalidate the control task's cached
  command after loss/disarm/maintenance, preventing old throttle restoration.
- Arm requests have IDs, 60ms deadlines and completion records. Pending/inflight
  requests can be cancelled. Before PWM, core1 atomically commits the candidate
  against cancellation/deadline/generation; a failed candidate is discarded and
  recomputed from the previous control state, disarmed. Once commit wins, callers
  read the completed ACK instead of falsely claiming a timeout/cancellation.
  ACK is the actual control decision, not an externally measured output pulse.
- Persistent link fault disarms on every control cycle, so new arm keys cannot
  sneak in behind an already-set stop bit. Maintenance inhibits all motion,
  including NONE mode; NONE's logically armed state is reported truthfully even
  before the first control iteration. The neutral-command latch remains required.
- Settings updates use revisions/CAS and disarmed/no-pending-or-inflight-arm gates.
  The console `mode` exception can change only arming mode and invalidates commands;
  it cannot smuggle unrelated settings through the armed-state restriction.
- Maintenance reservations block arm/commands/settings, wait for core1 neutral ACK
  after PWM and watchdog feed, and release on timeout/flash error. Console save
  uses the extracted coordinator; fake flash tests touch no real storage. Only
  control calls watchdog_update; flash no longer feeds it independently.
- Console no longer feeds core0 heartbeat. The explicitly staged `net_task`
  heartbeat-only body is the sole writer; Task5 must replace this same task body
  with the real communication loop, not create a second worker behind a proxy beat.

## Checks and limits

Host tests first reproduced persistent-fault rearming, missed source-independent
loss, maintenance failing in NONE mode and dropped disarm behind a full queue.
Tests now cover those cases plus sticky events, stale session generations,
request cancellation/expiry including uint32 wrap, settings conflicts, reservation
ownership, failure cleanup and queued/armed save refusal.

The actual control task runs in a fake-clock/PWM/watchdog integration harness:
old throttle stays absent after loss/rearm; cancellation and timeout between pop
and commit cannot arm; ongoing console command traffic cannot mask stale network
heartbeat; maintenance ACK follows neutral PWM and watchdog feed.

**Remaining acceptance item:** real erase/program/lockout timing and watchdog
margin have not been measured. SDK `flash_safe_execute` enter/exit timeout does
not bound its flash callback. Do not infer a worst-case250ms erase guarantee from
that API. Resolve the watchdog/flash budget in Task11 before accepting persistence
on hardware; never fix this by feeding watchdog from network/flash code. The
software coordination and failure paths above are tested, while this timing item
keeps the full Task2 acceptance open. No flash/save/arm/motion command has been
sent to the board during this work.

Task7 still supplies ROS service callbacks and response wording; these primitives
alone do not mean `/umbreon/arm` exists in production yet.


Final software checks: Docker firmware build passes with project -Werror;
301 individual C tests in21 executables and10 Python contract tests pass.
Core/driver coverage:1280/1288 executable lines (99.38%). App coordination is
exercised by integration tests but is outside that core/driver coverage metric.
Logs: build/plan3-task2-fw.log and build/plan3-task2-coverage.log.

The first final coordinator build caught a missing FreeRTOS.h-before-task.h
include. It was corrected; the host task stub now enforces the same ordering,
and the full Docker firmware/test/coverage checks were rerun successfully.
