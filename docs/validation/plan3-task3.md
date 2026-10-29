# Plan 3 Task 3 — network FSMs and identity policy

Date: 2026-10-29. Pure C implementation; no network/device I/O.

- WiFi policy covers AUTO/STA_ONLY/AP_ONLY, empty credentials, bounded async join,
  failure/timeout AP fallback, AP startup failure, capped backoff, mode changes
  deferred while session busy, 60s STA retry while AP has no session, and uint32
  deadline wrap. Interface actions request I/O; they do not perform it.
- Session policy separates endpoint finding, probe, partial entity creation,
  readiness, ping and teardown. Three failed20ms pings at100ms intervals clear
  ready/latch loss before teardown. Endpoint invalidation and partial create
  failure require cleanup; no new session starts over unreleased resources.
  Teardown adapter must be idempotent for bounded retry actions.
- Result flags are one-iteration pulses for the outstanding operation. FSM timing
  cannot interrupt an SDK call that blocks its owner; adapter timeouts and the
  independent core1 protections remain necessary.
- Identity helper derives SSID suffix from MAC, preserves a valid8..63 printable
  existing password, or formats16 supplied entropy bytes as32 hex characters.
  Missing/allzero/allones entropy and invalid passwords are rejected without
  changing output. Actual hardware entropy, persistence and AP activation are
  later tasks, not claimed by these pure functions.

Checks:11 FSM tests +2 identity tests; Docker firmware build passes with -Werror.
All314 C tests in23 executables plus10 Python build-contract tests pass.
Core/driver coverage1436/1446 lines (99.31%); net FSM129/131 (98%), identity27/27
(100%). Logs: build/plan3-task3-{fw,coverage}.log.

No board flash, AP switch, credential provisioning or ROS session was performed.
The production network task remains its explicit intermediate heartbeat-only body
until adapters/runtime wiring in Tasks4–6/9.
