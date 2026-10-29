# Plan 3 Task 5 — platform hooks and owner task

Date: 2026-10-29. Baseline adapters `9b0f7c4`.

Implemented:

- Installs a FreeRTOS-backed rcutils allocator once, before any rcl objects.
  Headers preserve max_align_t alignment; ARM compile checks portBYTE_ALIGNMENT
  against that requirement. Requested bytes/blocks/peak/failures are tracked for
  the single communication owner and later shared diagnostics.
- Allocation/header overflow and calloc product overflow are rejected. Failed
  realloc preserves the old pointer/data; zero/null operations follow the
  documented allocator behavior. Tests inject RTOS failure without touching the
  live board. The target's existing malloc-failed hook remains fatal; these tests
  do not claim a live target recovered gracefully from global heap exhaustion.
- Full64-bit monotonic boot clock underlies clock_gettime, with epoch adjustment
  left to micro-ROS sync. usleep rounds >=1ms to RTOS ticks and yields after
  scheduler start; sub-ms/pre-scheduler waits use SDK busy_wait_us.
- Network task stack raised8192 words following Phase0 as the initial budget.
  This same core0 owner installs platform hooks and remains explicitly staged
  offline until Task6/9; a separate proxy task does not beat for a blocked worker.
  Platform install failure stops its heartbeat and reports an error cue.
- Verified library now linked to the firmware. CYW43/lwIP support and startup
  affinity hook were staged in Task4; no WiFi init/association is performed by
  this intermediate network-task body. Actual runtime stack/heap gates remain.

Checks:

- Five native+Docker platform tests cover install failure/once-only success,
  alignment and resized data, failed realloc preservation, overflow guards,
  zero/calloc semantics, >uint32-us clock and yielding sleep.
- Initial Docker host compile exposed GCC13's nonnull clock_gettime declaration;
  removed the unsupported NULL comparison, preserving -Werror. Native GCC11 had
  not diagnosed it. Actual ARM build and all Docker checks then passed.
- 336 C tests in27 executables +10 Python contract tests pass. Core+driver coverage
  remains1533/1543 (99.35%); allocator is HAL and tested separately.
- Current ELF text116432/data0/bss209400 bytes. This does not include a fully active
  production ROS graph; heap/stack and reconnect lifetime are measured later.

Logs: build/plan3-task5-{fw,coverage}.log. No flash, radio init, arm, drive, settings
save or live allocation fault injection occurred on the board.
