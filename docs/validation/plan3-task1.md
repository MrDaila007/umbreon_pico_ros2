# Plan 3 Task 1 — resource/build gate

Date: 2026-10-29. Branch `plan3-network`, planning baseline `d0ba447`.

Completed:

- Generated Jazzy library custom MTU1200, node1/pub16/sub4/service10/client0/history4.
  Dependency revisions and available type list unchanged from committed baseline.
- Configure-time checks reject missing/stale headers/archive/type support or
  wrong ARM ABI. A newly verified archive/header pair has a SHA256 manifest.
- Library builds preserve the previous install until the candidate passes.
  Generator fails on errors and handles already-flat/no-C-header package layouts.
- Corrected the stream-framing metadata CMake flag typo; UDP callback framing
  remains false. This does not enable an unimplemented native discovery API.
- Coverage searches only the Docker host-build tree, avoiding incompatible native
  compiler coverage artifacts. `make test` includes10 contract regression tests.

Evidence:

- Old archive rejected by resource check (MTU512/pub12/service8/client1).
- Actual ARM serialization run under QEMU fails old512 at Odometry760 CDR bytes;
  new1200 passes all six message fixtures and captures12 bytes of XRCE overhead.
  See `docs/ros-interface.md` for bounds and packet sizes.
- 10 Python contract tests pass, including wrong ABI, missing symbol/header/archive,
  stale limits/MTU, missing manifest and replacement of a sealed archive.
- `readelf -A`: all1720 archive objects and a real Pico SDK main object agree on
  v8-M.mainline/base AAPCS softfp (no Tag_ABI_VFP_args hard-float convention).
- Docker firmware build and280 inherited C unit/integration tests pass. Coverage:
  1279/1287 core+driver lines (99.38%). The build-time contract tests do not inflate
  that C coverage metric.
- Compile-only network spike link: text433704/data0/bss304684 bytes. Current
  sensor/control firmware: text109688/data0/bss209024. These are initial static
  link budgets; final production entity heap/stack remains a later acceptance gate.

The first Docker baseline coverage attempt failed because gcovr picked up native
GCC11 objects under build/ alongside Docker GCC13 data. Restricting search to
build/host fixed the command; errors were not ignored. Initial generator flattening
warnings were corrected with guarded paths and fail-fast behavior; the final
full library rebuild completed successfully. Upstream dependency compile warnings
are present in the builder log; the project firmware builds under -Werror.

Local logs: build/plan3-microros-build-final.log, plan3-task1-fw.log,
plan3-task1-coverage.log, plan3-wire.log, plan3-sdk-abi.txt,
plan3-microros-abi.txt and plan3-link-probe-*.log.

No board flash, drive/arm/save command or network-mode change was performed for
this task. The ROS graph described here is not live in the production firmware.
