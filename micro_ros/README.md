# micro-ROS Jazzy static library (RP2350 / Cortex-M33)

Build: `make microros` (runs `micro_ros/build.sh`, Docker image
`microros/micro_ros_static_library_builder:jazzy`). Output (git-ignored):
`micro_ros/libmicroros/{libmicroros.a,include/}`.

Based on `micro-ROS/micro_ros_raspberrypi_pico_sdk` branch `jazzy`, commit
`339ae3f166bd225e5325c1d81ddd61eb8044da23`, folder `microros_static_library/library_generation/`.

Changes from upstream:
- `toolchain.cmake`: Cortex-M33 flags copied from pico-sdk 2.2.0
  `pico_arm_cortex_m33_gcc.cmake` (`-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse`);
  no pico-sdk dependency (plain `arm-none-eabi-gcc`).
- `colcon.meta`: `rmw_microxrcedds` limits (1 node, 12 pub, 4 sub, 8 services, 1 client, history 4, custom transport).
- `library_generation.sh`: paths relative to `micro_ros/` mount; `control_msgs` extra package dropped (unused); output
  lists `available_ros2_types` and `built_packages` (pinned git revisions of everything built) are committed.
- `build.sh` sets `MICROROS_LIBRARY_FOLDER=.` because the image entrypoint runs
  `/project/$MICROROS_LIBRARY_FOLDER/library_generation/library_generation.sh`.

## ABI check (`arm-none-eabi-readelf -A libmicroros.a`, all 1720 objects)

    Tag_CPU_arch: v8-M.mainline
    Tag_FP_arch: FPv5/FP-D16 for ARMv8
    Tag_ABI_HardFP_use: SP only
    (no Tag_ABI_VFP_args => base AAPCS, i.e. softfp)

Comparison against a real pico-sdk `pico2_w` object is deferred to Task 3.

## Production resource gate (Plan 3)

The production archive uses custom MTU1200 and capacity1 node/16 publishers/
4 subscriptions/10 services/0 clients/history4. `make microros` builds in an
ignored candidate directory, verifies generated headers, required type-support
symbols and Cortex-M33 softfp ABI, then seals archive/header hashes in
`libmicroros/contract-sha256.json` before replacing the installed library. A failed
candidate leaves the previous artifact installed; rollback copies remain under
`build_tmp/run.*/previous-libmicroros`.

Firmware CMake runs `tools/check_microros.py` and stops with `run make microros`
if the generated contract/manifest is missing, stale or mismatched. Hash sealing
is build integrity evidence, not a substitute for runtime memory/rate checks.

`make test` also runs the Python contract rejection tests. `make wire` runs actual
ARM-generated telemetry serializers plus best-effort XRCE framing in QEMU
(`qemu-system-arm`, machine `mps2-an505`, ARM GCC/newlib with rdimon specs needed).
It accesses no hardware. The wire budget and measured packet sizes are in
`docs/ros-interface.md`. Production lifecycle/RAM acceptance is completed as the
network and ROS entities are implemented, not inferred from library rebuild alone.
