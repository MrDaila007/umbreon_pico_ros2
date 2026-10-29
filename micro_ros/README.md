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
