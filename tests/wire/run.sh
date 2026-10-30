#!/usr/bin/env bash
# Actual ARM serialization without flashing hardware. Needs ARM GCC + QEMU M33.
set -euo pipefail
ulimit -c 0
cd "$(dirname "$0")/../.."
mkdir -p build/wire
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp \
  -mfloat-abi=softfp -mcmse -ffunction-sections -fdata-sections -Os \
  -Wall -Wextra -Werror -I micro_ros/libmicroros/include \
  -I firmware/app -I firmware/core \
  tests/wire/boot.S tests/wire/messages.c firmware/app/ros_messages.c \
  firmware/core/umb_telemetry.c firmware/core/umb_odometry.c -nostartfiles --specs=rdimon.specs \
  -Wl,-T,tests/wire/memory.ld -Wl,--gc-sections -Wl,-Map,build/wire/messages.map \
  -Wl,--start-group micro_ros/libmicroros/libmicroros.a -lc -lm -lrdimon -lgcc -Wl,--end-group \
  -o build/wire/messages.elf
timeout 15 qemu-system-arm -M mps2-an505 -nographic -monitor none -serial none \
  -semihosting-config enable=on,target=native -kernel build/wire/messages.elf
