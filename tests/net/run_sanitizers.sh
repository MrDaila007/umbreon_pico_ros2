#!/usr/bin/env bash
# DHCP codec allocates no heap; memory/UB checks do not access real network devices.
set -euo pipefail
cd "$(dirname "$0")/../.."
mkdir -p build/net-sanitizers
cc -std=c11 -Wall -Wextra -Werror -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined -fno-sanitize-recover=all \
  -I firmware/drivers -I tests/host tests/host/test_dhcp.c firmware/drivers/umb_dhcp.c \
  -o build/net-sanitizers/test-dhcp
# LeakSanitizer cannot run in some restricted runners; codec has no allocations.
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/net-sanitizers/test-dhcp
# Pure endpoint and ROS-field conversion exercise bounded snapshot counts and
# double-to-float extremes; float-cast-overflow also checks test assertions.
for module in endpoint ros_command network_identity ap_provision; do
  cc -std=c11 -Wall -Wextra -Werror -g -fno-omit-frame-pointer \
    -fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all \
    -I firmware/core -I tests/host "tests/host/test_${module}.c" "firmware/core/umb_${module}.c" \
    -lm -o "build/net-sanitizers/test-${module}"
  ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 "build/net-sanitizers/test-${module}"
done
# ROS graph tracker and session lifecycle under the fake rcl backend, plus the
# bounded message adapters on generated struct headers (no libmicroros link).
cc -std=c11 -Wall -Wextra -Werror -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined -fno-sanitize-recover=all \
  -I firmware/app -I firmware/core -I tests/net/fakes -I tests/host \
  tests/host/test_ros_session.c tests/net/fakes/fake_ros_backend.c firmware/app/ros_session.c \
  firmware/app/ros_entities.c firmware/core/umb_net_fsm.c firmware/core/umb_telemetry.c \
  -lm -o build/net-sanitizers/test-ros-session
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/net-sanitizers/test-ros-session
cc -std=c11 -Wall -Wextra -Werror -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all \
  -I firmware/app -I firmware/core -I firmware/hal -I tests/host -I tests/host/task_stubs \
  -I micro_ros/libmicroros/include \
  tests/host/test_ros_messages.c firmware/app/ros_messages.c firmware/core/umb_telemetry.c \
  firmware/core/umb_odometry.c -lm -o build/net-sanitizers/test-ros-messages
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/net-sanitizers/test-ros-messages
