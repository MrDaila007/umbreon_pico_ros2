#!/usr/bin/env bash
# Build micro-ROS (Jazzy) static library for RP2350 / Cortex-M33.
# Output: micro_ros/libmicroros/{libmicroros.a,include/}
set -euo pipefail

cd "$(dirname "$0")/.."

IMAGE=microros/micro_ros_static_library_builder:jazzy

rm -rf micro_ros/libmicroros micro_ros/build_tmp
# The image entrypoint runs /project/$MICROROS_LIBRARY_FOLDER/library_generation/library_generation.sh;
# our scripts live directly in micro_ros/library_generation/, hence the folder "." (not "library_generation").
docker run --rm -v "$PWD/micro_ros:/project" --env MICROROS_LIBRARY_FOLDER=. "$IMAGE"

test -f micro_ros/libmicroros/libmicroros.a
echo "micro-ROS library: micro_ros/libmicroros/libmicroros.a"
