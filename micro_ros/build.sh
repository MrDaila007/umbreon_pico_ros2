#!/usr/bin/env bash
# Build into a candidate directory; preserve the previous library if any step fails.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE=microros/micro_ros_static_library_builder:jazzy
mkdir -p micro_ros/build_tmp
stage=$(mktemp -d "$PWD/micro_ros/build_tmp/run.XXXXXX")
cp -R micro_ros/library_generation "$stage/library_generation"

# Only the staged generator/output is visible to the builder; no network secrets.
docker run --rm -v "$stage:/project" --env MICROROS_LIBRARY_FOLDER=. "$IMAGE"
test -f "$stage/libmicroros/libmicroros.a"
python3 tools/check_microros.py "$stage/libmicroros" --record

# Retain the old artifact as a rollback until the candidate has replaced it.
if [ -d micro_ros/libmicroros ]; then
    mv micro_ros/libmicroros "$stage/previous-libmicroros"
fi
if ! mv "$stage/libmicroros" micro_ros/libmicroros; then
    if [ -d "$stage/previous-libmicroros" ]; then
        mv "$stage/previous-libmicroros" micro_ros/libmicroros
    fi
    exit 1
fi
cp "$stage/available_ros2_types" micro_ros/available_ros2_types
cp "$stage/built_packages" micro_ros/built_packages
printf 'micro-ROS library verified and installed; build/rollback directory: %s\n' "$stage"
