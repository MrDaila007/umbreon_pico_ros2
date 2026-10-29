# umbreon_pico_ros2

Raspberry Pi Pico 2 W (RP2350) micro-ROS / ROS 2 Jazzy car controller firmware.

- `firmware/core/` - pure C logic, host-testable (no pico-sdk / FreeRTOS headers)
- `tests/host/` - host unit tests (`test_*.c`, one executable each)
- `docker/fw-build/` - build image (pico-sdk 2.2.0, FreeRTOS-Kernel V11.2.0, picotool 2.2.0)

## Commands (all run inside the Docker image)

- `make image` - build the image
- `make test` - build and run host tests
- `make coverage` - gcovr over `firmware/core/`, fails under 80 % lines
- `make fw` - firmware UF2 (Task 12, not yet)
- `make microros` - micro-ROS library (Task 2, not yet)

## Secrets

Copy `config/secrets.example.cmake` to `config/secrets.cmake` (git-ignored) and edit.
