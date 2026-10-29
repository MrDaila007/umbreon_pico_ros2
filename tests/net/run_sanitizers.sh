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
