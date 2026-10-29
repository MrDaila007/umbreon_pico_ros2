# Phase 0 spike results: micro-ROS over WiFi on Pico 2 W

Date: 2026-10-29. Throwaway code is in `spike/`. This document is what the spike delivers.

## Gate decision

**FreeRTOS SMP is confirmed.** Plan 3 keeps FreeRTOS SMP (`configNUMBER_OF_CORES 2`) with core affinity.
We do not need bare-metal `pico_multicore`.

- micro-ROS Jazzy (`libmicroros.a`, custom transport) + CYW43 + lwIP sockets (`NO_SYS=0`) + FreeRTOS
  V11.2.0 SMP build, link and run on the RP2350. No asserts, faults or stack/heap hook hits came up in
  about 20 minutes of runs. Free heap stayed flat (141 280 B) through 7+ session teardown/recreate cycles.
- Core 1 kept its 50 Hz timing while all the tests below ran: worst period error **275 µs**
  (median of the 5 s windows 83 µs, p95 141 µs, 126 windows).
- The session **recovers after an agent restart** with no reboot: 0.5–0.8 s from agent start to session up.
- Echo latency: **p95 46.7 ms** over the largest run (2000 samples), which is just under the 50 ms gate.
  The tail comes from the WiFi link, not from the stack (p50 is 4–5 ms). See the concerns below.

**For the owner, before Plan 3:** the latency gate passes, but with almost no margin, and only on this
office network. 2 of the 7 separate runs had p95 above 50 ms (70.8 ms and 161 ms). There were also
WiFi downlink blackouts of 3–10 s while nothing else was happening. The likely causes are a weak signal
(RSSI −64…−75 dBm) and an office AP routed onto a different L3 subnet. The car's own AP or hotspot
should be measured again before the link timeouts are tuned (command timeout 250 ms).

## Setup

| Item | Value |
|---|---|
| Board | Pico 2 W (RP2350A), pico-sdk 2.2.0, `PICO_BOARD=pico2_w` |
| RTOS | FreeRTOS-Kernel V11.2.0, port `RP2350_ARM_NTZ`, SMP 2 cores, 1 kHz tick, heap_4 192 KiB |
| Net | `pico_cyw43_arch_lwip_sys_freertos`, lwIP BSD sockets, CYW43 power save off (`CYW43_NONE_PM`) |
| micro-ROS | Jazzy `libmicroros.a` from Task 2, `rmw_uros_set_custom_transport(false, …)` over UDP |
| Agent | `microros/micro-ros-agent:jazzy udp4 -v4` in Docker, `--net=host --ipc=host` |
| ROS CLI | `ros:jazzy-ros-base`, `--net=host --ipc=host` (Fast DDS SHM needs a shared IPC namespace) |
| Pico | WiFi 10.202.34.34/21 (gw 10.202.32.1), RSSI −64…−75 dBm |
| PC | wired 10.202.20.150/22 — **a different L3 subnet** from the Pico |
| Firmware size | text 433 KB, bss 286 KB (of 520 KB SRAM) |

Firmware: node `umbreon_spike`. `/spike/count` (Int32) is published at 50 Hz from an rclc timer, and
`/spike/echo_in` is echoed to `/spike/echo_out`. Everything uses best-effort QoS. The micro-ROS task
is pinned to core 0. A core 1 task at priority `configMAX_PRIORITIES-2` toggles GP4 every 20 ms with
`vTaskDelayUntil` and logs the worst |period − 20 ms| every 5 s. Liveness is checked by
`rmw_uros_ping_agent(50 ms, 1)` every 500 ms. After 6 failed pings in a row (about 3 s) the session is
destroyed, and then the agent is pinged every 100 ms until it answers again.

### Network workaround used for the measurements (PC-specific)

The Pico's UDP packets reached the agent: at `-v6` the agent logged `GET_INFO` from 10.202.34.34 and
sent `INFO` replies. The replies never arrived, because the PC has a VPN interface `bedrock0` with the
route `10.0.0.0/8 dev bedrock0`. `ip route get 10.202.34.34` → `dev bedrock0`, so replies to the WiFi
subnet leave through the VPN. `ping 10.202.34.34` had 100 % loss. `ping -I enp3s0` worked.

The rules for this spike allow no sudo and no route changes. So all measurements ran through
`spike/udp_relay.py`, a small user-space relay. Its socket is bound to `enp3s0` with `SO_BINDTODEVICE`
(no privilege needed), it listens on :8888, and it forwards to the agent on 127.0.0.1:8890. The relay
adds well under 1 ms (a loopback hop).
**Owner action for real use:** add a route for 10.202.32.0/21 via enp3s0 (10.202.20.1), exclude it from
the VPN, or run the agent on a machine on the car's network.

## Measurements

### `ros2 topic hz /spike/count`

| Run | Average | min / max interval | std dev |
|---|---|---|---|
| 1000-msg window, ping timeout 50 ms | **49.6–49.7 Hz** | 0.000 / 0.253 s | 11 ms |
| first firmware (ping 100 ms × 3 → frequent false session drops) | 50.0 Hz at first, then about 28 Hz | up to 2.7 s gaps | — |

### Echo round trip (`spike/echo_latency.py`: Int32 sequence numbers sent at 20 Hz; RTT measured on the host)

| Samples | p50 | p95 | max | lost |
|---|---|---|---|---|
| 200 | 3.7 ms | 5.7 ms | 123 ms | 0 |
| 200 | 3.8 ms | 13.7 ms | 301 ms | 23 |
| 200 | 4.3 ms | 161.3 ms | 305 ms | 15 |
| 1000 | 3.8 ms | 6.7 ms | 308 ms | 20 (2 %) |
| 500 | 4.4 ms | 8.1 ms | 306 ms | 12 |
| 500 | 5.3 ms | 70.8 ms | 1211 ms | 60 (12 %) |
| **2000** | **5.4 ms** | **46.7 ms** | 1616 ms | 107 (5.4 %) |

(An earlier firmware build, whose ping blocked for up to 100 ms and so caused false session drops,
measured p50 4.2 / p95 111 ms / 43 of 200 lost. That build was superseded.)

Where the tail and the loss come from: the Pico's own counters show that lost echoes never reached the
Pico. In one 500-sample run, the Pico counted 440 of 500 inputs and the host got all 440 back. So the
loss is on the WiFi **downlink** (AP → Pico). An ICMP baseline (`ping -I enp3s0 -i 0.1`, no micro-ROS
involved) shows the same pattern: p50 2.6 ms, p95 157 ms, max 305–318 ms, 1–15 % loss. The wired
path to the WiFi gateway 10.202.32.1 is clean (0.19 ms, 0 % loss). The liveness ping also adds up to
50 ms whenever a pong is lost, because it blocks the executor for that time.

### Agent killed and restarted (`docker kill` then `docker run`)

| Event | Agent down | Detected (kill → `SESSION lost`) | Recovered (agent start → `SESSION up`) |
|---|---|---|---|
| 1 | 16.7 s | 2.7 s | 0.79 s |
| 2 | 6.0 s | 2.93 s | 0.82 s |
| 3 | 6.1 s | 2.61 s | 0.47 s (session up 0.36 s after the container was up) |
| `docker restart -t 0` | ~1 s | 2.8 s after restart: the new agent does not know the old session, so pings in that session fail | 13 ms after detection |

The session recovers every time, with no reboot and no heap growth. Most of the outage is the 3 s
detection window, so the real firmware should detect the loss faster (see the notes below).

### Network loss with the agent still alive (relay stopped for about 20 s)

A substitute for "the AP goes away". It blocks the agent's UDP path without sudo. Detection took
2.7 s (`SESSION lost`). After the relay came back, the agent was reachable within about 1 s and the
session was up 1.5 s later. Entity creation is slower on a lossy link: 6 reliable create requests.

**Real AP off/on: PENDING owner** (switch the AP off for about 30 s, then on, and read the USB log).
The firmware polls `cyw43_tcpip_link_status()` on every loop. When the link is lost it tears down
the session and calls `cyw43_arch_wifi_connect_timeout_ms()` until it reconnects. This code path has not
run yet: during the spike, the CYW43 never lost association. WiFi association at boot takes 3–5 s.

### Spontaneous outages

With nothing being killed, sessions dropped twice in about 4.5 minutes (15:58:03, 15:59:59). In both
cases the CYW43 stayed associated, and 6 pings in a row went unanswered. The outages lasted 3–10 s
each, including detection. This matches the ICMP bursts above. It is a property of the link, not of
the software.

### Core 1 jitter (all of the above, both firmware builds)

126 windows of 5 s each: median worst-case 83 µs, p95 141 µs, **max 275 µs** on a 20 ms period. Agent
kills, session teardown/recreation, WiFi traffic and USB logging had no visible effect beyond this.

### Agent discovery (`AGENT_IP` empty, build with `-DSPIKE_DISCOVERY=ON`)

**Not working here, and not available in the library.** `rmw_uros_discover_agent()` is declared in
the headers but missing from `libmicroros.a`. It is built only with `RMW_UXRCE_TRANSPORT=udp`, and
Task 2 also set `UCLIENT_PROFILE_DISCOVERY=OFF`. As a stand-in, the spike sends the XRCE `GET_INFO`
(what `uxr_ping_agent` sends) over the custom transport. It goes to the subnet broadcast
10.202.39.255:8888, to 255.255.255.255:8888, and to the XRCE discovery multicast 239.255.0.2:7400.
No reply came in 5+ rounds. That is expected: the PC is on a different L3 subnet, and routers do not
forward broadcast or multicast. On a flat network (the car's own AP), sending to the agent port by
broadcast should work. That has not been measured. Plan 3 should keep a configured agent IP and treat
discovery as optional.

## ABI check (Task 2 follow-up)

`arm-none-eabi-readelf -A` on pico2_w objects from this build (`main.c.o`, lwIP `sockets.c.o`) and on
all 1720 objects in `libmicroros.a`:

    Tag_CPU_arch: v8-M.mainline
    Tag_FP_arch: FPv5/FP-D16 for ARMv8
    Tag_ABI_HardFP_use: SP only
    (no Tag_ABI_VFP_args on either => base AAPCS / softfp)

They are identical, so the ABIs match.

## Settings needed (carry over to Plan 3)

lwIP (`spike/lwipopts.h`):
- `NO_SYS 0`, `LWIP_SOCKET 1`, `LWIP_NETCONN 0`, `MEM_LIBC_MALLOC 0` (required with the non-poll CYW43
  arch), `LWIP_TIMEVAL_PRIVATE 0` (newlib `struct timeval`), `LWIP_SO_RCVTIMEO 1`.
- `DEFAULT_UDP_RECVMBOX_SIZE 16`, `TCPIP_MBOX_SIZE 16`, `TCPIP_THREAD_STACKSIZE 2048`, `MEM_SIZE 16000`,
  `PBUF_POOL_SIZE 24`, `IP_SOF_BROADCAST 0` (for the discovery broadcast).
- Transport read: lwIP treats `SO_RCVTIMEO = 0` as "block forever". Use `MSG_DONTWAIT` when uxr passes
  timeout ≤ 0.

CYW43:
- After the WiFi connect, use `cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM)`. Power save adds 100+ ms
  latency.
- `cyw43_arch_init*` must run inside a FreeRTOS task. The spike calls it from the core 0 task.

FreeRTOS (`spike/FreeRTOSConfig.h`): based on pico-examples' common config. `configNUMBER_OF_CORES 2`,
`configUSE_CORE_AFFINITY 1`, `configRUN_MULTIPLE_PRIORITIES 1`, `configENABLE_BACKWARD_COMPATIBILITY 1`
(lwIP sys_arch needs it), `configSUPPORT_PICO_SYNC_INTEROP/TIME_INTEROP 1`, `configENABLE_FPU 1`,
`configENABLE_TRUSTZONE 0`, `configRUN_FREERTOS_SECURE_ONLY 1`, `configMAX_SYSCALL_INTERRUPT_PRIORITY 16`.

micro-ROS glue:
- Install the rcl allocator with `rcutils_set_default_allocator()` before `rclc_support_init`. The spike
  uses `pvPortMalloc`/`vPortFree` with an 8-byte size header so `reallocate` can work.
- `clock_gettime()` comes from `time_us_64()`. `usleep()` (`vTaskDelay` when the time is ≥ 1 ms) must
  be supplied.
- Before tearing down a dead session, call `rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0)`.
  Otherwise each `*_fini` blocks waiting for the agent.

Build:
- The `umbreon-pico-fw-build` image has no arm-none-eabi libstdc++ (`<cstdlib>`, `-lstdc++`). The spike
  works around this with `PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1` and `LINKER_LANGUAGE C`. The
  Plan 3 firmware should either do the same or add `libstdc++-arm-none-eabi-newlib` to the image.

## Notes for the real firmware

- Use a liveness ping that does not block the executor, or keep its timeout short. A 100 ms timeout with
  3 failures caused false session drops on this link. A 50 ms timeout with 6 failures did not, but it
  takes about 3 s to detect a dead agent. The agent/link loss protection in the spec should not rely
  on the session alone. The 250 ms command timeout on the car is what actually stops motion.
- Over this link, expect occasional echo RTTs of 300 ms to 1.6 s and 3–10 s blackouts. Every link
  protection must assume such gaps are normal.

## Reproduce

    docker compose -f docker/compose.yml run --rm fw-build bash -c \
      'cmake -S spike -B build/spike -G Ninja && cmake --build build/spike'
    picotool load -f -x build/spike/umbreon_spike.uf2
    docker run --rm -d --name uros-agent --net=host --ipc=host microros/micro-ros-agent:jazzy udp4 -p 8890 -v4
    python3 spike/udp_relay.py enp3s0 8888 8890      # only on PCs whose route to the Pico is wrong
    docker run --rm --net=host --ipc=host ros:jazzy-ros-base bash -c \
      'source /opt/ros/jazzy/setup.bash; ros2 topic hz /spike/count'
    docker run --rm --net=host --ipc=host -v $PWD/spike:/spike:ro ros:jazzy-ros-base bash -c \
      'source /opt/ros/jazzy/setup.bash; python3 /spike/echo_latency.py 2000 20'
