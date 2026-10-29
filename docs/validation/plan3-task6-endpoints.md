# Plan3 Task6: WiFi orchestration, AP provisioning and endpoint selection

Task 6 software is complete. ROS sessions/entities are Task 9: `agent_up` stays
false here, and a selected endpoint is only "an XRCE agent answered a ping".

## What runs

`net_task` (core 0, idle+3) installs the platform hooks, brings up CYW43 once
(retried every 5 s if it fails) and then calls `net_owner_step` every 10 ms. It
always yields at least one tick, so the console (idle+2) stays responsive. It is
the only core-0 heartbeat writer and beats before and after each iteration.

`net_owner` (app, host-tested) makes one bounded pass per iteration:

1. Reads the shared settings. `agent_ip`/`agent_port` apply in place: the
   endpoint policy cancels the probe or selection. WiFi fields (mode, SSID,
   passwords, `sta_timeout_s`) are staged. The owner drops the link (loss is
   latched), closes the probe socket, and only then stops both radios and
   restarts the FSM. If the close fails, the change waits and is retried.
2. Provisions a missing AP password only when the FSM is about to start the AP.
3. Steps `net_wifi`: async STA join, AUTO fallback to AP, 60 s STA retry from AP
   when no session is up, bounded backoff. Loss → `shared_set_link(false,false)`
   runs before any teardown or radio I/O. Link loss bumps the session
   generation, so no pending command crosses into a new link.
4. Runs endpoint selection with one ≤100 ms XRCE probe at most per iteration.
   After a failed round (one STA probe, or one probe per DHCP lease in AP mode),
   it waits 1 s and leaves the device offline. It never loops on discovery.
   A probe socket that fails to close is retried before any further probe and
   blocks radio changes.
5. Publishes `shared_network_t` (no credentials, no STA SSID). The console
   `status` prints it as `network:` / `endpoint:` / `ap:` / `net loop:` lines.

Endpoint rules are unchanged from the earlier checkpoint. STA uses the
configured IP first; a zero IP sends a subnet-broadcast XRCE probe. AP mode
ignores the STA IP and probes associated DHCP leases round-robin. Only a valid
XRCE reply selects an endpoint, and lease expiry or disconnection invalidates
it. Multicast discovery is **not** implemented. It was never verified on this
transport, and the linked library lacks `rmw_uros_discover_agent`.

`umb_net_fsm` fix: in AUTO mode with station credentials, a failed AP start
now retries the station first. Before the fix, an unprovisioned or failing AP
kept the radio in AP retries forever.

## AP password provisioning

- `umb_network_identity` builds a 16-character password from 80 bits of
  entropy. The 32-symbol alphabet `23456789ABCDEFGHJKLMNPQRSTUVWXYZ` leaves out
  0/O/1/I/l/o. Entropy comes from `hal_rand_fill16` (`pico_rand`, `get_rand_128`).
- `settings_provision_ap_pass` reserves maintenance and waits for control's
  neutral ACK. It then persists the complete current settings plus the password
  and verifies them. Only after that does it publish the password to shared
  settings, through `shared_provision_ap_pass`, which is scoped to the
  reservation. It never overwrites an existing `ap_pass`.
- `umb_ap_provision` (pure) bounds the retries. When the car is armed or there
  is no neutral ACK, it retries after 1 s and nothing is written. Missing
  entropy or a flash failure backs off 10 → 20 → 40 → 60 s. After 3 flash write
  attempts it gives up until reboot. On failure `ap_pass` stays empty, so
  `net_wifi` refuses to start the AP: there is never an open or unknown AP.
  `net_task` prints one non-secret error line and posts the error cue.
- If a password already exists, or the mode is STA only, or STA joins, nothing
  is written at boot.

## Watchdog around flash writes

`hal_settings_save` re-arms the hardware watchdog to 2000 ms around
`flash_safe_execute`. It waits one control period first, so core 1's own feed
reloads the extended value: the SDK reload value is shared, which closes the
race. It restores 250 ms afterwards. Core 0 never feeds on control's behalf.
This replaces the earlier requirement to bound the write inside 250 ms
(controller ruling, Task 6).

## Host evidence

- `test_net_owner` (9 fake-time end-to-end tests, real owner + bridges +
  coordinator + shared state). Covers: offline radio; async STA join with the
  configured IP; STA broadcast rounds bounded to about 1/s; STA timeout → AP
  with phone first (no XRCE reply) and PC second (selected); the STA fixed IP
  never probed in AP mode; disconnect and lease expiry invalidating the
  selection; 60 s STA retry with loss latched before any radio stop;
  provisioning once and then reboot with no second write; flash failure
  (exactly 3 writes in 10 minutes, AP never started); armed car (busy, no write,
  succeeds after disarm); entropy failure; AUTO and STA-only doing no write;
  endpoint-only edits (no radio stop); a WiFi edit staged behind a socket that
  failed to close; STA link loss latched before rejoin.
- `test_ap_provision` (4), `test_network_identity` (+1), `test_settings_apply`
  (+3), `test_net_fsm` (+1).
- Full suite: 400 C tests in 36 executables plus 10 Python, all pass. Core and
  driver coverage is 99.4% (1715/1725). App coverage: net_owner 96% (the
  uncovered lines are the provisioning-race `ALREADY_SET` branch), net_wifi,
  net_endpoint and settings_apply 100%. ASan/UBSan pass for endpoint,
  ros_command, network_identity and ap_provision. Logs are ignored
  `build/plan3-task6-{fw,coverage,sanitizers}.log`.

## Hardware evidence (bare Pico 2 W, no robot board), 2026-10-30

- The image boots, joins STA asynchronously and the console stays responsive
  throughout.
- One-time `radio init` takes 568 ms. `hal_wifi_init` now powers the WLAN chip
  with a read-only query: the first ioctl downloads the CYW43 firmware, and it
  also fills the MAC. Before this fix the AP SSID came out as `umbreon-0000`.
  Afterwards it is MAC-derived.
- Steady loop takes 1–2 ms. The worst iteration is **243–244 ms**, the deferred
  `START_STA` (`cyw43_arch_enable_sta_mode` + `cyw43_arch_wifi_connect_async`,
  inside the SDK). It happens once per join attempt. It stays below the 500 ms
  core-1 stale-beat limit but exceeds the 100 ms heartbeat target. See the
  concern in the task report.
- Configured-IP probe: with no agent, probes failed at about 1/s. With
  `microros/micro-ros-agent:jazzy` running, the endpoint showed as configured
  and selected from a real XRCE ping. The agent was stopped afterwards.
- One `save` while disarmed and stationary returned
  `ok: saved to slot 0 (seq 1, flash write 5 ms)`. The board did not reset
  (reset reason stayed power-on), and `watchdog: 237 ms left` showed the
  250 ms timeout restored. After a reboot, `settings: slot 0 (seq 1)` loaded
  from flash. The 5 ms is the measured `flash_safe_execute` duration for this
  part; the 2000 ms window does not depend on it.
- `get ap_pass` still prints `***`.

## Not verified on hardware (PENDING, owner-run per Task 14)

- AP fallback and automatic provisioning on the board. STA joined, so no
  provisioning write happened, and the agent may not change live settings.
- AP DHCP phone/PC selection, STA broadcast discovery, 60 s STA retry, and
  owner setting changes on the live link.
- AP start blocking time.
