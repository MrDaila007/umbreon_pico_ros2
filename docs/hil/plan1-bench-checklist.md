# Plan 1 bench checklist (owner-run, car on a stand)

Firmware: Pico 2 W, console-driven (no ROS2 yet). Every step gives the exact console input and what you
must see. Record results in the `Result:` lines. Stop and report at the first step that does not match.

## 0. SAFETY (read first)

- **Wheels off the ground.** Car on a stand or a box, wheels spinning freely. Nothing touching the wheels,
  except where a step says to hold one.
- **Hand near the ESC power** (battery connector / switch) for the whole session.
- **Start low.** First movement is `raw 0 0.3`. Do not go above 0.5 on this bench. Reverse is capped by
  firmware at 0.38.
- **Emergency stop, in order of speed:** cut ESC power; unplug the battery; type `stop` (neutral now) or
  `disarm` (neutral and clears both keys). Unplugging the USB console cable, closing the USB port, or
  killing the helper stops a running USB command within about 0.3 s (the firmware drops the repeat when the
  USB host disconnects; the 250 ms timeout does the rest, see step 7). A command typed on the **UART**
  console (GP16/GP17) still runs until its `[ms]` ends (3 s at most), whatever happens to the cable.
- A driving command lasts at most 3 s (`[ms]` defaults to 1000, max 3000) and then the car goes neutral
  by itself (command timeout 250 ms after the last refresh).
- Keep the steering servo away from your fingers. Do not run any step that is not on this list.

## Tools

- Console: USB CDC `/dev/ttyACM*` (e.g. `picocom -b 115200 /dev/ttyACM0`, or `screen /dev/ttyACM0 115200`).
  Line endings CR or LF both work. Type `help` for the command list.
- Helper: `python3 tools/console_drive.py --help` (needs `pip install pyserial`).
  Examples: `--status`; `--send "raw 0 0"`; `--stop-after 1` (see step 7).

## 1. Flash

- [ ] Build: `make fw` -> `build/fw/umbreon_pico_ros2.uf2`.
- [ ] Flash: `picotool load -x -f build/fw/umbreon_pico_ros2.uf2`, or hold BOOTSEL, plug in, drag the
  `.uf2` onto the `RP2350` drive.
  Expected: picotool prints `The device was rebooted to start the application.`; `/dev/ttyACM0` appears.

Result:

## 2. Boot state

- [ ] Open the console, type `status`. Expected, exactly:
  ```
  outputs: servo 1500 us, esc 1500 us, motion no, speed 0.00 m/s
  armed: no (mode both, car key no, ros key no, neutral seen no)
  stop: no_cmd
  gear: neutral, drive mode: manual
  reflex: detected none, intervening none
  settings: defaults
  link: wifi down, agent down
  reset: power-on / reset pin / debugger
  ```
  (`stop:` may read `cmd_timeout` later; `settings:` reads `slot N (seq M)` once saved.) Wheels do not move.

Result:

## 3. Arming (mode `both` is the default)

- [ ] `raw 0 0.3` -> reply `ok: 1000 ms` and `note: disarmed, outputs stay neutral`. Wheels do not move.
- [ ] `arm car` -> `armed: no (car key yes, ros key no; motion needs a neutral command first)`.
  Then `arm ros` -> `armed: yes (car key yes, ros key yes; ...)`.
- [ ] Still no motion: `raw 0 0.3 500` right now must NOT move the wheels (no neutral yet).
  Send `raw 0 0 3000` (neutral), then within about 1 s `status` shows `armed: yes (mode both, car key yes, ros key yes, neutral seen yes)`.
- [ ] `raw 0 0.3 2000` -> wheels turn for about 2 s, then stop on their own. During the run `status` shows
  `motion yes` and `stop: none`; afterwards `esc 1500 us` and `stop: cmd_timeout`.
  If the wheels do not turn at 0.3 (below the motor's start), repeat with 0.4, never above 0.5.
- [ ] **Latch:** `raw 0 0.3 2000` again, with no neutral in between. Wheels do **not** move.
  Then `raw 0 0`, wait 0.5 s, `raw 0 0.3 2000` -> moves again.
- [ ] `disarm` -> `armed: no (car key no, ros key no; ...)`. `raw 0 0` then `raw 0 0.3` does nothing.

Result:

## 4. Reverse

- [ ] `arm car`, `arm ros`, `raw 0 0`, then `raw 0 -0.5 3000`.
  Expected: brake pulse, pause, then reverse (`gear:` in `status` goes brake, gap, reverse). The wheels turn
  backwards. The throttle is capped at 0.38 in reverse (`max_throttle_rev`).
  The car stops by itself after 3 s.

Result:

## 5. Steering limits and settings persistence

Defaults: `min_point 60`, `neutral_point 90`, `max_point 120` (servo degrees; pulse = 700 + deg*1600/180 us, integer division).

- [ ] `raw -1 0 3000`, and type `status` while it is still running (within the 3 s; the servo only steers while
  a command is live and returns to 1500 afterwards): `servo 1233 us` (default). Same with `raw 1 0 3000`:
  `servo 1766 us`. `raw 0 0 3000`: `servo 1500 us`.
  The steering must reach each end without binding or buzzing. (Needs arming and a neutral first, as in step 3.)
- [ ] If it binds, adjust (disarmed): `get min_point`, `set min_point 65`, `set max_point 115`,
  `set neutral_point 92` (each replies `ok (not saved; run save)`). Re-check with `raw -1 0` / `raw 1 0`.
- [ ] `disarm`, then `save` -> `ok: saved to slot 0 (seq 1)` (slot and seq may differ).
  `save` is refused with `refused: armed` while armed and `refused: driving` while a run is active.
  In `mode none` the car is always armed, so `save` is refused there too.
- [ ] **Power-cycle** (unplug and replug, or `reboot`). `get min_point` / `get max_point` / `get neutral_point`
  return the saved values, and `status` shows `settings: slot N (seq M)` instead of `defaults`.
- [ ] `defaults` loads defaults (not saved). Finish with the values you want and `save` again.

Record final `min_point` / `neutral_point` / `max_point`:

Result:

## 6. Closed-loop speed

- [ ] `mode both`, `arm car`, `arm ros`, `raw 0 0`, then `speed 0 0.5 3000`.
  Expected: steady wheel speed, and `status` taken during the run shows `speed` about 0.50 m/s
  (`outputs: ... speed 0.50 m/s`). Type `status` immediately after the line (replies arrive while it runs).
  Record the tach readings (several `status` lines) and the wheel's behaviour.
  The tachometer is GP13, 2 holes, 60 mm wheel by default (`get encoder_holes`, `get wheel_diam_m`).

Result:

## 7. Command loss: the car must stop by itself

Console repetition: the firmware repeats a `raw/speed/auto` line every 20 ms for its `[ms]` (max 3 s) on
its own. A line typed on the USB console stops repeating as soon as the USB host disconnects (cable
unplugged, port closed, helper killed: the host drops DTR); no neutral is posted, so the command timeout
(250 ms) stops the car and motion then needs a fresh neutral. A UART-origin line repeats to its `[ms]`.
Test the PC-dying case with the helper, which keeps refreshing a short command and then goes silent:

- [ ] Close the terminal program (the port must be free). Arm and neutral first (step 3), then:
  `python3 tools/console_drive.py --send "arm car" --send "arm ros" --send "raw 0 0" --send "raw 0 0.3" --stop-after 1 --status`
  Expected: wheels turn for about 1 s, then stop within about 0.3 s of the script going silent (60 ms repeat
  tail + 250 ms timeout). The `status` printed afterwards shows `esc 1500 us` and `stop: cmd_timeout`.
- [ ] Helper closes the port mid-run: arm and neutral first, then
  `python3 tools/console_drive.py --send "raw 0 0.3 3000" --no-final-stop`
  (the script sends the line, waits about 0.3 s and exits; closing the port drops DTR; `--no-final-stop`
  makes sure no `stop` is sent). Expected: the wheels stop within about 0.3 s of the script exiting, well
  before the 3 s of `[ms]`. Repeat with a terminal: `raw 0 0.3 3000`, then quit picocom/screen at once;
  same expectation. `status` afterwards shows `esc 1500 us`, `stop: cmd_timeout`; the next movement needs a
  fresh neutral (`raw 0 0`) first.
- [ ] Physical unplug: `raw 0 0.3 3000`, pull the USB console cable at once. Expected: the car stops within
  about 0.3 s of the unplug. If the Pico is powered from USB it simply loses power with the cable and the
  wheels stop immediately (see step 8 for what the ESC does with no pulses). Record what happened.

Result:

## 8. ESC with no pulses (BOOTSEL)

Purpose: see what the ESC does when the Pico stops sending pulses (the Pico's own outputs go away).

The Pico 2 W has no reset button. Two ways to put it in BOOTSEL (no PWM) while the motor is turning:

- **picotool** (preferred, USB cable plugged in): keep picocom/screen **open** on `/dev/ttyACM0` (closing
  it would stop the command, see step 7) and type the drive line there; then, from a second shell, run
  `picotool reboot -u -f`. This forces BOOTSEL through the USB reset interface (it does not need the
  serial port); PWM stops at once.
- **Power cycle**: hold BOOTSEL and cycle VSYS (the Pico's own supply) while the motor is turning.

BEC caveat: if the Pico is powered from the ESC's BEC, unplugging USB does **not** reset it; use picotool
or cycle the BEC/VSYS supply with BOOTSEL held.

- [ ] Arm, neutral, then `raw 0 0.3 3000` and, while the wheels are turning, put the Pico in BOOTSEL by one
  of the two ways above. The Pico is now in the bootloader and sends no PWM.
  Expected: the ESC stops the motor and stays stopped (failsafe on signal loss). **If the wheels keep
  turning, cut ESC power at once** and report it.
  Record the ESC model and the result: ____________________

Result:

## 9. Stall reflex

Hold-wheel test. Keep the hold short (under 3 s per run); release immediately after each run.

- [ ] Autonomous (default reflex action `pico`): `arm car`, `arm ros`, `auto 0 0`, then hold one rear
  wheel still and `auto 0 0.5 3000`. Expected: after about 0.7 s the ESC drops to neutral and
  `status` shows `reflex: detected stall, intervening stall`, with `esc 1500 us`. Release the wheel.
- [ ] Manual (default reflex action `warn`): `raw 0 0`, hold the wheel and `raw 0 0.5 3000`.
  (0.5, not 0.3: the stall detector needs the ESC at least 60 us from neutral; 0.3 is only about 58 us.)
  Expected: the motor keeps pushing (esc stays above 1500) and after about 0.7 s `status` shows
  `reflex: detected stall, intervening none`. Release the wheel at once.
- [ ] After either run: `raw 0 0` (neutral) clears it; `status` shows `detected none`.

Result:

## 10. Mode none (no arming)

- [ ] `mode none` -> `ok (not saved; run save)`. `status` first line is
  `WARNING: NO ARMING (mode none): any non-neutral command moves the car`.
- [ ] `raw 0 0`, then `raw 0 0.3 2000`: the wheels move with no `arm` keys. A neutral is still required first.
- [ ] Restore: `mode both` (disarms). `status` must show `armed: no (mode both, ...)`. Do not `save` mode none.

Result:

## 11. UART0 console

- [ ] Connect a USB-serial adapter (3.3 V logic): adapter RX to GP16 (Pico TX), adapter TX to GP17 (Pico RX),
  GND to GND. Do not connect the adapter's 5 V.
  `picocom -b 115200 /dev/ttyUSB0`, then `help`, `status`, `arm car`, `disarm`.
  Expected: the same replies as over USB, echoed on the UART. Both consoles can be open together.
  (Motion over UART is optional; if tried, apply the same safety rules.)

Result:

## 12. Deferred to Plan 3 (not testable now: no networking yet)

- WiFi AP on/off behaviour (`wifi_mode`, `sta_timeout_s`): `link:` always reads `wifi down, agent down` in Plan 1.
- Re-measure micro-ROS latency on the car's own AP (spike gate p95 46.7 ms vs 50 ms, almost no margin;
  see `docs/spike-results.md`).
- PC VPN route: `10.0.0.0/8 dev bedrock0` sends replies to the Pico's WiFi subnet out through the VPN, so the
  agent never gets answers. Drop the route/VPN or run the agent on the car's network
  (`docs/spike-results.md`).

## Sign-off

| Date | Firmware (git sha) | ESC model | Result |
|------|--------------------|-----------|--------|
|      |                    |           |        |
