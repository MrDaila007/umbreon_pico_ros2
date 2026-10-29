#!/usr/bin/env python3
"""console_drive.py - drive the Umbreon Pico console from a PC (bench use).

Needs Python 3 and pyserial (pip install pyserial). --help works without it.

Examples (car on a stand, wheels off the ground):
  tools/console_drive.py --status
  tools/console_drive.py --send "arm car" --send "arm ros" --send "raw 0 0"
  tools/console_drive.py --send "arm car" --send "arm ros" --send "raw 0 0" \
      --send "raw 0 0.3" --stop-after 1

--stop-after S simulates the PC dying: the LAST --send line (a raw/speed/auto
command) is streamed every 20 ms for S seconds, then the script goes silent
and waits 1 s before anything else, so you can watch the car stop on its own
(the final `stop` on exit is harmless by then). Each
streamed line carries a short [ms] (--stream-ms, default 60) so the firmware's
own repeat ends soon after the last line; the command-timeout does the rest.
"""
import argparse
import sys
import time

STREAM_PERIOD_S = 0.02
DRIVE_WORDS = ("raw", "speed", "auto")


def parse_args(argv):
    p = argparse.ArgumentParser(description="Send Umbreon console lines over serial.")
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port (default %(default)s)")
    p.add_argument("--baud", type=int, default=115200, help="baud rate (default %(default)s)")
    p.add_argument("--send", action="append", default=[], metavar="LINE",
                   help="console line to send; repeatable, sent in order")
    p.add_argument("--delay", type=float, default=0.3,
                   help="seconds between --send lines (default %(default)s)")
    p.add_argument("--status", action="store_true", help="send `status` and print the reply")
    p.add_argument("--stop-after", type=float, metavar="S",
                   help="stream the last --send raw/speed/auto line every 20 ms for S seconds, "
                        "then go silent (simulates the PC dying)")
    p.add_argument("--stream-ms", type=int, default=60,
                   help="[ms] put on each streamed line with --stop-after (default %(default)s)")
    p.add_argument("--no-final-stop", action="store_true",
                   help="do not send `stop` on exit / Ctrl-C")
    return p.parse_args(argv)


def drive_line_with_ms(line, ms):
    """Return `line` (raw/speed/auto <a> <b> [ms]) with its [ms] set to ms, or None."""
    parts = line.split()
    if len(parts) not in (3, 4) or parts[0] not in DRIVE_WORDS:
        return None
    return " ".join(parts[:3] + [str(ms)])


def read_available(ser):
    data = ser.read(ser.in_waiting or 0)
    return data.decode("utf-8", errors="replace") if data else ""


def send_line(ser, line):
    ser.write((line + "\r\n").encode("ascii"))
    ser.flush()


def settle_read(ser, seconds):
    """Read replies for a while and return them as one string."""
    end = time.monotonic() + seconds
    out = []
    while time.monotonic() < end:
        out.append(read_available(ser))
        time.sleep(0.01)
    return "".join(out)


def run_sends(ser, lines, delay):
    for line in lines:
        send_line(ser, line)
        reply = settle_read(ser, delay)
        sys.stdout.write(f">>> {line}\n{reply}")
        if reply and not reply.endswith("\n"):
            sys.stdout.write("\n")


def stream_then_silent(ser, line, seconds):
    end = time.monotonic() + seconds
    print(f"streaming '{line}' every 20 ms for {seconds:g} s, then silent")
    while time.monotonic() < end:
        send_line(ser, line)
        read_available(ser)  # drop the "ok" replies so the input buffer never fills
        time.sleep(STREAM_PERIOD_S)
    print("silent now (PC 'died'): the car must stop on its own")


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    sends = list(args.send)
    stream_line = None
    if args.stop_after is not None:
        if not sends:
            print("error: --stop-after needs at least one --send raw/speed/auto line", file=sys.stderr)
            return 2
        stream_line = drive_line_with_ms(sends.pop(), args.stream_ms)
        if stream_line is None:
            print("error: the last --send must be 'raw|speed|auto <steer> <value>'", file=sys.stderr)
            return 2
    if not (sends or stream_line or args.status):
        print("nothing to do: give --send, --status or --stop-after (see --help)", file=sys.stderr)
        return 2

    try:
        import serial  # lazy: --help must work without pyserial
    except ImportError:
        print("error: pyserial is not installed (pip install pyserial)", file=sys.stderr)
        return 2
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as exc:
        print(f"error: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    try:
        read_available(ser)
        run_sends(ser, sends, args.delay)
        if stream_line:
            stream_then_silent(ser, stream_line, args.stop_after)
            time.sleep(1.0)  # leave the car alone: it must stop by itself
        if args.status:
            run_sends(ser, ["status"], max(args.delay, 0.5))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if not args.no_final_stop:
            try:
                send_line(ser, "stop")
                print("sent final stop")
            except Exception as exc:  # port may be gone
                print(f"warning: final stop failed: {exc}", file=sys.stderr)
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
