#!/usr/bin/env python3
"""
set_external_mode.py — one-shot: switch rl_tools_commander to EXTERNAL mode.

Sends `rl_tools_commander set_mode EXTERNAL` to the PX4 NuttX shell over
MAVLink SERIAL_CONTROL (same mechanism as Tools/mavlink_shell.py), then runs
`rl_tools_commander status` and only exits 0 if the reply confirms EXTERNAL.

Uses the FC's USB MAVLink port (default /dev/ttyACM0) — the uXRCE-DDS link on
ttyTHS1 is untouched. Safe failure: if the FC is unreachable or the mode is
not confirmed, EXTERNAL is simply not set; Raptor keeps hovering at its
activation target and ignores the setpoint stream.

Usage:
    python3 set_external_mode.py [/dev/ttyACM0 | udp:IP:PORT] [-b BAUD] [-r RETRIES]
"""
import argparse
import sys
import time
import traceback

from pymavlink import mavutil

SHELL_DEVNUM = 10  # SERIAL_CONTROL_DEV_SHELL


class MavShell:
    """Minimal NuttX-shell-over-SERIAL_CONTROL client (see mavlink_shell.py)."""

    def __init__(self, url: str, baud: int):
        self.mav = mavutil.mavlink_connection(url, baud=baud, autoreconnect=True)
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.mav.wait_heartbeat(timeout=10) is None:
            raise TimeoutError(f"no MAVLink heartbeat on {url}")

    def write(self, text: str):
        data = text.encode()
        while data:
            chunk, data = data[:70], data[70:]
            buf = list(chunk) + [0] * (70 - len(chunk))
            self.mav.mav.serial_control_send(
                SHELL_DEVNUM,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(chunk), buf)

    def read_for(self, duration: float) -> str:
        out = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            m = self.mav.recv_match(condition="SERIAL_CONTROL.count!=0",
                                    type="SERIAL_CONTROL", blocking=True, timeout=0.05)
            if m is not None:
                out.append(bytes(m.data[:m.count]).decode(errors="replace"))
        return "".join(out)

    def close(self):
        self.mav.mav.serial_control_send(SHELL_DEVNUM, 0, 0, 0, 0, [0] * 70)
        self.mav.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0",
                        help="MAVLink port (serial device or udp:IP:PORT)")
    parser.add_argument("-b", "--baudrate", type=int, default=57600)
    parser.add_argument("-r", "--retries", type=int, default=3)
    args = parser.parse_args()

    shell = None
    for attempt in range(1, args.retries + 1):
        try:
            shell = MavShell(args.port, args.baudrate)
            break
        except Exception as e:
            print(f"[set_external_mode] connect attempt {attempt}/{args.retries} "
                  f"failed on {args.port}: {e}", file=sys.stderr)
            traceback.print_exc()
            time.sleep(2.0)
    if shell is None:
        print(f"[set_external_mode] ERROR: cannot reach FC on {args.port}\n"
              f"[set_external_mode] EXTERNAL NOT SET — set it manually: "
              f"rl_tools_commander set_mode EXTERNAL", file=sys.stderr)
        return 1

    try:
        shell.write("\n")
        shell.read_for(0.5)  # drain prompt/banner

        for attempt in range(1, args.retries + 1):
            shell.write("rl_tools_commander set_mode EXTERNAL\n")
            shell.read_for(1.0)
            shell.write("rl_tools_commander status\n")
            reply = shell.read_for(1.5)
            if "EXTERNAL" in reply:
                print("[set_external_mode] mode confirmed: EXTERNAL")
                return 0
            print(f"[set_external_mode] attempt {attempt}/{args.retries}: "
                  f"EXTERNAL not confirmed, reply:\n{reply}", file=sys.stderr)

        print("[set_external_mode] ERROR: EXTERNAL NOT CONFIRMED after "
              f"{args.retries} attempts — set it manually before flying the planner.",
              file=sys.stderr)
        return 1
    finally:
        shell.close()


if __name__ == "__main__":
    sys.exit(main())
