#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Probe an OpenYAM (Anvil) on a CAN bus. Phase-0 hardware verification.

Enumerates the expected Damiao motors (6 arm joints + gripper, send IDs
1..7), enables each, reads back one state frame, then disables. Every frame
seen on the bus is printed — including ones on unexpected IDs — so if the
arm's actual ID map or protocol differs from the assumed layout, this
script is how you find out. Run it BEFORE any coordinator blueprint.

Works on Linux (SocketCAN, bring the bus up first with openyam_can_up.sh)
and macOS (userspace gs_usb via libusb — `brew install libusb`,
`pip install 'python-can>=4.3' gs_usb`; SLCAN serial as fallback).

Usage:
    # Linux
    python openyam_can_probe.py --channel can0
    # macOS with a candlelight-firmware dongle (CANable 2.0 etc.)
    python openyam_can_probe.py --channel gs_usb:0
    # macOS with a serial-firmware dongle
    python openyam_can_probe.py --channel /dev/tty.usbmodem1101
    # Listen only, no enable frames (safe: arm cannot move)
    python openyam_can_probe.py --channel gs_usb:0 --listen 5
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed. Run: pip install 'python-can>=4.3'")

from dimos.hardware.manipulators.openyam.driver import resolve_transport

# [p_max rad, v_max rad/s, t_max Nm] — Damiao datasheet values
LIMITS: dict[str, tuple[float, float, float]] = {
    "DM3507": (12.5, 50.0, 5.0),
    "DM4310": (12.5, 30.0, 10.0),
    "DM4340": (12.5, 8.0, 28.0),
}

# Presumed OpenYAM layout (I2RT YAM convention). If motors don't reply here,
# check the unexpected-frame log below for the real IDs.
DEFAULT_MOTORS: list[tuple[int, str]] = [
    (0x01, "DM4340"),  # joint1
    (0x02, "DM4340"),  # joint2
    (0x03, "DM4340"),  # joint3
    (0x04, "DM4310"),  # joint4
    (0x05, "DM4310"),  # joint5
    (0x06, "DM4310"),  # joint6
    (0x07, "DM4310"),  # gripper
]

ENABLE = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])


def uint_to_float(x: int, lo: float, hi: float, bits: int) -> float:
    return x / ((1 << bits) - 1) * (hi - lo) + lo


def parse_state(motor_type: str, data: bytes) -> tuple[float, float, float, int, int] | None:
    if len(data) < 8:
        return None
    p_max, v_max, t_max = LIMITS[motor_type]
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    q = uint_to_float(q_u, -p_max, p_max, 16)
    dq = uint_to_float(dq_u, -v_max, v_max, 12)
    tau = uint_to_float(tau_u, -t_max, t_max, 12)
    return q, dq, tau, data[6], data[7]


def _fmt_frame(msg: can.Message) -> str:
    data = " ".join(f"{b:02X}" for b in msg.data)
    return f"id=0x{msg.arbitration_id:03X} dlc={msg.dlc} [{data}]"


def probe_motor(
    bus: can.BusABC, send_id: int, recv_id: int, motor_type: str, timeout: float
) -> bool:
    while bus.recv(0.0) is not None:  # flush stale frames
        pass
    bus.send(can.Message(arbitration_id=send_id, data=ENABLE, is_extended_id=False))
    t0 = time.monotonic()
    replied = False
    while time.monotonic() - t0 < timeout:
        msg = bus.recv(max(0.0, timeout - (time.monotonic() - t0)))
        if msg is None:
            break
        if msg.arbitration_id != recv_id:
            # Frames on unexpected IDs are the protocol-discovery signal.
            print(f"    unexpected frame: {_fmt_frame(msg)}  (echo byte 0x{msg.data[0]:02X})")
            continue
        parsed = parse_state(motor_type, bytes(msg.data))
        if parsed is None:
            print(f"  0x{send_id:02X} ({motor_type}): short reply {_fmt_frame(msg)}")
            break
        q, dq, tau, t_mos, t_rot = parsed
        print(
            f"  0x{send_id:02X} ({motor_type:>6}): "
            f"q={q:+.3f} rad  dq={dq:+.3f} rad/s  tau={tau:+.3f} Nm  "
            f"T_mos={t_mos}C  T_rotor={t_rot}C"
        )
        replied = True
        break
    if not replied:
        print(f"  0x{send_id:02X} ({motor_type:>6}): NO REPLY on 0x{recv_id:02X}")
    bus.send(can.Message(arbitration_id=send_id, data=DISABLE, is_extended_id=False))
    return replied


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--channel",
        default="can0",
        help="can0 (SocketCAN), gs_usb[:N] (macOS/candlelight), or /dev/tty* (SLCAN)",
    )
    ap.add_argument("--interface", default=None, help="Force a python-can interface name")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--ids", default=None, help="Comma-separated send IDs (default: 1..7)")
    ap.add_argument("--timeout", type=float, default=0.3, help="Reply timeout per motor (s)")
    ap.add_argument(
        "--listen",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="Passive mode: send nothing, print every frame seen for N seconds",
    )
    args = ap.parse_args()

    motors = DEFAULT_MOTORS
    if args.ids:
        wanted = {int(x, 0) for x in args.ids.split(",")}
        motors = [m for m in DEFAULT_MOTORS if m[0] in wanted]

    interface, channel, bus_kwargs = resolve_transport(
        args.channel, interface=args.interface, bitrate=args.bitrate
    )
    print(f"Opening {channel} via {interface} {bus_kwargs or ''}...")
    try:
        bus = can.Bus(interface=interface, channel=channel, **bus_kwargs)
    except Exception as e:
        print(f"ERROR opening {channel}: {e}", file=sys.stderr)
        if interface == "socketcan":
            print(
                "  Bring the bus up first: sudo ./dimos/robot/manipulators/openyam/scripts/openyam_can_up.sh",
                file=sys.stderr,
            )
        elif interface == "gs_usb":
            print(
                "  gs_usb needs: pip install gs_usb; libusb (macOS: brew install libusb).\n"
                "  If the dongle runs serial firmware, pass its /dev/tty* path instead.",
                file=sys.stderr,
            )
        return 1

    try:
        if args.listen > 0:
            print(f"Listening for {args.listen:.1f}s (no frames sent)...")
            n = 0
            deadline = time.monotonic() + args.listen
            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.2)
                if msg is not None:
                    n += 1
                    print(f"  {_fmt_frame(msg)}")
            print(f"{n} frame(s) seen.")
            return 0

        print(f"Probing {len(motors)} motor(s):")
        ok = 0
        for send_id, motor_type in motors:
            if probe_motor(bus, send_id, send_id | 0x10, motor_type, args.timeout):
                ok += 1
        print(f"\n{ok}/{len(motors)} motors replied.")
        if ok == 0:
            print(
                "No replies at all. Check: 24V power on, CAN-H/L not swapped, "
                "termination present, bitrate 1M, and re-run with --listen 5 "
                "while wiggling the arm — any frames printed reveal the real ID map."
            )
        return 0 if ok == len(motors) else 2
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
