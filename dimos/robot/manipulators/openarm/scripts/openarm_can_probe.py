#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Probe an OpenArm on a SocketCAN interface.

Enumerates all 8 expected Damiao motors (7 arm joints + gripper) on one CAN bus
(classical by default, use --fd for CAN-FD), enables each, reads back one state
frame, then disables. Phase-0 hardware-verification script.

Run AFTER bringing the bus up with dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh.

Usage:
    python dimos/robot/manipulators/openarm/scripts/openarm_can_probe.py --channel can0
    python dimos/robot/manipulators/openarm/scripts/openarm_can_probe.py --channel can1 --ids 1,2,3,4,5,6,7
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed. Run: pip install 'python-can>=4.3'")

# ---- Damiao motor limit tables (from enactic/openarm_can dm_motor_constants.hpp)
#      [p_max rad, v_max rad/s, t_max Nm]
LIMITS: dict[str, tuple[float, float, float]] = {
    "DM4310": (12.5, 30.0, 10.0),
    "DM4340": (12.5, 8.0, 28.0),
    "DM8006": (12.5, 45.0, 40.0),
}

# OpenArm v10 per-joint motor assignment (derived from joint_limits.yaml effort column)
DEFAULT_MOTORS: list[tuple[int, str]] = [
    (0x01, "DM8006"),  # joint1
    (0x02, "DM8006"),  # joint2
    (0x03, "DM4340"),  # joint3
    (0x04, "DM4340"),  # joint4
    (0x05, "DM4310"),  # joint5
    (0x06, "DM4310"),  # joint6
    (0x07, "DM4310"),  # joint7
    (0x08, "DM4310"),  # gripper
]

ENABLE = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])

FD = False  # set by --fd at runtime; defaults to classical CAN @ 1 Mbit


def uint_to_float(x: int, lo: float, hi: float, bits: int) -> float:
    return x / ((1 << bits) - 1) * (hi - lo) + lo


def parse_state(motor_type: str, data: bytes) -> tuple[float, float, float, int, int] | None:
    """Decode an 8-byte DM motor state reply. Returns (q, dq, tau, t_mos, t_rotor)."""
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


def probe_motor(
    bus: can.BusABC, send_id: int, recv_id: int, motor_type: str, timeout: float = 0.2
) -> bool:
    """Enable motor, wait for state reply on recv_id, print result, disable."""
    # Flush any stale frames
    while bus.recv(0.0) is not None:
        pass

    bus.send(
        can.Message(
            arbitration_id=send_id, data=ENABLE, is_extended_id=False, is_fd=FD, bitrate_switch=FD
        )
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = bus.recv(timeout - (time.monotonic() - t0))
        if msg is None:
            break
        if msg.arbitration_id != recv_id:
            continue
        parsed = parse_state(motor_type, bytes(msg.data))
        if parsed is None:
            print(f"  0x{send_id:02X} ({motor_type}): short reply {list(msg.data)}")
            bus.send(
                can.Message(
                    arbitration_id=send_id,
                    data=DISABLE,
                    is_extended_id=False,
                    is_fd=FD,
                    bitrate_switch=FD,
                )
            )
            return False
        q, dq, tau, t_mos, t_rot = parsed
        print(
            f"  0x{send_id:02X} ({motor_type:>6}): "
            f"q={q:+.3f} rad  dq={dq:+.3f} rad/s  tau={tau:+.3f} Nm  "
            f"T_mos={t_mos}C  T_rotor={t_rot}C"
        )
        bus.send(
            can.Message(
                arbitration_id=send_id,
                data=DISABLE,
                is_extended_id=False,
                is_fd=FD,
                bitrate_switch=FD,
            )
        )
        return True

    print(
        f"  0x{send_id:02X} ({motor_type:>6}): NO REPLY on 0x{recv_id:02X} within {timeout * 1e3:.0f}ms"
    )
    bus.send(
        can.Message(
            arbitration_id=send_id, data=DISABLE, is_extended_id=False, is_fd=FD, bitrate_switch=FD
        )
    )
    return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--channel", default="can0", help="SocketCAN interface (default: can0)")
    ap.add_argument(
        "--fd",
        action="store_true",
        help="Use CAN-FD (requires FD-capable adapter). Default is classical CAN @ 1 Mbit, which is what most gs_usb adapters support.",
    )
    ap.add_argument("--ids", default=None, help="Comma-separated send IDs to probe (default: 1..8)")
    ap.add_argument("--timeout", type=float, default=0.2, help="Reply timeout per motor (s)")
    args = ap.parse_args()

    global FD
    FD = args.fd
    motors = DEFAULT_MOTORS
    if args.ids:
        wanted = {int(x, 0) for x in args.ids.split(",")}
        motors = [m for m in DEFAULT_MOTORS if m[0] in wanted]

    # Preflight: is the interface up?
    try:
        flags = int(open(f"/sys/class/net/{args.channel}/flags").read().strip(), 16)
        iface_up = bool(flags & 0x1)
    except OSError:
        print(f"ERROR: interface '{args.channel}' not found", file=sys.stderr)
        return 1
    if not iface_up:
        print(f"ERROR: SocketCAN interface '{args.channel}' is DOWN.", file=sys.stderr)
        print(
            f"  Run: sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh {args.channel}",
            file=sys.stderr,
        )
        return 1

    print(f"Opening {args.channel} ({'CAN-FD' if FD else 'classical CAN'})...")
    try:
        bus = can.Bus(interface="socketcan", channel=args.channel, fd=FD)
    except Exception as e:
        print(f"ERROR opening {args.channel}: {e}", file=sys.stderr)
        print(
            "  Did you run 'sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh' first?",
            file=sys.stderr,
        )
        return 1

    try:
        print(f"Probing {len(motors)} motor(s) on {args.channel}:")
        ok = 0
        for send_id, motor_type in motors:
            recv_id = send_id | 0x10
            if probe_motor(bus, send_id, recv_id, motor_type, args.timeout):
                ok += 1
        print(f"\n{ok}/{len(motors)} motors replied.")
        return 0 if ok == len(motors) else 2
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
