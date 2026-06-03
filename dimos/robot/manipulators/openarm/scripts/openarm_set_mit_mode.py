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

"""Write CTRL_MODE = MIT (1) to one or all OpenArm motors.

Damiao motors have a persistent CTRL_MODE register (RID=10). If a motor was
previously configured in POS_VEL (2) / VEL (3) / POS_FORCE (4) mode, it will
respond to enable/disable but IGNORE MIT control frames — exactly the
"motor doesn't move, error grows" symptom.

This script writes CTRL_MODE=1 (MIT) via the 0x7FF broadcast-write frame
format used by enactic/openarm_can:

    ID=0x7FF  data = [id_lo, id_hi, 0x55, RID=10, val[0], val[1], val[2], val[3]]

Run once per motor after CAN bring-up. The value is persistent across power
cycles.

Usage:
    # All 8 motors on can0 (classical CAN @ 1 Mbit, default)
    python dimos/robot/manipulators/openarm/scripts/openarm_set_mit_mode.py --channel can0

    # Single motor
    python dimos/robot/manipulators/openarm/scripts/openarm_set_mit_mode.py --channel can0 --id 0x05

    # CAN-FD (only if your adapter supports it)
    python dimos/robot/manipulators/openarm/scripts/openarm_set_mit_mode.py --channel can0 --fd
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed")

RID_CTRL_MODE = 10
MIT_MODE = 1
DEFAULT_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]


def write_ctrl_mode(bus: can.BusABC, send_id: int, fd: bool) -> bool:
    val = struct.pack("<I", MIT_MODE)  # little-endian uint32
    data = bytes(
        [send_id & 0xFF, (send_id >> 8) & 0xFF, 0x55, RID_CTRL_MODE, val[0], val[1], val[2], val[3]]
    )
    # Flush
    while bus.recv(0.0) is not None:
        pass
    bus.send(
        can.Message(
            arbitration_id=0x7FF, data=data, is_extended_id=False, is_fd=fd, bitrate_switch=fd
        )
    )
    # Wait for ack on 0x7FF (per openarm_can param response)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.2:
        msg = bus.recv(0.2 - (time.monotonic() - t0))
        if msg is None:
            break
        # Reply on 0x7FF: [id_lo, id_hi, 0x33|0x55, rid, value[0..3]]
        if msg.arbitration_id != 0x7FF or len(msg.data) < 8:
            continue
        if msg.data[2] not in (0x33, 0x55):
            continue
        if msg.data[0] != (send_id & 0xFF) or msg.data[1] != ((send_id >> 8) & 0xFF):
            continue  # ack from a different motor
        rid = msg.data[3]
        if rid == RID_CTRL_MODE:
            echoed = int(struct.unpack("<I", bytes(msg.data[4:8]))[0])
            print(
                f"  0x{send_id:02X}: ack  CTRL_MODE={echoed} "
                f"({'MIT' if echoed == MIT_MODE else f'code {echoed}'})"
            )
            return echoed == MIT_MODE
    print(f"  0x{send_id:02X}: no ack on 0x7FF")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--fd", action="store_true", help="Use CAN-FD (default: classical CAN)")
    ap.add_argument(
        "--id", type=lambda s: int(s, 0), default=None, help="Single send ID (default: all 8)"
    )
    args = ap.parse_args()

    fd = args.fd
    ids = [args.id] if args.id is not None else DEFAULT_IDS

    # Preflight: is the interface up?
    try:
        flags = int(open(f"/sys/class/net/{args.channel}/flags").read().strip(), 16)
    except OSError:
        print(f"ERROR: interface '{args.channel}' not found", file=sys.stderr)
        return 1
    if not (flags & 0x1):
        print(f"ERROR: SocketCAN interface '{args.channel}' is DOWN.", file=sys.stderr)
        print(
            f"  Run: sudo ./dimos/robot/manipulators/openarm/scripts/openarm_can_up.sh {args.channel}",
            file=sys.stderr,
        )
        return 1

    print(f"Opening {args.channel} ({'CAN-FD' if fd else 'classical'})")
    bus = can.Bus(interface="socketcan", channel=args.channel, fd=fd)
    try:
        ok = 0
        for i in ids:
            if write_ctrl_mode(bus, i, fd):
                ok += 1
            time.sleep(0.05)
        print(f"\n{ok}/{len(ids)} motors set to MIT mode.")
        return 0 if ok == len(ids) else 2
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
