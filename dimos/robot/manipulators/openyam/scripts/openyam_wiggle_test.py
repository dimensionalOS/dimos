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

"""First-motion smoke test: wiggle one OpenYAM joint by a few degrees.

Sequence: connect -> enable -> actively hold the current pose -> ramp one
joint (wrist by default) out by --delta rad -> verify the encoder tracked
-> ramp back -> stop -> disable. Streams position commands at 100 Hz the
whole time and aborts (torque off) if any joint diverges from its command
by more than --abort-error rad.

SAFETY: watch the arm, keep hands clear, keep the 24V kill within reach.
Disabling drops all torque - the arm should start from a stable rest pose.

Usage:
    python openyam_wiggle_test.py --channel gs_usb:0                # macOS
    python openyam_wiggle_test.py --channel can0 --joint 5 --delta 0.05
"""

from __future__ import annotations

import argparse
import sys
import time

from dimos.hardware.manipulators.openyam.adapter import OpenYamAdapter

RATE_HZ = 100.0


def stream(
    adapter: OpenYamAdapter,
    target_fn,
    duration_s: float,
    abort_error: float,
) -> list[float]:
    """Stream targets from target_fn(progress in [0,1]) at RATE_HZ."""
    steps = max(1, int(duration_s * RATE_HZ))
    q_cmd: list[float] = []
    for i in range(steps):
        q_cmd = target_fn((i + 1) / steps)
        if not adapter.write_joint_positions(q_cmd):
            raise RuntimeError("write_joint_positions returned False")
        time.sleep(1.0 / RATE_HZ)
        q_now = adapter.read_joint_positions()
        worst = max(abs(a - b) for a, b in zip(q_now, q_cmd, strict=True))
        if worst > abort_error:
            raise RuntimeError(
                f"tracking error {worst:.3f} rad exceeds {abort_error} rad at step {i}"
            )
    return q_cmd


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--channel", default="gs_usb:0")
    ap.add_argument("--joint", type=int, default=6, help="joint to move, 1-6 (default: wrist)")
    ap.add_argument("--delta", type=float, default=0.08, help="excursion in rad (default ~4.6 deg)")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold before/at excursion")
    ap.add_argument("--ramp", type=float, default=1.5, help="seconds for each ramp")
    ap.add_argument(
        "--abort-error",
        type=float,
        default=0.3,
        help="abort (torque off) if any joint lags its command by this many rad",
    )
    args = ap.parse_args()
    if not 1 <= args.joint <= 6:
        sys.exit("--joint must be 1..6")
    if abs(args.delta) > 0.3:
        sys.exit("--delta capped at 0.3 rad for this smoke test")
    j = args.joint - 1

    adapter = OpenYamAdapter(address=args.channel)
    if not adapter.connect():
        sys.exit("connect failed")
    try:
        if not adapter.write_enable(True):
            sys.exit("enable failed")
        time.sleep(0.05)
        q0 = adapter.read_joint_positions()
        print("start pose:", [f"{q:+.3f}" for q in q0])

        def toward(delta: float, base: list[float]):
            def _fn(progress: float) -> list[float]:
                q = list(base)
                q[j] = base[j] + delta * progress
                return q

            return _fn

        print(f"holding current pose {args.hold:.1f}s...")
        stream(adapter, toward(0.0, q0), args.hold, args.abort_error)

        print(f"ramping joint {args.joint} by {args.delta:+.3f} rad over {args.ramp:.1f}s...")
        stream(adapter, toward(args.delta, q0), args.ramp, args.abort_error)
        stream(adapter, toward(args.delta, q0), args.hold, args.abort_error)

        q_peak = adapter.read_joint_positions()
        moved = q_peak[j] - q0[j]
        print(f"peak pose:  {[f'{q:+.3f}' for q in q_peak]}")
        print(f"joint {args.joint} moved {moved:+.3f} rad (commanded {args.delta:+.3f})")

        print("ramping back...")
        q_out = list(q0)
        q_out[j] = q0[j] + args.delta

        def back(progress: float) -> list[float]:
            q = list(q_out)
            q[j] = q_out[j] - args.delta * progress
            return q

        stream(adapter, back, args.ramp, args.abort_error)
        adapter.write_stop()
        time.sleep(0.2)

        q_end = adapter.read_joint_positions()
        print("end pose:  ", [f"{q:+.3f}" for q in q_end])
        ok = abs(moved - args.delta) < 0.05 and abs(q_end[j] - q0[j]) < 0.05
        print("RESULT:", "PASS" if ok else "CHECK TRACKING (see numbers above)")
        return 0 if ok else 2
    finally:
        # Always drop torque on the way out, even after an abort.
        try:
            adapter.write_enable(False)
        finally:
            adapter.disconnect()
            print("disabled and disconnected")


if __name__ == "__main__":
    sys.exit(main())
