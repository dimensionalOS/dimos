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

"""Slow, supervised return of the R1 Lite arms to a captured home pose.

Runs against a live r1lite blueprint (hardware or sim) the same way
preflight.py does, talking only to dimos streams — never to vendor
topics — so every safety layer (arming gate, sole-writer, servo
arbitration) stays in the loop:

    capture   save the CURRENT arm pose as home (run it while the arms
              are folded in their natural resting state)
    status    show current-vs-home deltas; moves nothing
    go        stream a slow joint-space ramp to home via the servo task

The ramp is generated here because the servo task is deliberately
pass-through: dense 50 Hz targets, smoothstep-eased, capped at a slow
joint speed. Ctrl-C at any moment simply stops the stream — the servo
task times out and the arms hold where they are. While the connection
is DISARMED the whole run is inert at the actuators, which makes
`go` while disarmed a harmless full rehearsal.

Do not run `go` while a teleop hand is engaged: the teleop tasks
outrank the servo task and the two would fight for the same joints.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1lite import config as cfg

ARM_JOINTS: list[str] = list(cfg.R1LITE_ARM_JOINTS)

DEFAULT_HOME_FILE = "/app/logs/r1lite_home_pose.json"
# State sources, in preference order, with the joint_command prefix that
# belongs to the same blueprint: the hardware blueprint maps the
# connection's motor_states to /r1lite/*, while the sim blueprint keeps
# default names and exposes pose via the coordinator's own state stream.
STATE_SOURCES = (
    ("/r1lite/motor_states", "/r1lite"),
    ("/motor_states", ""),
    ("/coordinator_joint_state", ""),
)
JOINT_COMMAND_STREAM = "joint_command"

STATE_WAIT_SEC = 5.0  # blueprint must be publishing motor_states
RATE_HZ = 50.0
JOINT_SPEED_RAD_S = 0.15  # slow: ~8.6 deg/s at the fastest joint
MIN_DURATION_SEC = 4.0
LARGE_DELTA_RAD = 2.6  # ~150 deg on one joint: refuse without --yes-large
SETTLE_SEC = 1.0
DONE_TOL_RAD = 0.03


class ArmStateReader:
    """Latest arm joint positions from the blueprint's motor_states stream.

    Listens on every known namespace and remembers which one delivered,
    so commands go back to the same blueprint that owns the state.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._fresh = threading.Event()
        self.prefix: str | None = None
        self._transports = []
        for topic, prefix in STATE_SOURCES:
            transport = LCMTransport(topic, JointState)
            transport.subscribe(lambda msg, _meta=None, p=prefix: self._on_msg(msg, p))
            self._transports.append(transport)

    def _on_msg(self, msg: JointState, prefix: str) -> None:
        got = dict(zip(list(msg.name), list(msg.position), strict=False))
        arm = {j: got[j] for j in ARM_JOINTS if j in got}
        if len(arm) != len(ARM_JOINTS):
            return  # not the whole-body state frame we need
        with self._lock:
            # Hardware source wins if both are alive (prefer "/r1lite").
            if self.prefix is None or prefix == "/r1lite" or self.prefix != "/r1lite":
                self._positions = arm
                self.prefix = prefix
        self._fresh.set()

    def wait_for_pose(self, timeout: float = STATE_WAIT_SEC) -> dict[str, float]:
        if not self._fresh.wait(timeout):
            sources = ", ".join(t for t, _ in STATE_SOURCES)
            sys.exit(
                f"FAIL no arm state on any of [{sources}] within {timeout:.0f}s "
                "— is the r1lite blueprint running?"
            )
        with self._lock:
            return dict(self._positions)


def _fmt_deltas(current: dict[str, float], home: dict[str, float]) -> tuple[str, float]:
    lines = []
    max_delta = 0.0
    for j in ARM_JOINTS:
        d = home[j] - current[j]
        max_delta = max(max_delta, abs(d))
        lines.append(
            f"  {j:28s} {current[j]:+7.3f} -> {home[j]:+7.3f}  ({math.degrees(d):+6.1f} deg)"
        )
    return "\n".join(lines), max_delta


def cmd_capture(args: argparse.Namespace) -> None:
    reader = ArmStateReader()
    pose = reader.wait_for_pose()
    path = Path(args.file)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps({"captured_at": time.strftime("%Y-%m-%d %H:%M:%S"), "pose": pose}, indent=2)
    )
    print(f"OK   home pose captured ({len(pose)} joints) -> {path}")
    for j in ARM_JOINTS:
        print(f"  {j:28s} {pose[j]:+7.3f} rad ({math.degrees(pose[j]):+6.1f} deg)")


def _load_home(path_str: str) -> dict[str, float]:
    path = Path(path_str)
    if not path.exists():
        sys.exit(f"FAIL no home pose file at {path} — run `capture` first (arms folded at home)")
    pose = json.loads(path.read_text())["pose"]
    missing = [j for j in ARM_JOINTS if j not in pose]
    if missing:
        sys.exit(f"FAIL home file missing joints: {missing}")
    return {j: float(pose[j]) for j in ARM_JOINTS}


def cmd_status(args: argparse.Namespace) -> None:
    home = _load_home(args.file)
    reader = ArmStateReader()
    current = reader.wait_for_pose()
    table, max_delta = _fmt_deltas(current, home)
    print(table)
    print(f"max joint delta: {math.degrees(max_delta):.1f} deg")


def cmd_go(args: argparse.Namespace) -> None:
    home = _load_home(args.file)
    reader = ArmStateReader()
    start = reader.wait_for_pose()
    table, max_delta = _fmt_deltas(start, home)
    print("planned motion (current -> home):")
    print(table)

    if max_delta < DONE_TOL_RAD:
        print("OK   already at home (all joints within tolerance); nothing to do")
        return
    if max_delta > LARGE_DELTA_RAD and not args.yes_large:
        sys.exit(
            f"FAIL largest joint delta is {math.degrees(max_delta):.0f} deg (> "
            f"{math.degrees(LARGE_DELTA_RAD):.0f}). Check the pose and the home file; "
            "re-run with --yes-large only if this motion is genuinely intended."
        )

    duration = max(MIN_DURATION_SEC, max_delta / JOINT_SPEED_RAD_S)
    print(
        f"ramp: {duration:.1f}s at <= {math.degrees(JOINT_SPEED_RAD_S):.1f} deg/s, {RATE_HZ:.0f} Hz stream"
    )
    print("hands off the engage buttons; Ctrl-C stops the stream and the arms hold.")
    if not args.yes:
        answer = input("type 'GO HOME' to move: ")
        if answer.strip() != "GO HOME":
            sys.exit("aborted — nothing sent")

    pub = LCMTransport(f"{reader.prefix}/{JOINT_COMMAND_STREAM}", JointState)
    t0 = time.monotonic()
    period = 1.0 / RATE_HZ
    try:
        while True:
            t = time.monotonic() - t0
            s = min(1.0, t / duration)
            s = s * s * (3.0 - 2.0 * s)  # smoothstep: zero velocity at both ends
            positions = [start[j] + s * (home[j] - start[j]) for j in ARM_JOINTS]
            pub.publish(JointState(name=ARM_JOINTS, position=positions))
            if t >= duration + SETTLE_SEC:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nSTOPPED by operator — servo stream ended, arms hold their current pose")
        return

    time.sleep(0.3)
    final = reader.wait_for_pose()
    _, residual = _fmt_deltas(final, home)
    verdict = "OK  " if residual < DONE_TOL_RAD else "WARN"
    print(f"{verdict} home ramp complete; residual max error {math.degrees(residual):.2f} deg")
    if residual >= DONE_TOL_RAD:
        print("     (residual above tolerance — arms may be disarmed, obstructed, or preempted)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--file", default=DEFAULT_HOME_FILE, help=f"home pose file (default {DEFAULT_HOME_FILE})"
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("capture", help="save the current arm pose as home")
    sub.add_parser("status", help="show current-vs-home deltas, move nothing")
    go = sub.add_parser("go", help="slowly move the arms to the saved home pose")
    go.add_argument("--yes", action="store_true", help="skip the interactive confirm")
    go.add_argument(
        "--yes-large", action="store_true", help="allow deltas above the large-motion guard"
    )
    args = parser.parse_args()
    {"capture": cmd_capture, "status": cmd_status, "go": cmd_go}[args.cmd](args)


if __name__ == "__main__":
    main()
