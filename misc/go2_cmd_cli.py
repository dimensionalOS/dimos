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

"""External CLI for sending joint targets to the Go2 wholebody coordinator.

Pair with ``unitree-go2-wholebody-coordinator`` running in another terminal.
This script publishes ``JointState`` to ``/go2/joint_command``; the
coordinator's servo task picks it up and drives motors via the
TransportWholeBodyAdapter → Go2WholeBodyConnection → rt/lowcmd chain.

Why external (not a Module): dimos worker processes don't reliably
attach stdin, so the embedded ``Go2JointCommanderModule`` could not
read terminal input. Running this script in your own shell sidesteps
that entirely.

Commands at the prompt:
    sync               read current actual positions via RPC, set as
                       local target (use BEFORE 'arm' so first publish
                       has zero error → no motion, no torque)
    arm                publish the current local target ONCE
    show               print the current local target (no publish)
    bounds             print the per-joint envelope (sit ↔ stand + margin)
    stand              load 'standing' preset → publish
    sit                load 'sitting' preset → publish
    lerp <alpha>       linear interpolate between sit (0) and stand (1)
                       → publish. e.g. 'lerp 0.3' = 30% standing.
                       Useful for incremental crouch-to-stand testing.
    set <i> <q>        set joint i absolute, clamped → publish
    nudge <i> <dq>     add dq to joint i, clamped → publish
    pose <q0..q11>     full 12-value target, clamped → publish
    help               list commands
    quit               exit (does NOT relax motors — Ctrl-C the
                       coordinator to send the safe-stop LowCmd)

The 'standing' and 'sitting' presets are Unitree's official _targetPos_1
and _targetPos_2 from `example/go2/low_level/go2_stand_example.py` in
unitree_sdk2_python. These are the hand-tuned poses the Unitree firmware
uses for its own stand-up demo, so they're known-good on real Go2 hardware.
Envelope = [min(stand_i, sit_i) - margin, max(stand_i, sit_i) + margin]
per joint, with a default margin of 0.05 rad (~2.9°).
"""

from __future__ import annotations

import sys
import threading
import time

from dimos import Dimos
from dimos.control.components import make_quadruped_joints
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState

_NUM_MOTORS = 12
_JOINT_NAMES = make_quadruped_joints("go2")  # canonical FR/FL/RR/RL × hip/thigh/calf
assert len(_JOINT_NAMES) == _NUM_MOTORS

# Captured live from this Go2 via rt/lowstate while sport mode held each pose.
# _STANDING was recorded while Sport's StandUp had the robot upright at its
# usual operator-facing height (~0.32 m base). _SITTING was recorded after
# StandDown, robot crouched on hindquarters. Both are known-stable holds.
#
# These are MORE TRUSTWORTHY than Unitree's go2_stand_example.cpp constants:
# their `_targetPos_1` (the example calls it "stand") is actually a low
# pre-walk crouch with the knees deeply folded - taller-than-_targetPos_2
# but much shorter than what the app's StandUp produces. Tested 2026-06-08.
#
# Index order matches _JOINT_NAMES exactly:
#   0:FR_hip 1:FR_thigh 2:FR_calf 3:FL_hip 4:FL_thigh 5:FL_calf
#   6:RR_hip 7:RR_thigh 8:RR_calf 9:RL_hip 10:RL_thigh 11:RL_calf
_STANDING = [
    -0.0442,
    +0.6880,
    -1.4558,  # FR
    +0.0240,
    +0.6984,
    -1.5437,  # FL
    -0.0105,
    +0.7103,
    -1.4343,  # RR
    +0.1227,
    +0.6420,
    -1.4284,  # RL
]

_SITTING = [
    -0.0867,
    +1.2304,
    -2.7534,  # FR
    +0.0390,
    +1.2525,
    -2.7704,  # FL
    -0.3335,
    +1.2953,
    -2.7901,  # RR  (rear hips splay outward at rest)
    +0.3887,
    +1.2642,
    -2.7634,  # RL
]

# Per-joint envelope min/max from the two captures.
# 0.05 rad ≈ 2.9° on each side widens the tight front-hip envelope just
# enough to be useful without straying into territory the dog hasn't shown.
_SAFETY_MARGIN_RAD = 0.05

_ENVELOPE_MIN = [min(_STANDING[i], _SITTING[i]) - _SAFETY_MARGIN_RAD for i in range(_NUM_MOTORS)]
_ENVELOPE_MAX = [max(_STANDING[i], _SITTING[i]) + _SAFETY_MARGIN_RAD for i in range(_NUM_MOTORS)]

# Heartbeat must beat the servo task's stale-target timeout (~500 ms).
# 10 Hz gives ~100 ms between publishes — comfortable margin.
_HEARTBEAT_HZ = 10.0

# Max per-joint movement speed when ramping toward a new target. The CLI's
# `lerp`/`stand`/`sit`/`pose` commands set a NEW target instantly, but the
# heartbeat thread interpolates the *published* value toward it at this rate
# so the robot moves smoothly instead of lurching. 0.8 rad/s ≈ 46°/s — a
# full sit↔stand transition takes ~1.7s of smooth motion. Bump higher for
# snappier moves, lower for gentler.
_RAMP_RAD_PER_S = 0.8


def _clamp(target: list[float]) -> tuple[list[float], list[int]]:
    """Clamp each entry to its envelope. Return (clamped, list_of_clamped_indices)."""
    out: list[float] = []
    clamped_idx: list[int] = []
    for i, q in enumerate(target):
        lo, hi = _ENVELOPE_MIN[i], _ENVELOPE_MAX[i]
        if q < lo:
            out.append(lo)
            clamped_idx.append(i)
        elif q > hi:
            out.append(hi)
            clamped_idx.append(i)
        else:
            out.append(q)
    return out, clamped_idx


class CommanderCLI:
    def __init__(self) -> None:
        print("Connecting to dimos app...")
        self._app = Dimos.connect()
        self._publisher = LCMTransport("/go2/joint_command", JointState)
        self._target: list[float] | None = None  # None until first sync/stand/sit/pose
        # _published is what the heartbeat thread is currently sending. It
        # smoothly ramps toward _target at _RAMP_RAD_PER_S per joint, so a
        # `lerp` / `stand` / `sit` command sets a new goal but the actual
        # output evolves gradually instead of lurching.
        self._published: list[float] | None = None
        # _armed flips True on any publishing command (arm/stand/sit/lerp/set/nudge/pose).
        # The heartbeat thread only republishes when armed — so a bare 'sync' won't
        # start driving motors until the user explicitly publishes.
        self._armed = False
        self._lock = threading.Lock()
        self._stop_heartbeat = threading.Event()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="commander-heartbeat", daemon=True
        )
        self._heartbeat_thread.start()
        print("Connected.\n")
        print(
            f"Commander ready (heartbeat republishing at {_HEARTBEAT_HZ:.0f} Hz when armed). "
            "Try 'help' for commands. First step is usually 'sync'."
        )

    # ----- top-level loop -----
    def run(self) -> None:
        try:
            while True:
                try:
                    line = input("go2> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    return
                if not line:
                    continue
                try:
                    self._handle(line)
                except (EOFError, KeyboardInterrupt):
                    # quit/exit/q raise EOFError as a sentinel — let it bubble
                    print()
                    return
                except Exception as e:
                    print(f"  error: {e}")
        finally:
            self._stop_heartbeat.set()
            self._heartbeat_thread.join(timeout=1.0)
            print(
                "Heartbeat stopped. Servo task will time out within ~0.5s and "
                "motors will fall to safety-damped hold. For a full release, "
                "Ctrl-C the coordinator (it sends safe-stop LowCmd in stop())."
            )

    # ----- heartbeat -----
    def _heartbeat_loop(self) -> None:
        """Smoothly ramp the published target toward the goal at
        _HEARTBEAT_HZ. Each tick, each joint moves at most
        _RAMP_RAD_PER_S * period rad. Once we're within that step of the
        goal, the published value snaps to the goal and stays there.
        Only publishes after the user has armed."""
        period = 1.0 / _HEARTBEAT_HZ
        max_step = _RAMP_RAD_PER_S * period  # rad per heartbeat
        while not self._stop_heartbeat.wait(period):
            with self._lock:
                if not self._armed or self._target is None:
                    continue
                if self._published is None:
                    # First tick after arm — start from the goal so we don't
                    # ramp from a stale value.
                    self._published = list(self._target)
                goal = self._target
                pub = self._published
                # Step each joint toward the goal, capped at max_step.
                for i in range(_NUM_MOTORS):
                    delta = goal[i] - pub[i]
                    if abs(delta) <= max_step:
                        pub[i] = goal[i]
                    else:
                        pub[i] += max_step if delta > 0 else -max_step
                target_copy = list(pub)
            self._publish_target(target_copy)

    def _publish_target(self, target: list[float]) -> None:
        """Build a JointState and publish — no clamp, no print. Used by
        both the immediate publish path and the heartbeat thread."""
        msg = JointState(
            ts=time.time(),
            frame_id="go2_base",
            name=_JOINT_NAMES,
            position=target,
            velocity=[0.0] * _NUM_MOTORS,
            effort=[0.0] * _NUM_MOTORS,
        )
        self._publisher.publish(msg)

    # ----- command dispatch -----
    def _handle(self, line: str) -> None:
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            raise EOFError
        if cmd in ("help", "?"):
            self._help()
            return
        if cmd == "sync":
            self._sync()
            return
        if cmd == "show":
            self._show()
            return
        if cmd == "bounds":
            self._bounds()
            return
        if cmd == "arm":
            self._arm()
            return
        if cmd == "stand":
            self._target = list(_STANDING)
            self._publish_with_clamp_report("loaded standing preset")
            return
        if cmd == "sit":
            self._target = list(_SITTING)
            self._publish_with_clamp_report("loaded sitting preset")
            return
        if cmd == "lerp":
            if len(parts) != 2:
                print("  usage: lerp <alpha>   (0=sit, 1=stand, 0.3=30% standing)")
                return
            alpha = float(parts[1])
            self._target = [
                (1.0 - alpha) * _SITTING[i] + alpha * _STANDING[i] for i in range(_NUM_MOTORS)
            ]
            self._publish_with_clamp_report(f"lerp alpha={alpha:.3f} (0=sit, 1=stand)")
            return
        if cmd == "set":
            if len(parts) != 3:
                print("  usage: set <joint_index 0..11> <value_rad>")
                return
            self._require_target("set")
            idx, q = int(parts[1]), float(parts[2])
            self._range_check(idx)
            assert self._target is not None
            self._target[idx] = q
            self._publish_with_clamp_report(f"set {_JOINT_NAMES[idx]} = {q:+.4f}")
            return
        if cmd == "nudge":
            if len(parts) != 3:
                print("  usage: nudge <joint_index 0..11> <delta_rad>")
                return
            self._require_target("nudge")
            idx, dq = int(parts[1]), float(parts[2])
            self._range_check(idx)
            assert self._target is not None
            old = self._target[idx]
            new = old + dq
            self._target[idx] = new
            self._publish_with_clamp_report(
                f"nudge {_JOINT_NAMES[idx]} {old:+.4f} {dq:+.4f} = {new:+.4f}"
            )
            return
        if cmd == "pose":
            if len(parts) != 1 + _NUM_MOTORS:
                print(f"  usage: pose <q0> <q1> ... <q{_NUM_MOTORS - 1}>")
                return
            values = [float(p) for p in parts[1:]]
            self._target = values
            self._publish_with_clamp_report("loaded full pose")
            return

        print(f"  unknown command {cmd!r}; type 'help'")

    # ----- handlers -----
    def _help(self) -> None:
        print(
            "Commands:\n"
            "  sync             read live joint positions, set as local target\n"
            "  arm              publish current local target ONCE\n"
            "  show             print local target (no publish)\n"
            "  bounds           print sit↔stand envelope per joint\n"
            "  stand            load standing preset and publish\n"
            "  sit              load sitting preset and publish\n"
            "  lerp <alpha>     0=sit, 1=stand, 0.3=30% standing\n"
            "  set <i> <q>      set joint i absolute, clamped, publish\n"
            "  nudge <i> <dq>   add dq to joint i, clamped, publish\n"
            "  pose <q0..q11>   set all 12 joints, clamped, publish\n"
            "  quit             exit (does NOT relax motors)\n"
        )

    def _sync(self) -> None:
        positions = self._app.ControlCoordinator.get_joint_positions()
        if not positions:
            print("  no joints reported — is the coordinator running?")
            return
        # Build a 12-value list in canonical order.
        with self._lock:
            self._target = [float(positions.get(name, 0.0)) for name in _JOINT_NAMES]
            # Drop the stale ramp state so the next heartbeat re-seeds from
            # this fresh target (avoids ramping from a half-stepped value
            # that no longer matches the robot's actual pose).
            self._published = None
        print("  synced local target to current actual pose")
        self._print_target(self._target)

    def _show(self) -> None:
        if self._target is None:
            print("  no local target yet — try 'sync', 'stand', 'sit', or 'pose'")
            return
        self._print_target(self._target)

    def _bounds(self) -> None:
        print(f"  envelope (sit↔stand + ±{_SAFETY_MARGIN_RAD:.3f} rad):")
        for i, name in enumerate(_JOINT_NAMES):
            lo, hi = _ENVELOPE_MIN[i], _ENVELOPE_MAX[i]
            print(f"    [{i:2d}] {name:14s}  [{lo:+.4f}, {hi:+.4f}]  width {hi - lo:.4f}")

    def _arm(self) -> None:
        if self._target is None:
            print("  no local target — try 'sync' first (or 'stand'/'sit'/'pose')")
            return
        self._publish_with_clamp_report("armed — published current target")

    # ----- helpers -----
    def _require_target(self, cmd_name: str) -> None:
        if self._target is None:
            raise RuntimeError(f"'{cmd_name}' needs a starting target — run 'sync' first")

    @staticmethod
    def _range_check(idx: int) -> None:
        if not 0 <= idx < _NUM_MOTORS:
            raise ValueError(f"joint index {idx} out of range [0, {_NUM_MOTORS})")

    def _publish_with_clamp_report(self, note: str) -> None:
        assert self._target is not None
        with self._lock:
            requested = list(self._target)
            clamped_target, clamped_idx = _clamp(self._target)
            self._target = clamped_target  # keep local in sync with what was published
            self._armed = True  # enable heartbeat republishing
        if clamped_idx:
            print(f"  {note}; clamped {len(clamped_idx)} joint(s):")
            for i in clamped_idx:
                print(
                    f"    [{i:2d}] {_JOINT_NAMES[i]:14s}  "
                    f"requested {requested[i]:+.4f} → "
                    f"clamped {clamped_target[i]:+.4f}"
                )
        else:
            print(f"  {note}")
        self._publish_target(clamped_target)

    @staticmethod
    def _print_target(target: list[float]) -> None:
        for i, name in enumerate(_JOINT_NAMES):
            print(f"    [{i:2d}] {name:14s}  {target[i]:+.4f}")


def main() -> None:
    try:
        cli = CommanderCLI()
    except Exception as e:
        print(f"failed to connect: {e}")
        sys.exit(1)
    cli.run()


if __name__ == "__main__":
    main()
