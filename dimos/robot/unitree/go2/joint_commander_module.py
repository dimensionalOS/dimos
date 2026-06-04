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

"""Dummy stdin-driven joint commander for Go2 wholebody bring-up testing.

Publishes JointState on `joint_command` from typed commands. Pair with
`unitree-go2-wholebody-coordinator` so the coordinator's servo task
converts the positions into MotorCommandArray via wb_config PD gains.

Commands (typed at the prompt):
    arm             publish the current (local) target — first publish energizes
                    motors via the coordinator's servo task; nothing is sent
                    before this
    stand           set local target to Go2 standing pose AND publish
    lay             set local target to folded pose AND publish (UNVERIFIED)
    zero            set local target to all zeros AND publish (CALIBRATION
                    pose, legs splay straight — DANGEROUS on the ground)
    set <i> <q>     set joint index i ∈ [0,11] to q (rad) AND publish
    nudge <i> <dq>  add dq (rad) to joint i's current target AND publish
                    (use small dq like 0.05 for safe joint-by-joint testing)
    pose <q0..q11>  set all 12 joints from explicit values AND publish
    show            print the current local target WITHOUT publishing
    help            list commands
    quit            stop the input thread (Module keeps running)

Safety: ``start()`` does NOT publish anything — the dog stays limp until
you type ``arm`` (or any pose / set / nudge / pose command). Use ``show``
to inspect the local target before publishing.

Pose preset sources:
    stand: mujoco-menagerie unitree_go2/go2.xml `home` keyframe
           (qpos per leg: hip=0, thigh=0.9, calf=-1.8)
    lay:   GUESSED — no authoritative source in this repo; sanity-check
           against Unitree SDK examples before trusting on hardware
    zero:  trivial; **not a safe stand-on-ground pose** — calibration only
"""

from __future__ import annotations

import threading
import time
from typing import Any

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import make_quadruped_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 12

# Joint order matches make_quadruped_joints("go2") and the Module's wire
# layout: [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
#          RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf].
GO2_JOINT_NAMES: list[str] = make_quadruped_joints("go2")
assert len(GO2_JOINT_NAMES) == _NUM_MOTORS

# Per-leg presets (hip, thigh, calf). Same pose applied to all four legs.
# Source: mujoco-menagerie unitree_go2/go2.xml `home` keyframe.
_STAND_LEG = (0.0, 0.9, -1.8)
# UNVERIFIED — no authoritative source for a sat/laid Go2 pose in this repo.
# Sanity-check against the Unitree SDK examples before trusting on hardware.
_LAY_LEG = (0.0, 1.36, -2.7)
_ZERO_LEG = (0.0, 0.0, 0.0)


def _broadcast_leg(leg: tuple[float, float, float]) -> list[float]:
    """Apply a (hip, thigh, calf) triple to all 4 legs in canonical order."""
    return [leg[i % 3] for i in range(_NUM_MOTORS)]


_PRESETS: dict[str, list[float]] = {
    "stand": _broadcast_leg(_STAND_LEG),
    "lay": _broadcast_leg(_LAY_LEG),
    "zero": _broadcast_leg(_ZERO_LEG),
}


class Go2JointCommanderConfig(ModuleConfig):
    initial_pose: str = Field(default="zero")  # one of _PRESETS keys
    prompt: str = Field(default="go2> ")
    frame_id: str = Field(default="go2_base")


class Go2JointCommanderModule(Module):
    """Stdin-driven publisher of JointState targets for Go2 wholebody control.

    Pairs with the coordinator's `joint_command` In port via LCM transport.
    """

    config: Go2JointCommanderConfig

    joint_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._lock = threading.Lock()
        self._target: list[float] = list(_PRESETS["zero"])
        self._stop_event = threading.Event()
        self._input_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.initial_pose not in _PRESETS:
            logger.warning(
                f"Unknown initial_pose {self.config.initial_pose!r}; "
                f"falling back to 'zero'. Valid: {list(_PRESETS)}"
            )
            initial = "zero"
        else:
            initial = self.config.initial_pose

        with self._lock:
            self._target = list(_PRESETS[initial])

        logger.info(
            f"Go2JointCommanderModule started — local target set to '{initial}' "
            f"but NOT published (motors stay limp). Type 'arm' to publish the "
            f"first target, or 'help' for the full command list."
        )

        self._input_thread = threading.Thread(
            target=self._input_loop, name="go2-commander-stdin", daemon=True
        )
        self._input_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._input_thread is not None and self._input_thread.is_alive():
            # Daemon thread is blocked in input(); join with a short timeout
            # and let process shutdown drop it.
            self._input_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._input_thread = None
        logger.info("Go2JointCommanderModule stopped")
        super().stop()

    def _input_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = input(self.config.prompt)
            except (EOFError, KeyboardInterrupt):
                logger.info("Commander input closed")
                return

            line = line.strip()
            if not line:
                continue

            try:
                self._handle_command(line)
            except Exception as e:
                logger.error(f"Command {line!r} failed: {e}")

    def _handle_command(self, line: str) -> None:
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            logger.info("quit — input thread exiting")
            self._stop_event.set()
            return

        if cmd in ("help", "?"):
            print(
                "Commands:\n"
                "  arm              publish the current local target (FIRST publish\n"
                "                   energizes motors; nothing published before this)\n"
                "  stand            menagerie home pose (hip=0, thigh=0.9, calf=-1.8)\n"
                "  lay              folded pose (UNVERIFIED — sanity-check first)\n"
                "  zero             all joints to 0 rad (CALIBRATION pose; dangerous\n"
                "                   on the ground — legs splay flat)\n"
                "  set <i> <q>      set joint i in [0,11] to absolute q rad\n"
                "  nudge <i> <dq>   add dq rad to joint i's current target\n"
                "                   (use small dq like 0.05 for safe testing)\n"
                "  pose <q0..q11>   set all 12 joints explicitly\n"
                "  show             print current local target (no publish)\n"
                "  quit             stop input thread\n"
            )
            return

        if cmd == "arm":
            self._publish_target()
            print("  armed — first target published")
            return

        if cmd in _PRESETS:
            with self._lock:
                self._target = list(_PRESETS[cmd])
            self._publish_target()
            return

        if cmd == "show":
            with self._lock:
                tgt = list(self._target)
            for i, (name, q) in enumerate(zip(GO2_JOINT_NAMES, tgt, strict=True)):
                print(f"  [{i:2d}] {name:20s} {q:+.4f}")
            return

        if cmd == "set":
            if len(parts) != 3:
                print("usage: set <joint_index 0..11> <value_rad>")
                return
            idx = int(parts[1])
            q = float(parts[2])
            if not 0 <= idx < _NUM_MOTORS:
                print(f"joint index {idx} out of range [0, {_NUM_MOTORS})")
                return
            with self._lock:
                self._target[idx] = q
                new_q = self._target[idx]
            print(f"  {GO2_JOINT_NAMES[idx]} → {new_q:+.4f} rad")
            self._publish_target()
            return

        if cmd == "nudge":
            if len(parts) != 3:
                print("usage: nudge <joint_index 0..11> <delta_rad>")
                return
            idx = int(parts[1])
            dq = float(parts[2])
            if not 0 <= idx < _NUM_MOTORS:
                print(f"joint index {idx} out of range [0, {_NUM_MOTORS})")
                return
            with self._lock:
                self._target[idx] += dq
                new_q = self._target[idx]
            print(f"  {GO2_JOINT_NAMES[idx]} {dq:+.4f} → {new_q:+.4f} rad")
            self._publish_target()
            return

        if cmd == "pose":
            if len(parts) != 1 + _NUM_MOTORS:
                print(f"usage: pose <q0> <q1> ... <q{_NUM_MOTORS - 1}>")
                return
            values = [float(p) for p in parts[1:]]
            with self._lock:
                self._target = values
            self._publish_target()
            return

        print(f"unknown command {cmd!r}; type 'help'")

    def _publish_target(self) -> None:
        with self._lock:
            positions = list(self._target)
        msg = JointState(
            ts=time.time(),
            frame_id=self.config.frame_id,
            name=GO2_JOINT_NAMES,
            position=positions,
            velocity=[0.0] * _NUM_MOTORS,
            effort=[0.0] * _NUM_MOTORS,
        )
        self.joint_command.publish(msg)
        logger.debug(f"Published target: {positions}")


__all__ = [
    "GO2_JOINT_NAMES",
    "Go2JointCommanderConfig",
    "Go2JointCommanderModule",
]
