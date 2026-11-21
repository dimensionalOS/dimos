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

"""A fake Damiao arm for the calibration tests, stepped by the test: no clock, no threads.

``tick`` advances first-order PD physics, ``(b + kd) dq = kp (target - q) + g_comp - g_true``
less each joint's Coulomb ``friction_nm`` (it moves only while that torque is larger), by one
reading period and hands the reading (quantized, with a little noise) to ``listener``.
A joint held stiffer than ``buzz_above_kp`` buzzes once a step of ``buzz_after_rad`` or more
excites it.
Its ``get_config``/``set_config`` follow the Damiao connection's: all or nothing, live gains
undone when no command arrives within the deadman, a save applied at once.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import copy
import math
import threading
from typing import Any

import numpy as np
import pinocchio as pin

from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.module import Operator, RunState
from dimos.msgs.sensor_msgs.JointState import JointState


class FakeArm:
    def __init__(
        self,
        arm: ArmModel,
        *,
        kp: float = 40.0,
        kd: float = 1.0,
        true_links: dict[str, LinkInertial] | None = None,
        period_s: float = 0.01,
        deadman_s: float = 0.1,
        quantum_rad: float = 1e-4,
        speed_noise_rad_s: float = 1e-2,
        damping: float = 2.0,
        buzz_above_kp: float = math.inf,
        buzz_after_rad: float = math.radians(2.0),
        friction_nm: Mapping[str, float] | None = None,
    ) -> None:
        self.arm, self.period_s, self.deadman_s = arm, period_s, deadman_s
        self.quantum, self.speed_noise, self.damping = quantum_rad, speed_noise_rad_s, damping
        self.buzz_above_kp, self.buzz_after_rad = buzz_above_kp, buzz_after_rad
        self.excited = np.zeros(arm.dof, dtype=bool)
        self.friction = np.asarray([(friction_nm or {}).get(j, 0.0) for j in arm.joint_names])
        self.truth = arm.model.copy()
        apply_inertial_overrides(self.truth, true_links or {})
        self.t, self.q = 0.0, arm.home.copy()
        self.target = self.q.copy()
        self.defaults = {joint: (kp, kd) for joint in arm.joint_names}
        self.live: dict[str, tuple[float, float]] = {}
        self.floating: dict[str, float] = {}
        self.file: dict[str, Any] = {}
        self.listener: Callable[[JointState], None] | None = None
        self.last_command: float | None = None
        self.commands: list[float] = []
        self.halts: list[float] = []
        self.refuse: str | None = None
        self.on_tick: Callable[[FakeArm], None] | None = None
        self._rng = np.random.default_rng(0)
        self._buzz = 1.0
        # Start at rest where the arm sags to under its gains.
        for _ in range(round(1.0 / period_s)):
            self.tick()
        self.t = 0.0

    def gains(self) -> dict[str, tuple[float, float]]:
        saved = {j: (g["kp"], g["kd"]) for j, g in self.file.get("gains", {}).items()}
        held = {**self.defaults, **saved, **self.live}
        return {**held, **{j: (0.0, kd) for j, kd in self.floating.items()}}

    def get_config(self) -> dict[str, Any]:
        return {
            self.arm.arm_id: {
                "gains": {j: {"kp": kp, "kd": kd} for j, (kp, kd) in self.gains().items()},
                "floating": sorted(self.floating),
                "links": copy.deepcopy(self.file.get("links", {})),
                "file": {
                    "path": "fake/calibration.json",
                    "sha256": "0" * 64,
                    "created": "2026-11-21T00:00:00+00:00",
                },
                "can_float": True,
                "deadman_timeout_s": self.deadman_s,
            }
        }

    def set_config(self, patch: dict[str, Any], save: bool = False) -> dict[str, Any]:
        errors = [f"{arm_id}: not this arm" for arm_id in patch if arm_id != self.arm.arm_id]
        entry = patch.get(self.arm.arm_id, {})
        errors += [f"{key}: saved only" for key in entry if not save and key != "gains"]
        for joint, gains in entry.get("gains", {}).items():
            if joint not in self.arm.joint_names or (save and gains["kp"] == 0.0):
                errors.append(f"gains.{joint}: refused")
        if self.refuse:
            errors.append(self.refuse)
        if errors:
            raise ValueError("set_config changed nothing: " + "; ".join(errors))
        if save:
            self.file = {
                **self.file,
                **{k: v for k, v in entry.items() if k not in ("gains", "checks")},
                "gains": {**self.file.get("gains", {}), **entry.get("gains", {})},
                "checks": {**self.file.get("checks", {}), **entry.get("checks", {})},
            }
            self.live.clear()
            self.floating.clear()
        for joint, gains in entry.get("gains", {}).items() if not save else ():
            if gains["kp"] == 0.0:
                self.floating[joint] = gains["kd"]
            else:
                if self.floating.pop(joint, None) is not None:
                    self.target[self.arm.joint_names.index(joint)] = self.q[
                        self.arm.joint_names.index(joint)
                    ]
                self.live[joint] = (gains["kp"], gains["kd"])
        if not save:
            self.last_command = self.t
        return self.get_config()

    def command(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position, strict=True):
            self.target[self.arm.joint_names.index(name.removeprefix("arm/"))] = value
        self.last_command = self.t
        self.commands.append(self.t)

    def _gravity(self, model: pin.Model, q: np.ndarray) -> np.ndarray:
        return np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q))

    def tick(self) -> None:
        if self.last_command is not None and self.t - self.last_command > self.deadman_s:
            self.live.clear()
            self.floating.clear()
            self.target = self.q.copy()
            self.last_command = None
            self.halts.append(self.t)
        gains = self.gains()
        kp = np.asarray([gains[j][0] for j in self.arm.joint_names])
        kd = np.asarray([gains[j][1] for j in self.arm.joint_names])
        compensated = self.arm.model.copy()
        apply_inertial_overrides(
            compensated,
            {
                name: LinkInertial(link["mass_kg"], tuple(link["com_m"]))
                for name, link in self.file.get("links", {}).items()
            },
        )
        torque = kp * (self.target - self.q) + self._gravity(compensated, self.q)
        net = torque - self._gravity(self.truth, self.q)
        net = np.sign(net) * np.maximum(np.abs(net) - self.friction, 0.0)
        dq = net / (self.damping + kd)
        self.q = self.q + dq * self.period_s
        self.t += self.period_s
        noise = self._rng.uniform(-0.3, 0.3, self.q.size) * self.quantum
        position = np.round((self.q + noise) / self.quantum) * self.quantum
        stiff = kp > self.buzz_above_kp
        self.excited = stiff & (
            self.excited | (np.abs(self.target - self.q) >= self.buzz_after_rad)
        )
        self._buzz = -self._buzz
        position += np.where(self.excited, self._buzz * 20.0 * self.quantum, 0.0)
        speed = dq + self._rng.uniform(-1.0, 1.0, self.q.size) * self.speed_noise
        if self.listener is not None:
            self.listener(
                JointState(
                    ts=self.t,
                    name=[f"arm/{joint}" for joint in self.arm.joint_names],
                    position=position.tolist(),
                    velocity=speed.tolist(),
                )
            )
        if self.on_tick is not None:
            self.on_tick(self)

    def run_until(self, event: threading.Event, max_ticks: int = 100_000) -> None:
        """Tick until ``event`` is set; a test that would tick forever fails instead."""
        for _ in range(max_ticks):
            if event.is_set():
                return
            self.tick()
        raise AssertionError(f"still waiting after {max_ticks} ticks at t={self.t:.2f} s")


class FakeOperator(Operator):
    """Answers each question as it is asked with ``pick(prompt, answers)``, and keeps what it saw.

    ``done`` is how the run ended: whether it was done, its summary, and whether the arm is home.
    """

    def __init__(self, pick: Callable[[str, list[str]], str] | None = None) -> None:
        super().__init__("fake", [])
        self.pick = pick or (lambda prompt, answers: answers[0])
        self.shown: list[str] = []
        self.asked: list[str] = []
        self.progressed: list[str] = []
        self.done: tuple[bool, str, bool] | None = None

    def show(self, text: str) -> None:
        super().show(text)
        self.shown.append(text)

    def progress(self, text: str) -> None:
        super().progress(text)
        self.progressed.append(text)

    def ask(self, prompt: str, answers: list[str]) -> threading.Event:
        answered = super().ask(prompt, answers)
        self.asked.append(prompt)
        assert self.answer(self.pick(prompt, answers)) == "ok"
        return answered

    def finish(self, state: RunState, summary: str, home: bool) -> None:
        super().finish(state, summary, home)
        self.done = (state == "done", summary, home)
