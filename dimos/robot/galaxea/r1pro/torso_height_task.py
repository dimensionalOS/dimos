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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Joystick lift intent -> the R1Pro's two lower pitch joints.

The default URDF has two 0.35 m links and parallel pitch axes; the 2025
hardware uses configurable 0.40/0.30 m links. This planar
2-link solve changes height while retaining the initial fore/aft position.
It controls neither the top pitch/yaw nor the arms; their IK uses feedback.
"""

from __future__ import annotations

import math
import threading
from typing import Any

from pydantic import Field, model_validator

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.service.spec import BaseConfig

TORSO_LIFT_JOINTS = ("r1pro/torso_joint1", "r1pro/torso_joint2")


class TorsoHeightParams(BaseConfig):
    link_lengths: tuple[float, float] = (0.35, 0.35)
    max_height_rate: float = Field(default=0.03, gt=0, allow_inf_nan=False)
    max_joint_velocity: float = Field(default=0.15, gt=0, allow_inf_nan=False)
    min_height: float = Field(default=0.45, gt=0, allow_inf_nan=False)
    timeout: float = Field(default=0.2, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _validate_geometry(self) -> TorsoHeightParams:
        if not all(math.isfinite(v) and v > 0 for v in self.link_lengths):
            raise ValueError("torso link lengths must be finite and positive")
        if (
            not abs(self.link_lengths[0] - self.link_lengths[1])
            < self.min_height
            < sum(self.link_lengths)
        ):
            raise ValueError("min_height must be inside the torso reach")
        return self


def torso_xz(
    q: list[float], link_lengths: tuple[float, float] = (0.35, 0.35)
) -> tuple[float, float]:
    a, b = q
    l1, l2 = link_lengths
    return l1 * math.sin(a) + l2 * math.sin(a + b), l1 * math.cos(a) + l2 * math.cos(a + b)


class R1ProTorsoHeightTask(BaseControlTask):
    def __init__(
        self, name: str, priority: int, params: TorsoHeightParams, limits: list[tuple[float, float]]
    ) -> None:
        self._name = name
        self._priority = priority
        self._params = params
        self._limits = limits
        self._lock = threading.Lock()
        self._rate = 0.0
        self._received = -math.inf
        self._target: list[float] | None = None
        self._x = 0.0
        self._bend = -1.0
        self._estopped = False

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            frozenset(TORSO_LIFT_JOINTS), self._priority, ControlMode.SERVO_POSITION
        )

    def is_active(self) -> bool:
        return True

    def on_lift_rate(self, msg: Float32, t_now: float) -> bool:
        with self._lock:
            if self._estopped:
                return False
            value = float(msg.data)
            self._rate = (
                max(-self._params.max_height_rate, min(self._params.max_height_rate, value))
                if math.isfinite(value)
                else 0.0
            )
            self._received = t_now
            return math.isfinite(value)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        measured = [state.joints.get_position(j) for j in TORSO_LIFT_JOINTS]
        with self._lock:
            if any(q is None or not math.isfinite(q) for q in measured):
                self._rate, self._target = 0.0, None
                return None
            q = [float(v) for v in measured if v is not None]
            if (
                self._estopped
                or not self._rate
                or state.t_now - self._received >= self._params.timeout
            ):
                if self._target is None:
                    return None
                self._target = None
                target = q  # Stop at feedback, not a future integrated target.
            else:
                if (
                    self._target is None
                    or max(abs(a - b) for a, b in zip(self._target, q, strict=True)) > 0.05
                ):
                    self._target = q
                    self._x, _ = torso_xz(q, self._params.link_lengths)
                    self._bend = 1.0 if q[1] > 0.0 else -1.0
                current = self._target
                l1, l2 = self._params.link_lengths
                _, height = torso_xz(current, self._params.link_lengths)
                ceiling = math.sqrt(max(0.0, (l1 + l2) ** 2 - self._x**2))
                if not self._params.min_height <= height <= ceiling + 1e-6:
                    self._rate, self._target = 0.0, None
                    return None
                dt = max(0.0, min(state.dt, 0.05))
                height = max(self._params.min_height, min(ceiling, height + self._rate * dt))
                cosine = (self._x**2 + height**2 - l1**2 - l2**2) / (2 * l1 * l2)
                b = self._bend * math.acos(max(-1.0, min(1.0, cosine)))
                a = math.atan2(self._x, height) - math.atan2(
                    l2 * math.sin(b), l1 + l2 * math.cos(b)
                )
                delta = [a - current[0], b - current[1]]
                scale = min(
                    1.0, self._params.max_joint_velocity * dt / max(max(map(abs, delta)), 1e-12)
                )
                target = [p + scale * d for p, d in zip(current, delta, strict=True)]
                if any(not lo <= p <= hi for p, (lo, hi) in zip(target, self._limits, strict=True)):
                    self._rate, self._target = 0.0, None
                    return None
                self._target = target
        return JointCommandOutput(
            joint_names=list(TORSO_LIFT_JOINTS), positions=target, mode=ControlMode.SERVO_POSITION
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._lock:
            self._rate, self._target = 0.0, None

    def set_estop(self, estopped: bool) -> None:
        with self._lock:
            self._estopped = estopped
            self._rate, self._target = 0.0, None


def create_task(cfg: Any, hardware: Any, connections: Any) -> R1ProTorsoHeightTask:
    if tuple(cfg.joint_names) != TORSO_LIFT_JOINTS:
        raise ValueError("R1Pro height task must own only torso_joint1/2, in that order")
    source = connections["r1pro"]
    limits = [source.position_limit(j) for j in TORSO_LIFT_JOINTS]
    if any(limit.lo is None or limit.hi is None for limit in limits):
        raise ValueError("R1Pro height task requires bounded position joints")
    return R1ProTorsoHeightTask(
        cfg.name,
        cfg.priority,
        TorsoHeightParams.model_validate(cfg.params),
        [(float(limit.lo), float(limit.hi)) for limit in limits],
    )
