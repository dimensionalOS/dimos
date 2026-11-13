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

"""Input-independent motion limits, applied after per-joint arbitration."""

from __future__ import annotations

import math
import threading
from typing import Any, Literal, Protocol

from pydantic import Field, model_validator

from dimos.control.task import ControlMode
from dimos.protocol.service.spec import BaseConfig
from dimos.spec.utils import Spec


class MotionGroup(BaseConfig):
    """One operator-adjustable rate and its deployment ceiling.

    Position commands slew at this rate; velocity commands are capped at it.
    Effort commands are unaffected. Units describe the joint's velocity.
    """

    joints: tuple[str, ...]
    rate: float = Field(ge=0, allow_inf_nan=False)
    ceiling: float = Field(gt=0, allow_inf_nan=False)
    unit: Literal["rad/s", "m/s"] = "rad/s"
    label: str = ""
    coordinated: bool = False

    @model_validator(mode="after")
    def validate_group(self) -> MotionGroup:
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise ValueError("motion groups need distinct joint names")
        if self.rate > self.ceiling:
            raise ValueError("motion rate exceeds its deployment ceiling")
        return self


class MotionLimiter:
    """Own live settings and the previous commanded position across task changes."""

    def __init__(self, groups: dict[str, MotionGroup]) -> None:
        self._groups = {name: group.model_copy(deep=True) for name, group in groups.items()}
        self._by_joint: dict[str, str] = {}
        for name, group in self._groups.items():
            for joint in group.joints:
                if joint in self._by_joint:
                    raise ValueError(f"joint {joint!r} belongs to multiple motion groups")
                self._by_joint[joint] = name
        self._positions: dict[str, float] = {}
        self._lock = threading.Lock()

    def settings(self) -> dict[str, MotionGroup]:
        with self._lock:
            return {name: group.model_copy(deep=True) for name, group in self._groups.items()}

    def set_rates(self, rates: dict[str, float]) -> dict[str, MotionGroup]:
        with self._lock:
            for name, rate in rates.items():
                if name not in self._groups:
                    raise ValueError(f"unknown motion group {name!r}")
                if not math.isfinite(rate) or not 0 <= rate <= self._groups[name].ceiling:
                    raise ValueError(f"{name} must be within 0..{self._groups[name].ceiling}")
            for name, rate in rates.items():
                self._groups[name].rate = rate
            return {name: group.model_copy(deep=True) for name, group in self._groups.items()}

    def reset(self) -> None:
        with self._lock:
            self._positions.clear()

    def snapshot(self, measured: dict[str, float]) -> dict[str, float | None]:
        """Previous dispatched positions; None means this joint needs a fresh anchor."""
        with self._lock:
            self._positions = {j: q for j, q in self._positions.items() if j in measured}
            return {joint: self._positions.get(joint) for joint in self._by_joint}

    def commit(self, positions: dict[str, float]) -> None:
        """Advance only after the hardware/connection accepted the dispatch."""
        with self._lock:
            self._positions.update((j, q) for j, q in positions.items() if j in self._by_joint)

    def apply(
        self,
        commands: dict[str, tuple[float, ControlMode, str]],
        measured: dict[str, float],
        dt: float,
    ) -> dict[str, tuple[float, ControlMode, str]]:
        with self._lock:
            result = commands.copy()
            active = set()
            scales = {}
            for name, group in self._groups.items():
                if group.coordinated:
                    distances = [
                        abs(commands[j][0] - self._positions.get(j, measured[j]))
                        for j in group.joints
                        if j in commands
                        and j in measured
                        and commands[j][1] in (ControlMode.POSITION, ControlMode.SERVO_POSITION)
                    ]
                    scales[name] = min(1.0, group.rate * max(0.0, dt) / max([1e-12, *distances]))
            for joint, (desired, mode, owner) in commands.items():
                group_name = self._by_joint.get(joint)
                if group_name is None:
                    continue
                if not math.isfinite(desired):
                    raise ValueError(f"non-finite target for {joint}")
                rate = self._groups[group_name].rate
                if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
                    reading = measured.get(joint)
                    if reading is None or not math.isfinite(reading):
                        result.pop(joint)
                        continue
                    current = self._positions.get(joint, reading)
                    step = rate * max(0.0, dt)
                    value = current + (
                        (desired - current) * scales[group_name]
                        if group_name in scales
                        else max(-step, min(step, desired - current))
                    )
                    active.add(joint)
                    result[joint] = (value, mode, owner)
                elif mode == ControlMode.VELOCITY:
                    result[joint] = (max(-rate, min(rate, desired)), mode, owner)
            # An idle/missing-feedback joint must restart from measurement.
            self._positions = {j: q for j, q in self._positions.items() if j in active}
            return result


class MotionLimitsSpec(Spec, Protocol):
    def get_motion_limits(self) -> dict[str, dict[str, Any]]: ...
    def set_motion_limits(self, rates: dict[str, float]) -> dict[str, dict[str, Any]]: ...
