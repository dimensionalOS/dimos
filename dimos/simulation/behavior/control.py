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

"""Simulator-independent validation and command ownership."""

from collections.abc import Sequence
import math

from dimos.simulation.behavior.types import ControlMode


class RobotControl:
    """One owner, latest commands only, and measured holds on every transfer."""

    def __init__(self, timeout: float) -> None:
        self.timeout = timeout
        self.mode = ControlMode.DIMOS
        self.generation = 0
        self.targets: dict[str, float] = {}
        self.velocity = (0.0, 0.0, 0.0)
        self.velocity_at = -math.inf
        self.action: list[float] | None = None
        self.action_at = -math.inf

    def transfer(self, mode: ControlMode, measured: dict[str, float]) -> None:
        self.mode = mode
        self.generation += 1
        self.targets = dict(measured)
        self.velocity = (0.0, 0.0, 0.0)
        self.velocity_at = -math.inf
        self.action = None
        self.action_at = -math.inf

    def set_joints(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        limits: dict[str, tuple[float, float]],
    ) -> None:
        if self.mode != ControlMode.DIMOS:
            return
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("Joint names and positions must have equal lengths and unique names")
        for name, value in zip(names, positions, strict=True):
            if name not in limits:
                raise ValueError(f"Unknown or non-commandable joint: {name}")
            lower, upper = limits[name]
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"Joint target outside limits: {name}={value}")
        self.targets.update(zip(names, positions, strict=True))

    def set_velocity(self, velocity: tuple[float, float, float], now: float) -> None:
        if self.mode != ControlMode.DIMOS:
            return
        if not all(math.isfinite(v) for v in velocity):
            raise ValueError("Base velocity must be finite")
        self.velocity, self.velocity_at = velocity, now

    def get_velocity(self, now: float) -> tuple[float, float, float]:
        return self.velocity if now - self.velocity_at <= self.timeout else (0.0, 0.0, 0.0)

    def set_action(
        self, action: Sequence[float], bounds: Sequence[tuple[float, float]], now: float
    ) -> None:
        if self.mode != ControlMode.NATIVE:
            return
        if len(action) != len(bounds):
            raise ValueError(f"Expected {len(bounds)} action values, got {len(action)}")
        if any(
            not math.isfinite(v) or not lo <= v <= hi
            for v, (lo, hi) in zip(action, bounds, strict=True)
        ):
            raise ValueError("Native action contains nonfinite or out-of-range values")
        self.action, self.action_at = list(action), now

    def get_action(self, now: float) -> list[float] | None:
        return self.action if now - self.action_at <= self.timeout else None
