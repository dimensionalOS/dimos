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

"""Task-local Twist input mapping onto explicitly selected joint velocities."""

from __future__ import annotations

import math
from typing import Any, Literal

from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.velocity_task.velocity_task import (
    JointVelocityTask,
    JointVelocityTaskConfig,
    JointVelocityTaskParams,
)
from dimos.msgs.geometry_msgs.Twist import Twist

Axis = Literal["linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"]
_AXES: tuple[Axis, ...] = (
    "linear.x",
    "linear.y",
    "linear.z",
    "angular.x",
    "angular.y",
    "angular.z",
)


class TwistVelocityTask(JointVelocityTask):
    """Map axes in joint order; optional measured yaw rotates body input to world.

    No yaw_joint means the explicitly mapped axes are aligned with the input.
    With yaw_joint, rotation is recomputed every tick, not frozen on receipt.
    """

    def __init__(
        self,
        name: str,
        config: JointVelocityTaskConfig,
        axes: list[Axis],
        yaw_joint: str | None = None,
    ) -> None:
        super().__init__(name, config)
        if len(axes) != len(config.joint_names) or any(axis not in _AXES for axis in axes):
            raise ValueError("axes must explicitly map each joint to a Twist component")
        self._axes = list(axes)
        self._yaw_joint = yaw_joint
        self._intent: tuple[float, ...] | None = None

    def on_twist_command(self, msg: Twist, t_now: float) -> bool:
        values = tuple(
            getattr(getattr(msg, group), axis)
            for group, axis in (component.split(".") for component in _AXES)
        )
        if not all(math.isfinite(value) for value in values):
            return False
        with self._lock:
            if self._estopped:
                return False
            self._intent = values
            # Reuse the velocity task's receipt clock, activation and timeout.
            return self.set_velocities([0.0] * self._num_joints, t_now)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if self._intent is None or self._estopped or not self._active:
                return None
            components = dict(zip(_AXES, self._intent, strict=True))
            if self._yaw_joint is not None:
                yaw = state.joints.get_position(self._yaw_joint)
                if yaw is None or not math.isfinite(yaw):
                    return None
                c, s = math.cos(yaw), math.sin(yaw)
                x, y = components["linear.x"], components["linear.y"]
                components["linear.x"], components["linear.y"] = c * x - s * y, s * x + c * y
            self._velocities = [components[axis] for axis in self._axes]
            return super().compute(state)

    def set_estop(self, estopped: bool) -> None:
        with self._lock:
            if estopped or self._estopped:
                self._intent = None
            super().set_estop(estopped)

    def clear(self) -> None:
        with self._lock:
            self._intent = None
            super().clear()


class TwistVelocityTaskParams(JointVelocityTaskParams):
    axes: list[Axis]
    yaw_joint: str | None = None


def create_task(cfg: Any, hardware: Any) -> TwistVelocityTask:
    params = TwistVelocityTaskParams.model_validate(cfg.params)
    return TwistVelocityTask(
        cfg.name,
        JointVelocityTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            timeout=params.timeout,
            zero_on_timeout=params.zero_on_timeout,
        ),
        axes=params.axes,
        yaw_joint=params.yaw_joint,
    )
