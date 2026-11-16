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
    """Map explicit Twist axes to joint velocities in the same declared frame.

    This task performs no frame conversion and requires no pose feedback.
    The input producer and connection declaration must agree on the frame.
    """

    def __init__(
        self,
        name: str,
        config: JointVelocityTaskConfig,
        axes: list[Axis],
    ) -> None:
        super().__init__(name, config)
        if len(axes) != len(config.joint_names) or any(axis not in _AXES for axis in axes):
            raise ValueError("axes must explicitly map each joint to a Twist component")
        self._axes = list(axes)

    def on_twist_command(self, msg: Twist, t_now: float) -> bool:
        values = tuple(
            getattr(getattr(msg, group), axis)
            for group, axis in (component.split(".") for component in _AXES)
        )
        if not all(math.isfinite(value) for value in values):
            return False
        components = dict(zip(_AXES, values, strict=True))
        return self.set_velocities([components[axis] for axis in self._axes], t_now)


class TwistVelocityTaskParams(JointVelocityTaskParams):
    axes: list[Axis]


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
    )
