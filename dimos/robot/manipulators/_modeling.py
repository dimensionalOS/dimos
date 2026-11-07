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

"""Small helpers for manipulator planning model configs."""

from __future__ import annotations

from dataclasses import replace
from typing import TypeAlias

from dimos.manipulation.planning.spec.config import RobotModelConfig

DegreesOfFreedom: TypeAlias = int
JointPrefix: TypeAlias = str
UrdfJointName: TypeAlias = str


def joint_names(
    dof: DegreesOfFreedom,
    *,
    prefix: JointPrefix = "joint",
) -> list[UrdfJointName]:
    return [f"{prefix}{i}" for i in range(1, dof + 1)]


def named_for_robot(config: RobotModelConfig, robot: str | None) -> RobotModelConfig:
    """Name every commanded joint under ``robot``, e.g. ``arm/joint1``.

    The deployment calls its robots by name (``arm``), so the coordinator, the
    connection and the planner all see the same namespaced joints.
    """
    if robot is None:
        return config
    names = {name: f"{robot}/{name}" for name in config.joint_names}
    return config.model_copy(
        update={
            "model": config.model.with_renamed_joints(names),
            "joint_names": list(names.values()),
            "planning_groups": [
                replace(group, joint_names=tuple(names[name] for name in group.joint_names))
                for group in config.planning_groups
            ],
        }
    )
