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

import math
from typing import TypeAlias

from dimos.manipulation.planning.spec.models import RobotName
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

DegreesOfFreedom: TypeAlias = int
JointPrefix: TypeAlias = str
UrdfJointPrefix: TypeAlias = str
UrdfJointName: TypeAlias = str
CoordinatorJointName: TypeAlias = str
JointNameMapping: TypeAlias = dict[CoordinatorJointName, UrdfJointName]


def base_pose(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    pitch: float = 0.0,
) -> PoseStamped:
    half_pitch = pitch / 2.0
    return PoseStamped(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion([0.0, math.sin(half_pitch), 0.0, math.cos(half_pitch)]),
    )


def joint_names(
    dof: DegreesOfFreedom,
    *,
    prefix: JointPrefix = "joint",
) -> list[UrdfJointName]:
    return [f"{prefix}{i}" for i in range(1, dof + 1)]


def coordinator_joint_mapping(
    name: RobotName,
    dof: DegreesOfFreedom,
    *,
    joint_prefix: JointPrefix | None = None,
    urdf_joint_prefix: UrdfJointPrefix = "",
) -> JointNameMapping:
    prefix = f"{name}/" if joint_prefix is None else joint_prefix
    if not prefix:
        return {}
    return {f"{prefix}joint{i}": f"{urdf_joint_prefix}joint{i}" for i in range(1, dof + 1)}
