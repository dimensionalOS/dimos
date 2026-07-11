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

from __future__ import annotations

from typing import TypedDict

from dimos.manipulation.planning.spec.models import RobotName, WorldRobotID
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class TargetEvaluation(TypedDict, total=False):
    success: bool
    status: str
    message: str
    collision_free: bool
    joint_state: JointState | None
    ee_pose: PoseStamped | Pose | None
    position_error: float
    orientation_error: float


class RobotInfo(TypedDict):
    name: RobotName
    world_robot_id: WorldRobotID
    joint_names: list[str]
    end_effector_link: str
    base_link: str
    max_velocity: float
    max_acceleration: float
    has_joint_name_mapping: bool
    coordinator_task_name: str | None
    home_joints: list[float] | None
    pre_grasp_offset: float
    init_joints: list[float] | None
