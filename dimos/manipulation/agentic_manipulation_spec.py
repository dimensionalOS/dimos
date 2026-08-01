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

"""Spec contract for agent-facing manipulation primitive providers."""

from __future__ import annotations

from typing import Protocol

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec


class ManipulationControlSpec(Spec, Protocol):
    def get_robot_state(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def move_to_joints(
        self, joints: str, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def set_motion_speed(self, speed_scale: float) -> bool: ...
    def get_motion_speed(self) -> float: ...
    def open_gripper(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def close_gripper(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def reset(self) -> SkillResult[ManipulationSkillError]: ...
    def set_gripper(
        self, position: float, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def get_ee_pose(self, robot_name: str | None = None) -> Pose | None: ...
    def plan_to_pose(self, pose: Pose, robot_name: str | None = None) -> bool: ...
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]: ...
    def go_home(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]: ...


class AgenticGraspGenSpec(Spec, Protocol):
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None: ...
