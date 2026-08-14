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

"""Focused tests for group-aware Jacobian IK."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupDefinition
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def _pose(x: float = 0.0) -> PoseStamped:
    return PoseStamped(position=Vector3(x, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))


def _group(tip_link: str | None = "tool") -> PlanningGroup:
    return PlanningGroup(
        id="arm/manipulator",
        robot_name="arm",
        group_name="manipulator",
        joint_names=("arm/joint_a", "arm/joint_b"),
        local_joint_names=("joint_a", "joint_b"),
        base_link="base",
        tip_link=tip_link,
    )


class _World:
    is_finalized = True

    def __init__(self) -> None:
        self.group_pose_calls = 0
        self.group_jacobian_calls = 0
        self.legacy_pose_calls = 0
        self.legacy_jacobian_calls = 0
        self.config = RobotModelConfig(
            name="arm",
            model_path=Path("robot.urdf"),
            base_pose=_pose(),
            joint_names=["joint_a", "joint_b", "gripper"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator",
                    joint_names=("joint_a", "joint_b"),
                    base_link="base",
                    tip_link="tool",
                )
            ],
        )

    def get_robot_ids(self) -> list[str]:
        return ["robot"]

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self.config

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        return JointState({"name": ["joint_a", "joint_b", "gripper"], "position": [0.0, 0.0, 0.9]})

    def set_joint_state(self, ctx: object, robot_id: str, joint_state: JointState) -> None:
        self.last_state = joint_state

    def get_group_ee_pose(self, ctx: object, group_id: str) -> PoseStamped:
        self.group_pose_calls += 1
        return _pose(0.0)

    def get_group_jacobian(self, ctx: object, group_id: str) -> np.ndarray:
        self.group_jacobian_calls += 1
        return np.eye(6, 2)

    def get_ee_pose(self, ctx: object, robot_id: str) -> PoseStamped:
        self.legacy_pose_calls += 1
        raise AssertionError("legacy EE pose should not be used")

    def get_jacobian(self, ctx: object, robot_id: str) -> np.ndarray:
        self.legacy_jacobian_calls += 1
        raise AssertionError("legacy Jacobian should not be used")

    def check_config_collision_free(self, robot_id: str, joint_state: JointState) -> bool:
        return True


def test_solve_pose_targets_filters_to_group_and_uses_group_world_methods() -> None:
    world = _World()
    result = JacobianIK(max_iterations=2).solve_pose_targets(
        world=world,
        pose_targets={_group(): _pose()},
        seed=JointState(
            {"name": ["arm/joint_a", "arm/joint_b", "arm/gripper"], "position": [0.0, 0.0, 0.9]}
        ),
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint_a", "arm/joint_b"]
    assert world.group_pose_calls == 1
    assert world.group_jacobian_calls == 0
    assert world.legacy_pose_calls == 0
    assert world.legacy_jacobian_calls == 0


def test_solve_pose_targets_rejects_auxiliary_groups() -> None:
    result = JacobianIK().solve_pose_targets(
        world=_World(),
        pose_targets={_group(): _pose()},
        auxiliary_groups=[_group()],
    )

    assert result.status == IKStatus.UNSUPPORTED
    assert "no auxiliary" in result.message


def test_solve_pose_targets_rejects_group_without_pose_target_frame() -> None:
    result = JacobianIK().solve_pose_targets(world=_World(), pose_targets={_group(None): _pose()})

    assert result.status == IKStatus.UNSUPPORTED
    assert "no pose target frame" in result.message
