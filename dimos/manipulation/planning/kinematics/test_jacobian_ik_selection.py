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

"""Tests for Jacobian IK selected planning group result contracts."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import cast

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def _pose() -> PoseStamped:
    return PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1])


def _joint_state(names: list[str], positions: list[float]) -> JointState:
    return JointState({"name": names, "position": positions})


def _group(
    group_id: str, joint_names: tuple[str, ...], tip_link: str | None = "tool0"
) -> PlanningGroup:
    return PlanningGroup(
        id=group_id,
        robot_name="arm",
        group_name=group_id.split("/", maxsplit=1)[1],
        joint_names=joint_names,
        local_joint_names=tuple(name.split("/", maxsplit=1)[1] for name in joint_names),
        base_link="base_link",
        tip_link=tip_link,
    )


class _IKWorld:
    def __init__(self, groups: Mapping[str, PlanningGroup]) -> None:
        self._groups = groups
        self._robot_configs = {
            "robot_1": RobotModelConfig(
                name="arm",
                model_path=Path("robot.urdf"),
                base_pose=_pose(),
                joint_names=["joint1", "joint2", "gripper"],
                end_effector_link="tool0",
            )
        }

    def get_robot_ids(self) -> list[str]:
        return list(self._robot_configs)

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self._robot_configs[robot_id]


class _SuccessfulIK(JacobianIK):
    def solve(
        self,
        world: WorldSpec,
        robot_id: str,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=_joint_state(
                ["arm/joint1", "arm/joint2", "arm/gripper", "arm/unrelated"],
                [0.1, 0.2, 0.3, 0.4],
            ),
        )


def test_solve_pose_targets_filters_result_to_single_group_joints() -> None:
    world = _IKWorld(
        {
            "arm/arm": _group("arm/arm", ("arm/joint1", "arm/joint2")),
        }
    )

    result = _SuccessfulIK().solve_pose_targets(
        world=cast("WorldSpec", world),
        pose_targets={world._groups["arm/arm"]: _pose()},
        seed=_joint_state(["arm/joint1", "arm/joint2", "arm/gripper"], [0.0, 0.0, 0.0]),
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint1", "arm/joint2"]
    assert result.joint_state.position == [0.1, 0.2]


def test_solve_pose_targets_rejects_auxiliary_groups() -> None:
    world = _IKWorld({"arm/arm": _group("arm/arm", ("arm/joint1", "arm/joint2"))})

    result = _SuccessfulIK().solve_pose_targets(
        world=cast("WorldSpec", world),
        pose_targets={world._groups["arm/arm"]: _pose()},
        auxiliary_groups=[_group("arm/gripper", ("arm/gripper",))],
        seed=_joint_state(["arm/joint1", "arm/joint2"], [0.0, 0.0]),
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert "no auxiliary planning groups" in result.message


def test_solve_pose_targets_rejects_group_without_pose_target_frame() -> None:
    world = _IKWorld({"arm/gripper": _group("arm/gripper", ("arm/gripper",), tip_link=None)})

    result = JacobianIK().solve_pose_targets(
        world=cast("WorldSpec", world),
        pose_targets={world._groups["arm/gripper"]: _pose()},
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert "no pose target frame" in result.message
