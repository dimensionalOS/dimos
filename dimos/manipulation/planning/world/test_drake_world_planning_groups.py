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

from pathlib import Path

import numpy as np
import pytest

pydrake = pytest.importorskip("pydrake")
_ = pydrake

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.drake_world import DrakeWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def _write_urdf(path: Path) -> None:
    path.write_text(
        """
<robot name="chain">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool0"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="tool0"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    )


def _config(
    path: Path, groups: list[PlanningGroupDefinition], joints: list[str] | None = None
) -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=path,
        base_pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        joint_names=joints or ["joint1", "joint2"],
        base_link="base_link",
        planning_groups=groups,
    )


def _arm_group(
    *joint_names: str, tip_link: str | None = "tool0", name: str = "arm"
) -> PlanningGroupDefinition:
    return PlanningGroupDefinition(
        name=name, joint_names=joint_names, base_link="base_link", tip_link=tip_link
    )


def test_drake_group_fk_uses_tip_link_and_legacy_unique_pose_group(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    robot_id = world.add_robot(_config(urdf, [_arm_group("joint1", "joint2")]))
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx, robot_id, JointState({"name": ["joint1", "joint2"], "position": [0.0, 0.0]})
    )

    group_pose = world.get_group_ee_pose(ctx, "arm/arm")
    legacy_pose = world.get_ee_pose(ctx, robot_id)

    assert group_pose.position.x == pytest.approx(2.0)
    assert legacy_pose.position.x == pytest.approx(group_pose.position.x)
    assert world.get_jacobian(ctx, robot_id).shape == (6, 2)


def test_drake_group_jacobian_shape_and_group_local_order(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    robot_id = world.add_robot(
        _config(
            urdf,
            [
                _arm_group("joint1", "joint2", name="wrist_forward"),
                _arm_group("joint2", "joint1", name="wrist_reverse"),
            ],
        )
    )
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx, robot_id, JointState({"name": ["joint1", "joint2"], "position": [0.0, 0.0]})
    )

    forward_jacobian = world.get_group_jacobian(ctx, "arm/wrist_forward")
    reverse_jacobian = world.get_group_jacobian(ctx, "arm/wrist_reverse")

    assert reverse_jacobian.shape == (6, 2)
    np.testing.assert_allclose(reverse_jacobian[:, 0], forward_jacobian[:, 1])
    np.testing.assert_allclose(reverse_jacobian[:, 1], forward_jacobian[:, 0])


def test_drake_legacy_wrappers_fail_at_call_time_for_no_or_ambiguous_pose(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    no_pose = DrakeWorld()
    no_pose_id = no_pose.add_robot(_config(urdf, [_arm_group("joint1", tip_link=None)]))
    no_pose.finalize()
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose.get_ee_pose(no_pose.get_live_context(), no_pose_id)

    ambiguous = DrakeWorld()
    ambiguous_id = ambiguous.add_robot(
        _config(
            urdf,
            [
                _arm_group("joint1", tip_link="link1", name="a"),
                _arm_group("joint2", tip_link="tool0", name="b"),
            ],
        )
    )
    ambiguous.finalize()
    with pytest.raises(ValueError, match="multiple pose"):
        ambiguous.get_jacobian(ambiguous.get_live_context(), ambiguous_id)


def test_drake_group_jacobian_rejects_non_controllable_group_joints(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.add_robot(_config(urdf, [_arm_group("joint1", "joint2")], joints=["joint1"]))
    world.finalize()

    with pytest.raises(ValueError, match="non-controllable"):
        world.get_group_jacobian(world.get_live_context(), "arm/arm")
