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

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.drake_world import DRAKE_AVAILABLE, DrakeWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

requires_drake = pytest.mark.skipif(
    not DRAKE_AVAILABLE,
    reason="Drake planning-group tests require the manipulation extra",
)


def _trajectory(names: list[str], first: list[float], second: list[float]) -> JointTrajectory:
    return JointTrajectory(
        joint_names=names,
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=first, velocities=[0.0] * len(names)),
            TrajectoryPoint(time_from_start=2.0, positions=second, velocities=[0.0] * len(names)),
        ],
    )


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


def _write_urdf_with_world_base_joint(path: Path) -> None:
    path.write_text(
        """
<robot name="chain_with_world">
  <link name="world"/>
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool0"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
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


def test_drake_config_group_helpers_resolve_groups_without_drake_runtime(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    config = _config(urdf, [_arm_group("joint2", "joint1", name="wrist")])

    group = DrakeWorld._planning_group_from_config(config, "arm/wrist")

    assert DrakeWorld._primary_pose_group_id_for_config(config) == "arm/wrist"
    assert group.id == "arm/wrist"
    assert group.joint_names == ("arm/joint2", "arm/joint1")
    assert group.local_joint_names == ("joint2", "joint1")
    assert group.tip_link == "tool0"


def test_drake_config_group_helpers_validate_duplicate_and_ambiguous_groups(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    duplicate = _config(
        urdf,
        [_arm_group("joint1", name="same"), _arm_group("joint2", name="same")],
    )
    ambiguous = _config(
        urdf,
        [_arm_group("joint1", name="a"), _arm_group("joint2", name="b")],
    )

    with pytest.raises(ValueError, match="already registered"):
        DrakeWorld._validate_planning_group_config(duplicate)
    with pytest.raises(ValueError, match="multiple pose"):
        DrakeWorld._primary_pose_group_id_for_config(ambiguous)
    with pytest.raises(KeyError, match="Unknown planning group ID"):
        DrakeWorld._planning_group_from_config(ambiguous, "arm/missing")


@requires_drake
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


@requires_drake
def test_drake_applies_config_base_pose_when_urdf_has_world_base_joint(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot_with_world.urdf"
    _write_urdf_with_world_base_joint(urdf)
    world = DrakeWorld(enable_viz=False)
    left_id = world.add_robot(
        RobotModelConfig(
            name="left_arm",
            model_path=urdf,
            base_pose=PoseStamped(position=[0, 0.5, 0], orientation=[0, 0, 0, 1]),
            joint_names=["joint1", "joint2"],
            base_link="base_link",
            planning_groups=[_arm_group("joint1", "joint2")],
        )
    )
    right_id = world.add_robot(
        RobotModelConfig(
            name="right_arm",
            model_path=urdf,
            base_pose=PoseStamped(position=[0, -0.5, 0], orientation=[0, 0, 0, 1]),
            joint_names=["joint1", "joint2"],
            base_link="base_link",
            planning_groups=[_arm_group("joint1", "joint2")],
        )
    )
    world.finalize()
    ctx = world.get_live_context()

    left_base_pose = world.get_link_pose(ctx, left_id, "base_link")
    right_base_pose = world.get_link_pose(ctx, right_id, "base_link")

    assert left_base_pose[1, 3] == pytest.approx(0.5)
    assert right_base_pose[1, 3] == pytest.approx(-0.5)
    assert left_base_pose[1, 3] != pytest.approx(right_base_pose[1, 3])


@requires_drake
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


@requires_drake
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


@requires_drake
def test_drake_group_jacobian_rejects_non_controllable_group_joints(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.add_robot(_config(urdf, [_arm_group("joint1", "joint2")], joints=["joint1"]))
    world.finalize()

    with pytest.raises(ValueError, match="non-controllable"):
        world.get_group_jacobian(world.get_live_context(), "arm/arm")


@requires_drake
def test_drake_animate_trajectory_projects_all_robots_on_shared_ticks(
    tmp_path: Path, monkeypatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    left_config = _config(urdf, [_arm_group("joint1")]).model_copy(update={"name": "left"})
    right_config = _config(urdf, [_arm_group("joint2")]).model_copy(update={"name": "right"})
    left_id = world.add_robot(left_config)
    right_id = world.add_robot(right_config)
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    ctx = world.get_live_context()
    world.set_joint_state(ctx, left_id, JointState(name=["joint1", "joint2"], position=[0.1, 0.2]))
    world.set_joint_state(ctx, right_id, JointState(name=["joint1", "joint2"], position=[0.3, 0.4]))
    updates: list[tuple[str, list[float]]] = []
    shown: list[tuple[str, ...]] = []
    hidden: list[tuple[str, ...]] = []
    sleeps: list[float] = []
    monkeypatch.setattr(
        world,
        "_set_preview_positions",
        lambda _ctx, robot_id, positions: updates.append((robot_id, positions.tolist())),
    )
    monkeypatch.setattr(
        world,
        "_set_preview_visibility",
        lambda robot_id, visible: (shown if visible else hidden).append((robot_id,)),
    )
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)
    monkeypatch.setattr("time.sleep", sleeps.append)
    plan = type("Plan", (), {})()
    plan.trajectory = _trajectory(["left/joint1", "right/joint2"], [1.0, 2.0], [3.0, 4.0])

    world.animate_trajectory(plan.trajectory, duration=2.0)

    assert shown == [(left_id,), (right_id,)]
    assert hidden == [(left_id,), (right_id,)]
    assert updates == [
        (left_id, [1.0, 0.2]),
        (right_id, [0.3, 2.0]),
        (left_id, [3.0, 0.2]),
        (right_id, [0.3, 4.0]),
    ]
    assert sleeps == [2.0]


@requires_drake
def test_drake_animate_trajectory_validates_before_visibility_and_cleans_up(
    tmp_path: Path, monkeypatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    robot_id = world.add_robot(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    shown: list[tuple[str, ...]] = []
    hidden: list[tuple[str, ...]] = []
    monkeypatch.setattr(
        world,
        "_set_preview_visibility",
        lambda robot_id, visible: (shown if visible else hidden).append((robot_id,)),
    )
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)
    malformed = _trajectory(["unknown/joint1"], [0.0], [1.0])
    with pytest.raises(ValueError, match="unknown robot"):
        world.animate_trajectory(malformed)
    assert shown == []

    valid = _trajectory(["arm/joint1"], [0.0], [1.0])

    def fail_preview_update(*_args: object) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(world, "_set_preview_positions", fail_preview_update)
    with pytest.raises(RuntimeError, match="boom"):
        world.animate_trajectory(valid)
    assert shown == [(robot_id,)]
    assert hidden == [(robot_id,)]


@requires_drake
def test_drake_cancel_preview_hides_ghosts_before_animation_resumes(
    tmp_path: Path, monkeypatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    robot_id = world.add_robot(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    hidden: list[tuple[str, ...]] = []
    hidden_snapshots_during_sleep: list[list[tuple[str, ...]]] = []
    monkeypatch.setattr(
        world,
        "_set_preview_visibility",
        lambda robot_id, visible: None if visible else hidden.append((robot_id,)),
    )
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)

    def cancel_during_sleep(_duration: float) -> None:
        world.cancel_preview_animation()
        hidden_snapshots_during_sleep.append(list(hidden))

    monkeypatch.setattr("time.sleep", cancel_during_sleep)

    world.animate_trajectory(_trajectory(["arm/joint1"], [0.0], [1.0]))

    assert hidden_snapshots_during_sleep == [[(robot_id,)]]
    assert hidden[0] == (robot_id,)


@requires_drake
def test_drake_animate_trajectory_rejects_unknown_robot_before_visibility(
    tmp_path: Path, monkeypatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.add_robot(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    shown: list[tuple[str, ...]] = []
    with pytest.raises(ValueError, match="unknown robot"):
        world.animate_trajectory(_trajectory(["missing/joint1"], [0.0], [1.0]))

    assert shown == []


@requires_drake
def test_drake_animate_trajectory_cancellation_stops_stale_frames_and_hides_preview(
    tmp_path: Path, monkeypatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    robot_id = world.add_robot(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    updates: list[list[float]] = []
    hidden: list[tuple[str, ...]] = []
    monkeypatch.setattr(
        world,
        "_set_preview_positions",
        lambda _ctx, _robot_id, positions: updates.append(positions.tolist()),
    )
    monkeypatch.setattr(
        world,
        "_set_preview_visibility",
        lambda robot_id, visible: hidden.append((robot_id,)) if not visible else None,
    )
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)
    monkeypatch.setattr("time.sleep", lambda _duration: world.cancel_preview_animation())

    world.animate_trajectory(_trajectory(["arm/joint1"], [1.0], [2.0]))

    assert updates == [[1.0, 0.0]]
    assert hidden[0] == (robot_id,)
