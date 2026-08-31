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

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.world.drake_world import DRAKE_AVAILABLE, DrakeWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import PlanarBaseConfig, RobotModel

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
  <link name="base_link">
    <collision>
      <geometry><box size="0.4 0.4 0.4"/></geometry>
    </collision>
  </link>
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


def _write_canonical_urdf(path: Path) -> None:
    path.write_text(
        """
<robot name="canonical">
  <link name="world"/><link name="left/base"/><link name="left/tool"/>
  <joint name="left/mount" type="fixed">
    <parent link="world"/><child link="left/base"/>
  </joint>
  <joint name="left/j1" type="revolute">
    <parent link="left/base"/><child link="left/tool"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
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
        model=RobotModel.from_file(path),
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


@requires_drake
def test_drake_loads_canonical_slash_names_natively(tmp_path: Path) -> None:
    urdf = tmp_path / "canonical.urdf"
    _write_canonical_urdf(urdf)
    config = RobotModelConfig(
        model=RobotModel.from_file(urdf),
        joint_names=["left/j1"],
        base_link="left/base",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=("left/j1",),
                base_link="left/base",
                tip_link="left/tool",
            )
        ],
    )
    world = DrakeWorld()

    world.load_model(config)
    world.finalize()

    assert world.get_model_config().joint_names == ["left/j1"]


def test_drake_config_group_helpers_resolve_groups_without_drake_runtime(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    config = _config(urdf, [_arm_group("joint2", "joint1", name="wrist")])

    group = DrakeWorld._planning_group_from_config(config, "wrist")

    assert DrakeWorld._primary_pose_group_id_for_config(config) == "wrist"
    assert group.id == "wrist"
    assert group.joint_names == ("joint2", "joint1")
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
        DrakeWorld._planning_group_from_config(ambiguous, "missing")


@requires_drake
def test_drake_obstacle_ids_are_world_owned_and_invalid_insertions_are_rejected(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1", "joint2")]))

    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=[2, 0, 0], orientation=[0, 0, 0, 1]),
        dimensions=(0.1, 0.1, 0.1),
    )
    unnamed = replace(obstacle, name="")

    operations = [
        lambda: world.add_obstacle(obstacle),
        lambda: world.remove_obstacle("box"),
        lambda: world.update_obstacle(obstacle),
        lambda: world.update_obstacle_pose("box", obstacle.pose),
        world.clear_obstacles,
        world.get_obstacles,
    ]
    for operation in operations:
        with pytest.raises(RuntimeError, match="finalized"):
            operation()
    world.finalize()
    assert world.add_obstacle(unnamed) is None
    assert world.add_obstacle(obstacle) == "box"
    joint_state = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    assert world.check_config_collision_free(joint_state)
    original_geometry_id = world._obstacles["box"].geometry_id
    assert world.add_obstacle(obstacle) is None
    assert world.remove_obstacle("missing") is False
    assert world.update_obstacle(replace(obstacle, name="missing")) is False
    assert world.update_obstacle_pose("missing", obstacle.pose) is False
    moved_pose = PoseStamped(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    assert world.update_obstacle_pose("box", moved_pose)
    assert world._obstacles["box"].geometry_id != original_geometry_id
    assert world.get_obstacles()[0].pose.position.x == pytest.approx(0.0)
    assert not world.check_config_collision_free(joint_state)

    replacement = replace(
        obstacle,
        obstacle_type=ObstacleType.SPHERE,
        dimensions=(0.5,),
        color=(0.0, 1.0, 0.0, 1.0),
    )
    assert world.update_obstacle(replacement)
    assert world.check_config_collision_free(joint_state)
    replacement.dimensions = (9.0,)
    retrieved = world.get_obstacles()[0]
    retrieved.dimensions = (8.0,)
    assert world.get_obstacles()[0].dimensions == (0.5,)


@requires_drake
def test_drake_obstacle_replacement_failure_invalidates_world(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1", "joint2")]))
    world.finalize()
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1]),
        dimensions=(0.1, 0.1, 0.1),
    )
    world.add_obstacle(obstacle)
    monkeypatch.setattr(
        world,
        "_add_obstacle_to_scene_graph",
        lambda *_args: (_ for _ in ()).throw(ValueError("native replacement failed")),
    )

    with pytest.raises(ValueError, match="native replacement failed"):
        world.update_obstacle(replace(obstacle, dimensions=(1.0, 1.0, 1.0)))
    with pytest.raises(RuntimeError, match="invalid"):
        world.get_obstacles()


@requires_drake
def test_drake_group_fk_uses_tip_link_and_unique_pose_group(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1", "joint2")]))
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(ctx, JointState({"name": ["joint1", "joint2"], "position": [0.0, 0.0]}))

    group_pose = world.get_group_ee_pose(ctx, "arm")
    default_pose = world.get_ee_pose(ctx)

    assert group_pose.position.x == pytest.approx(2.0)
    assert default_pose.position.x == pytest.approx(group_pose.position.x)
    assert world.get_jacobian(ctx).shape == (6, 2)


@requires_drake
def test_drake_applies_config_base_pose_when_urdf_has_world_base_joint(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot_with_world.urdf"
    _write_urdf_with_world_base_joint(urdf)
    world = DrakeWorld(enable_viz=False)
    world.load_model(
        RobotModelConfig(
            model=RobotModel.from_file(urdf),
            base_pose=PoseStamped(position=[0, 0.5, 0], orientation=[0, 0, 0, 1]),
            joint_names=["joint1", "joint2"],
            base_link="base_link",
            planning_groups=[_arm_group("joint1", "joint2")],
        )
    )
    world.finalize()
    ctx = world.get_live_context()

    base_pose = world.get_link_pose(ctx, "base_link")

    assert base_pose[1, 3] == pytest.approx(0.5)


@requires_drake
def test_drake_planar_base_coordinates_move_original_robot_root(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    planar_base = PlanarBaseConfig(
        position_lower=(-2.0, -2.0, -3.14),
        position_upper=(2.0, 2.0, 3.14),
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    joint_names = [*planar_base.joint_names, "joint1", "joint2"]
    world = DrakeWorld(enable_viz=False)
    world.load_model(
        RobotModelConfig(
            model=RobotModel.from_file(urdf).with_planar_base(planar_base),
            base_pose=PoseStamped(position=[0, 0, 0.5], orientation=[0, 0, 0, 1]),
            joint_names=joint_names,
            base_link=planar_base.root_link,
            planning_groups=[
                PlanningGroupDefinition(
                    name="mobile_arm",
                    joint_names=tuple(joint_names),
                    base_link=planar_base.root_link,
                    tip_link="tool0",
                )
            ],
        )
    )
    world.finalize()
    context = world.get_live_context()
    world.set_joint_state(
        context,
        JointState(name=joint_names, position=[1.0, 2.0, np.pi / 2.0, 0.0, 0.0]),
    )

    original_root_pose = world.get_link_pose(context, "base_link")
    tool_pose = world.get_group_ee_pose(context, "mobile_arm")

    np.testing.assert_allclose(original_root_pose[:3, 3], [1.0, 2.0, 0.5], atol=1e-8)
    assert tool_pose.position.x == pytest.approx(1.0)
    assert tool_pose.position.y == pytest.approx(4.0)
    assert tool_pose.position.z == pytest.approx(0.5)


@requires_drake
def test_drake_group_jacobian_shape_and_group_local_order(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(
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
    world.set_joint_state(ctx, JointState({"name": ["joint1", "joint2"], "position": [0.0, 0.0]}))

    forward_jacobian = world.get_group_jacobian(ctx, "wrist_forward")
    reverse_jacobian = world.get_group_jacobian(ctx, "wrist_reverse")

    assert reverse_jacobian.shape == (6, 2)
    np.testing.assert_allclose(reverse_jacobian[:, 0], forward_jacobian[:, 1])
    np.testing.assert_allclose(reverse_jacobian[:, 1], forward_jacobian[:, 0])


@requires_drake
def test_drake_default_pose_methods_fail_for_no_or_ambiguous_pose(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    no_pose = DrakeWorld()
    no_pose.load_model(_config(urdf, [_arm_group("joint1", tip_link=None)]))
    no_pose.finalize()
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose.get_ee_pose(no_pose.get_live_context())

    ambiguous = DrakeWorld()
    ambiguous.load_model(
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
        ambiguous.get_jacobian(ambiguous.get_live_context())


@requires_drake
def test_drake_load_rejects_group_joints_outside_controllable_set(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    with pytest.raises(ValueError, match="outside the controllable model set"):
        world.load_model(_config(urdf, [_arm_group("joint1", "joint2")], joints=["joint1"]))


@requires_drake
def test_drake_animate_trajectory_projects_selected_joints_on_shared_ticks(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        JointState(name=["joint1", "joint2"], position=[0.1, 0.2]),
    )
    updates: list[list[float]] = []
    visibility: list[bool] = []
    sleeps: list[float] = []
    monkeypatch.setattr(
        world,
        "_set_preview_positions",
        lambda _ctx, positions: updates.append(positions.tolist()),
    )
    monkeypatch.setattr(world, "_set_preview_visibility", visibility.append)
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)
    monkeypatch.setattr("time.sleep", sleeps.append)

    world.animate_trajectory(_trajectory(["joint1"], [1.0], [3.0]), duration=2.0)

    assert visibility == [True, False]
    assert updates == [[1.0, 0.2], [3.0, 0.2]]
    assert sleeps == [2.0]


@requires_drake
def test_drake_animate_trajectory_validates_before_visibility_and_cleans_up(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    visibility: list[bool] = []
    monkeypatch.setattr(world, "_set_preview_visibility", visibility.append)
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)

    with pytest.raises(ValueError, match="unknown joints"):
        world.animate_trajectory(_trajectory(["unknown/joint1"], [0.0], [1.0]))
    assert visibility == []

    def fail_preview_update(*_args: object) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(world, "_set_preview_positions", fail_preview_update)
    with pytest.raises(RuntimeError, match="boom"):
        world.animate_trajectory(_trajectory(["joint1"], [0.0], [1.0]))
    assert visibility == [True, False]


@requires_drake
def test_drake_cancel_preview_hides_model_before_animation_resumes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    visibility: list[bool] = []
    snapshots_during_sleep: list[list[bool]] = []
    monkeypatch.setattr(world, "_set_preview_visibility", visibility.append)
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)

    def cancel_during_sleep(_duration: float) -> None:
        world.cancel_preview_animation()
        snapshots_during_sleep.append(list(visibility))

    monkeypatch.setattr("time.sleep", cancel_during_sleep)

    world.animate_trajectory(_trajectory(["joint1"], [0.0], [1.0]))

    assert snapshots_during_sleep == [[True, False]]


@requires_drake
def test_drake_animate_trajectory_cancellation_stops_stale_frames(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = tmp_path / "robot.urdf"
    _write_urdf(urdf)
    world = DrakeWorld()
    world.load_model(_config(urdf, [_arm_group("joint1")]))
    world.finalize()
    world._meshcat = object()  # type: ignore[assignment]
    world.set_joint_state(
        world.get_live_context(),
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    updates: list[list[float]] = []
    visibility: list[bool] = []
    monkeypatch.setattr(
        world,
        "_set_preview_positions",
        lambda _ctx, positions: updates.append(positions.tolist()),
    )
    monkeypatch.setattr(world, "_set_preview_visibility", visibility.append)
    monkeypatch.setattr(world, "_publish_visualization", lambda: None)
    monkeypatch.setattr("time.sleep", lambda _duration: world.cancel_preview_animation())

    world.animate_trajectory(_trajectory(["joint1"], [1.0], [2.0]))

    assert updates == [[1.0, 0.0]]
    assert visibility == [True, False]
