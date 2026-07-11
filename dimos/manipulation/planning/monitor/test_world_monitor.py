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
from typing import Any

import numpy as np
import pytest

from dimos.manipulation.planning import factory as planning_factory
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.monitor import world_monitor as world_monitor_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    PlanningSceneInfo,
    VisualizationSession,
    VisualizationStateFrame,
)
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class _VectorLike(list[float]):
    def tolist(self) -> list[float]:
        return list(self)


class _FakeStateMonitor:
    def __init__(self, positions: list[float], stale: bool = False) -> None:
        self._positions = _VectorLike(positions)
        self._stale = stale

    def get_current_positions(self) -> _VectorLike:
        return self._positions

    def get_current_velocities(self) -> None:
        return None

    def is_state_stale(self, max_age: float) -> bool:
        _ = max_age
        return self._stale


class _ScratchContext:
    def __enter__(self) -> str:
        return "scratch"

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> bool:
        return False


class FakeWorld:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []
        self.configs: dict[str, RobotModelConfig] = {}

    def add_robot(self, config):
        self.calls.append(("add_robot", config))
        robot_id = f"robot-{len(self.configs) + 1}"
        self.configs[robot_id] = config
        return robot_id

    def get_robot_ids(self):
        return []

    def get_robot_config(self, robot_id):
        return None

    def get_joint_limits(self, robot_id):
        return ([], [])

    def add_obstacle(self, obstacle):
        return "obstacle-1"

    def remove_obstacle(self, obstacle_id):
        return True

    def update_obstacle_pose(self, obstacle_id, pose):
        return True

    def clear_obstacles(self) -> None:
        return None

    def get_obstacles(self):
        return []

    def finalize(self) -> None:
        return None

    @property
    def is_finalized(self):
        return True

    def get_live_context(self):
        return None

    def scratch_context(self):
        self.calls.append(("scratch_context", None))
        return _ScratchContext()

    def sync_from_joint_state(self, robot_id, joint_state) -> None:
        return None

    def set_joint_state(self, ctx, robot_id, joint_state) -> None:
        self.calls.append(("set_joint_state", ctx, robot_id, joint_state))
        return None

    def get_joint_state(self, ctx, robot_id):
        return None

    def is_collision_free(self, ctx, robot_id):
        return True

    def get_min_distance(self, ctx, robot_id):
        return 0.0

    def check_config_collision_free(self, robot_id, joint_state):
        return True

    def check_edge_collision_free(self, robot_id, start, end, step_size: float = 0.05):
        return True

    def get_ee_pose(self, ctx, robot_id):
        return None

    def get_group_ee_pose(self, ctx, group_id):
        self.calls.append(("get_group_ee_pose", ctx, group_id))
        return PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))

    def get_link_pose(self, ctx, robot_id, link_name):
        return []

    def get_jacobian(self, ctx, robot_id):
        return []

    def get_group_jacobian(self, ctx, group_id):
        self.calls.append(("get_group_jacobian", ctx, group_id))
        return np.ones((6, 2))

    def get_visualization_url(self):
        return None

    def initialize(self, session: VisualizationSession) -> None:
        return None

    def update_state(self, frame: VisualizationStateFrame) -> None:
        return None

    def animate_trajectory(self, trajectory, duration: float | None = None) -> None:
        return None

    def cancel_preview_animation(self) -> None:
        return None

    def close(self) -> None:
        return None


class FakeViz:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def get_visualization_url(self):
        return None

    def initialize(self, session: VisualizationSession) -> None:
        self.calls.append(("initialize", session))

    def update_state(self, frame: VisualizationStateFrame) -> None:
        self.calls.append(("update_state", frame))

    def animate_trajectory(self, trajectory, duration: float | None = None) -> None:
        self.calls.append(("animate_trajectory", trajectory, duration))

    def cancel_preview_animation(self) -> None:
        self.calls.append(("cancel_preview_animation",))

    def close(self) -> None:
        self.calls.append(("close", None))


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/tmp/arm.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0, 0, 0, 1])),
        joint_names=["j1", "j2"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j1", "j2"), base_link="base", tip_link="ee"
            )
        ],
    )


def _robot_config_with_groups(groups: list[PlanningGroupDefinition]) -> RobotModelConfig:
    return _robot_config().model_copy(update={"planning_groups": groups})


def _three_joint_reordered_group_config() -> RobotModelConfig:
    return _robot_config().model_copy(
        update={
            "joint_names": ["j1", "j2", "j3"],
            "planning_groups": [
                PlanningGroupDefinition(
                    name="manipulator",
                    joint_names=("j2", "j1"),
                    base_link="base",
                    tip_link="ee",
                )
            ],
        }
    )


def test_world_monitor_add_robot_records_scene_without_visualization_probe() -> None:
    fake_world = FakeWorld()
    fake_viz = FakeViz()

    monitor = world_monitor_module.WorldMonitor(world=fake_world, visualization=fake_viz)  # type: ignore[arg-type]

    monitor.add_robot(_robot_config())
    assert fake_world.calls[0][0] == "add_robot"
    assert fake_viz.calls == []
    assert monitor.planning_scene_info().robots["robot-1"].name == "arm"


def test_world_monitor_syncs_planning_scene_to_visualization() -> None:
    fake_world = FakeWorld()
    fake_viz = FakeViz()

    monitor = world_monitor_module.WorldMonitor(world=fake_world, visualization=fake_viz)  # type: ignore[arg-type]
    monitor.add_robot(_robot_config())
    monitor.sync_visualization_scene()

    assert fake_viz.calls[0][0] == "initialize"
    session = fake_viz.calls[0][1]
    scene = session.scene
    assert isinstance(scene, PlanningSceneInfo)
    assert scene.robots["robot-1"].name == "arm"
    assert scene.planning_groups[0].id == "arm/manipulator"


def test_world_monitor_forwards_raw_trajectory_preview_protocol() -> None:
    fake_viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(world=FakeWorld(), visualization=fake_viz)  # type: ignore[arg-type]
    trajectory = JointTrajectory(joint_names=["arm/j1"], points=[])

    assert isinstance(fake_viz, VisualizationSpec)
    monitor.cancel_preview_animation()
    monitor.animate_trajectory(trajectory, 2.0)

    assert fake_viz.calls == [
        ("cancel_preview_animation",),
        ("cancel_preview_animation",),
        ("animate_trajectory", trajectory, 2.0),
    ]


def test_create_planning_specs_wraps_existing_world(monkeypatch) -> None:
    fake_world = FakeWorld()
    fake_kinematics = object()
    fake_planner = object()

    monkeypatch.setattr(
        planning_factory,
        "create_kinematics",
        lambda *args, **kwargs: fake_kinematics,
    )
    monkeypatch.setattr(planning_factory, "create_planner", lambda **kwargs: fake_planner)

    planning_specs = planning_factory.create_planning_specs(world=fake_world)  # type: ignore[arg-type]

    assert planning_specs.world_monitor.world is fake_world
    assert planning_specs.world_monitor.visualization is None
    assert planning_specs.kinematics is fake_kinematics
    assert planning_specs.planner is fake_planner


def test_world_monitor_exposes_planning_groups_and_duplicate_names_do_not_mutate() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    monitor.add_robot(_robot_config())

    assert [group.id for group in monitor.planning_groups.list()] == ["arm/manipulator"]
    with pytest.raises(ValueError, match="already registered"):
        monitor.add_robot(_robot_config())
    assert [call[0] for call in fake_world.calls].count("add_robot") == 1


def test_world_monitor_invalid_duplicate_group_config_does_not_mutate_backend() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    invalid_config = _robot_config_with_groups(
        [
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j1",), base_link="base", tip_link="ee"
            ),
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j2",), base_link="base", tip_link="ee"
            ),
        ]
    )

    with pytest.raises(ValueError, match="already registered"):
        monitor.add_robot(invalid_config)

    assert [call[0] for call in fake_world.calls].count("add_robot") == 0


def test_world_monitor_invalid_group_joint_name_does_not_mutate_backend() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    invalid_config = _robot_config_with_groups(
        [
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("j1", "bad/joint"),
                base_link="base",
                tip_link="ee",
            )
        ]
    )

    with pytest.raises(ValueError, match="Invalid local joint name"):
        monitor.add_robot(invalid_config)

    assert [call[0] for call in fake_world.calls].count("add_robot") == 0


def test_current_group_joint_state_uses_public_names_in_group_order() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    robot_id = monitor.add_robot(_three_joint_reordered_group_config())
    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.1, 0.2, 0.3])  # type: ignore[attr-defined]

    state = monitor.current_group_joint_state("arm/manipulator")

    assert state.name == ["arm/j2", "arm/j1"]
    assert state.position == [0.2, 0.1]


def test_current_global_joint_state_skips_stale_robots_and_preserves_state_order() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    fresh_id = monitor.add_robot(_three_joint_reordered_group_config())
    stale_id = monitor.add_robot(
        RobotModelConfig(
            name="arm2",
            model_path=Path("/tmp/arm2.urdf"),
            joint_names=["a", "b"],
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("a", "b"), base_link="base", tip_link="ee"
                )
            ],
        )
    )
    monitor._state_monitors[fresh_id] = _FakeStateMonitor([0.1, 0.2, 0.3])  # type: ignore[attr-defined]
    monitor._state_monitors[stale_id] = _FakeStateMonitor([1.0, 2.0], stale=True)  # type: ignore[attr-defined]
    monitor.add_robot(
        RobotModelConfig(
            name="arm3",
            model_path=Path("/tmp/arm3.urdf"),
            joint_names=["x"],
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("x",), base_link="base", tip_link="ee"
                )
            ],
        )
    )

    state = monitor.current_global_joint_state(max_age=0.5)

    assert state.name == ["arm/j1", "arm/j2", "arm/j3"]
    assert state.position == [0.1, 0.2, 0.3]


def test_current_group_joint_state_rejects_stale_or_unavailable_state() -> None:
    stale_world = FakeWorld()
    stale_monitor = world_monitor_module.WorldMonitor(world=stale_world)  # type: ignore[arg-type]
    stale_id = stale_monitor.add_robot(_three_joint_reordered_group_config())
    stale_monitor._state_monitors[stale_id] = _FakeStateMonitor([0.1, 0.2, 0.3], stale=True)  # type: ignore[attr-defined]

    with pytest.raises(ValueError, match="stale"):
        stale_monitor.current_group_joint_state("arm/manipulator")

    unavailable_monitor = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    unavailable_monitor.add_robot(_three_joint_reordered_group_config())
    with pytest.raises(ValueError, match="unavailable"):
        unavailable_monitor.current_group_joint_state("arm/manipulator")


def test_group_ee_pose_uses_current_state_when_no_joint_state_is_provided() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    robot_id = monitor.add_robot(_three_joint_reordered_group_config())
    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.1, 0.2, 0.3])  # type: ignore[attr-defined]

    pose = monitor.get_group_ee_pose("arm/manipulator")

    set_calls = [call for call in fake_world.calls if call[0] == "set_joint_state"]
    assert set_calls[0][3].name == ["j1", "j2", "j3"]
    assert set_calls[0][3].position == [0.1, 0.2, 0.3]
    assert pose.position.x == 1


def test_group_ee_pose_without_joint_state_rejects_stale_or_unavailable_state() -> None:
    stale_world = FakeWorld()
    stale_monitor = world_monitor_module.WorldMonitor(world=stale_world)  # type: ignore[arg-type]
    stale_id = stale_monitor.add_robot(_three_joint_reordered_group_config())
    stale_monitor._state_monitors[stale_id] = _FakeStateMonitor([0.1, 0.2, 0.3], stale=True)  # type: ignore[attr-defined]

    with pytest.raises(ValueError, match="stale"):
        stale_monitor.get_group_ee_pose("arm/manipulator")

    unavailable_monitor = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    unavailable_monitor.add_robot(_three_joint_reordered_group_config())
    with pytest.raises(ValueError, match="unavailable"):
        unavailable_monitor.get_group_ee_pose("arm/manipulator")


def test_group_kinematics_with_full_state_does_not_require_current_state() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    monitor.add_robot(_three_joint_reordered_group_config())

    pose = monitor.get_group_ee_pose(
        "arm/manipulator",
        JointState(name=["j1", "j2", "j3"], position=[0.1, 0.2, 0.3]),
    )

    set_calls = [call for call in fake_world.calls if call[0] == "set_joint_state"]
    assert set_calls[0][3].name == ["j1", "j2", "j3"]
    assert set_calls[0][3].position == [0.1, 0.2, 0.3]
    assert pose.position.x == 1


def test_group_kinematics_route_full_state_to_backend() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    monitor.add_robot(_three_joint_reordered_group_config())

    pose = monitor.get_group_ee_pose(
        "arm/manipulator",
        JointState(name=["j1", "j2", "j3"], position=[0.9, 0.8, 0.3]),
    )
    jacobian = monitor.get_group_jacobian(
        "arm/manipulator",
        JointState(name=["j1", "j2", "j3"], position=[0.4, 0.3, 0.3]),
    )

    set_calls = [call for call in fake_world.calls if call[0] == "set_joint_state"]
    assert set_calls[0][3].name == ["j1", "j2", "j3"]
    assert set_calls[0][3].position == [0.9, 0.8, 0.3]
    assert set_calls[1][3].name == ["j1", "j2", "j3"]
    assert set_calls[1][3].position == [0.4, 0.3, 0.3]
    assert pose.position.x == 1
    assert jacobian.shape == (6, 2)
    assert ("get_group_ee_pose", "scratch", "arm/manipulator") in fake_world.calls
    assert ("get_group_jacobian", "scratch", "arm/manipulator") in fake_world.calls


def test_legacy_wrappers_fail_for_no_pose_and_ambiguous_pose_groups() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    no_pose_id = monitor.add_robot(
        _robot_config_with_groups(
            [PlanningGroupDefinition(name="base", joint_names=("j1",), base_link="base")]
        )
    )
    with pytest.raises(ValueError, match="no pose-targetable"):
        monitor.get_ee_pose(no_pose_id, JointState(name=["j1", "j2"], position=[0.0, 0.0]))

    fake_world2 = FakeWorld()
    monitor2 = world_monitor_module.WorldMonitor(world=fake_world2)  # type: ignore[arg-type]
    ambiguous_id = monitor2.add_robot(
        _robot_config_with_groups(
            [
                PlanningGroupDefinition(
                    name="a", joint_names=("j1",), base_link="base", tip_link="ee1"
                ),
                PlanningGroupDefinition(
                    name="b", joint_names=("j2",), base_link="base", tip_link="ee2"
                ),
            ]
        )
    )
    with pytest.raises(ValueError, match="pose-targetable planning groups"):
        monitor2.get_jacobian(ambiguous_id, JointState(name=["j1", "j2"], position=[0.0, 0.0]))
