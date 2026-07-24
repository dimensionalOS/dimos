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
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning import factory as planning_factory
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.monitor import world_monitor as world_monitor_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    CollisionObjectMessage,
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
        self.calls.append(("add_obstacle", obstacle))
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
        self.calls.append(("finalize",))
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

    def add_vis_obstacle(self, obstacle_id: str, obstacle: object) -> None:
        self.calls.append(("add_vis_obstacle", obstacle_id, obstacle))

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        self.calls.append(("remove_vis_obstacle", obstacle_id))

    def clear_vis_obstacles(self) -> None:
        self.calls.append(("clear_vis_obstacles",))


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
    operator = object()
    monitor.finalize(fake_viz, operator=operator)  # type: ignore[arg-type]
    monitor.add_obstacle(object())  # type: ignore[arg-type]

    assert [call[0] for call in fake_world.calls] == ["add_robot", "finalize", "add_obstacle"]
    assert fake_viz.calls[0][0] == "initialize"
    session = fake_viz.calls[0][1]
    assert session.operator is operator
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


def test_world_monitor_forwards_successful_native_obstacle_id() -> None:
    fake_world = FakeWorld()
    fake_viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(world=fake_world, visualization=fake_viz)  # type: ignore[arg-type]
    obstacle = object()

    # The native fake returns its owned identifier; the monitor must not derive one.
    assert monitor.add_obstacle(obstacle) == "obstacle-1"  # type: ignore[arg-type]
    assert fake_viz.calls[-1] == ("add_vis_obstacle", "obstacle-1", obstacle)


def test_obstacle_monitor_routes_mutations_through_parent_world_monitor(
    mocker: MockerFixture,
) -> None:
    parent = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    add_obstacle = mocker.patch.object(parent, "add_obstacle", return_value="parent-id")
    update_obstacle_pose = mocker.patch.object(parent, "update_obstacle_pose", return_value=True)
    remove_obstacle = mocker.patch.object(parent, "remove_obstacle", return_value=True)
    parent.start_obstacle_monitor()
    obstacle_monitor = parent.obstacle_monitor
    assert obstacle_monitor is not None

    pose = PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))
    obstacle_monitor.on_collision_object(
        CollisionObjectMessage(
            id="source-id",
            operation="add",
            primitive_type="box",
            pose=pose,
            dimensions=(1.0, 2.0, 3.0),
        )
    )
    obstacle_monitor.on_collision_object(
        CollisionObjectMessage(id="source-id", operation="update", pose=pose)
    )
    obstacle_monitor.on_collision_object(CollisionObjectMessage(id="source-id", operation="remove"))

    added_obstacle = add_obstacle.call_args.args[0]
    assert added_obstacle.name == "source-id"
    update_obstacle_pose.assert_called_once_with("parent-id", pose)
    remove_obstacle.assert_called_once_with("parent-id")


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


def test_world_monitor_obstacle_mutations_cover_failure_and_visualization_errors(
    mocker: MockerFixture,
) -> None:
    world = FakeWorld()
    viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(world=world, visualization=viz)  # type: ignore[arg-type]
    obstacle = object()

    mocker.patch.object(world, "add_obstacle", return_value="")
    assert monitor.add_obstacle(obstacle) == ""  # type: ignore[arg-type]
    assert viz.calls == []

    mocker.patch.object(world, "add_obstacle", return_value="accepted")
    mocker.patch.object(viz, "add_vis_obstacle", side_effect=RuntimeError("renderer unavailable"))
    assert monitor.add_obstacle(obstacle) == "accepted"  # type: ignore[arg-type]

    remove = mocker.patch.object(world, "remove_obstacle", side_effect=[False, True])
    mocker.patch.object(
        viz, "remove_vis_obstacle", side_effect=RuntimeError("renderer unavailable")
    )
    assert monitor.remove_obstacle("missing") is False
    assert monitor.remove_obstacle("accepted") is True
    assert remove.call_count == 2


def test_world_monitor_updates_obstacle_pose_with_backend_result(mocker: MockerFixture) -> None:
    world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=world)  # type: ignore[arg-type]
    pose = PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))
    update = mocker.patch.object(world, "update_obstacle_pose", side_effect=[True, False])

    assert monitor.update_obstacle_pose("obstacle-id", pose) is True
    assert monitor.update_obstacle_pose("obstacle-id", pose) is False
    assert update.call_args_list == [
        mocker.call("obstacle-id", pose),
        mocker.call("obstacle-id", pose),
    ]


def test_world_monitor_clear_updates_world_tracking_and_survives_visualization_error(
    mocker: MockerFixture,
) -> None:
    world = FakeWorld()
    viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(world=world, visualization=viz)  # type: ignore[arg-type]
    monitor.start_obstacle_monitor()
    obstacle_monitor = monitor.obstacle_monitor
    assert obstacle_monitor is not None
    pose = PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))
    obstacle_monitor.on_collision_object(
        CollisionObjectMessage(
            id="source",
            operation="add",
            primitive_type="box",
            pose=pose,
            dimensions=(1.0, 2.0, 3.0),
        )
    )
    detection = SimpleNamespace(
        id="detection",
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=Vector3(4, 5, 6), orientation=Quaternion([0, 0, 0, 1])),
            size=Vector3(1, 1, 1),
        ),
    )
    obstacle_monitor.on_detections([detection])  # type: ignore[arg-type]
    assert obstacle_monitor.get_obstacle_count() == 2
    clear_world = mocker.patch.object(world, "clear_obstacles")
    mocker.patch.object(
        viz, "clear_vis_obstacles", side_effect=RuntimeError("renderer unavailable")
    )

    monitor.clear_obstacles()

    clear_world.assert_called_once_with()
    assert obstacle_monitor.get_obstacle_count() == 0


def test_world_monitor_routes_obstacle_sources_and_empty_monitor_operations(
    mocker: MockerFixture,
) -> None:
    monitor = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    detections = [object()]
    objects = [object()]

    assert monitor.refresh_obstacles() == []
    assert monitor.remove_object_obstacle("missing") is False
    assert monitor.clear_perception_obstacles() == 0
    monitor.on_collision_object(CollisionObjectMessage(id="id", operation="add"))
    monitor.on_detections(detections)  # type: ignore[arg-type]
    monitor.on_objects(objects)

    monitor.start_obstacle_monitor()
    obstacle_monitor = monitor.obstacle_monitor
    assert obstacle_monitor is not None
    refresh = mocker.patch.object(obstacle_monitor, "refresh_obstacles", return_value=[{"id": "x"}])
    remove = mocker.patch.object(obstacle_monitor, "remove_object_obstacle", return_value=True)
    clear = mocker.patch.object(obstacle_monitor, "clear_perception_obstacles", return_value=2)
    on_detections = mocker.patch.object(obstacle_monitor, "on_detections")
    on_objects = mocker.patch.object(obstacle_monitor, "on_objects")

    assert monitor.refresh_obstacles(0.5) == [{"id": "x"}]
    assert monitor.remove_object_obstacle("object-id") is True
    assert monitor.clear_perception_obstacles() == 2
    monitor.on_detections(detections)  # type: ignore[arg-type]
    monitor.on_objects(objects)
    refresh.assert_called_once_with(0.5)
    remove.assert_called_once_with("object-id")
    clear.assert_called_once_with()
    on_detections.assert_called_once_with(detections)
    on_objects.assert_called_once_with(objects)


def test_world_obstacle_monitor_rejects_invalid_add_and_handles_update_and_callbacks(
    mocker: MockerFixture,
) -> None:
    parent = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    parent.start_obstacle_monitor()
    obstacle_monitor = parent.obstacle_monitor
    assert obstacle_monitor is not None
    add = mocker.patch.object(parent, "add_obstacle", return_value="obstacle-id")
    update = mocker.patch.object(parent, "update_obstacle_pose", return_value=True)
    remove = mocker.patch.object(parent, "remove_obstacle", return_value=True)
    callback = mocker.Mock(side_effect=RuntimeError("callback failed"))
    obstacle_monitor.add_obstacle_callback(callback)
    pose = PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))

    obstacle_monitor.on_collision_object(CollisionObjectMessage(id="bad", operation="add"))
    assert obstacle_monitor.get_obstacle_count() == 0
    obstacle_monitor.on_collision_object(
        CollisionObjectMessage(
            id="source-id",
            operation="add",
            primitive_type="box",
            pose=pose,
            dimensions=(1.0, 2.0, 3.0),
        )
    )
    obstacle_monitor.on_collision_object(
        CollisionObjectMessage(id="source-id", operation="update", pose=pose)
    )
    obstacle_monitor.on_collision_object(CollisionObjectMessage(id="source-id", operation="remove"))

    add.assert_called_once()
    update.assert_called_once_with("obstacle-id", pose)
    remove.assert_called_once_with("obstacle-id")
    assert callback.call_count == 3


def test_world_obstacle_monitor_detection_add_update_and_stale_cleanup(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    parent = world_monitor_module.WorldMonitor(world=FakeWorld())  # type: ignore[arg-type]
    parent.start_obstacle_monitor()
    obstacle_monitor = parent.obstacle_monitor
    assert obstacle_monitor is not None
    add = mocker.patch.object(parent, "add_obstacle", return_value="first-id")
    update = mocker.patch.object(parent, "update_obstacle_pose")
    remove = mocker.patch.object(parent, "remove_obstacle", return_value=False)
    timestamps = iter([1.0, 1.0, 1.0, 1.0, 1.0, 10.0])
    monkeypatch.setattr(
        "dimos.manipulation.planning.monitor.world_obstacle_monitor.time.time",
        lambda: next(timestamps, 10.0),
    )
    center = SimpleNamespace(position=Vector3(1, 2, 3), orientation=Quaternion([0, 0, 0, 1]))
    bbox = SimpleNamespace(center=center, size=Vector3(1, 2, 3))
    detection = SimpleNamespace(id="det-1", bbox=bbox)

    obstacle_monitor.on_detections([detection])  # type: ignore[arg-type]
    obstacle_monitor.on_detections([detection])  # type: ignore[arg-type]
    obstacle_monitor.on_detections([])

    assert add.call_count == 1
    update.assert_called_once_with("first-id", mocker.ANY)
    remove.assert_called_once_with("first-id")
    assert obstacle_monitor.get_obstacle_count() == 0
