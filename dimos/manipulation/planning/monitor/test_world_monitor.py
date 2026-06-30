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

from collections.abc import Sequence
from pathlib import Path
from typing import Any

from dimos.manipulation.planning import factory as planning_factory
from dimos.manipulation.planning.monitor import world_monitor as world_monitor_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    PlanningGroupID,
    PlanningSceneInfo,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


class _VectorLike(list[float]):
    def tolist(self) -> list[float]:
        return list(self)


class _FakeStateMonitor:
    def __init__(self, positions: Sequence[float], stale: bool = False) -> None:
        self._positions = _VectorLike(float(position) for position in positions)
        self._stale = stale

    def get_current_positions(self) -> _VectorLike:
        return self._positions

    def get_current_velocities(self) -> None:
        return None

    def is_state_stale(self, max_age: float) -> bool:
        del max_age
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
        self.collision_free_by_robot: dict[str, bool] = {}

    def add_robot(self, config):
        robot_id = f"robot-{len(self.configs) + 1}"
        self.configs[robot_id] = config
        self.collision_free_by_robot[robot_id] = True
        self.calls.append(("add_robot", config))
        return robot_id

    def get_robot_ids(self):
        return list(self.configs)

    def get_robot_config(self, robot_id):
        return self.configs[robot_id]

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
        self.calls.append(("get_live_context", None))
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
        self.calls.append(("get_joint_state", ctx, robot_id))
        return None

    def is_collision_free(self, ctx, robot_id):
        self.calls.append(("is_collision_free", ctx, robot_id))
        return self.collision_free_by_robot[robot_id]

    def get_min_distance(self, ctx, robot_id):
        return 0.0

    def check_config_collision_free(self, robot_id, joint_state):
        return True

    def check_edge_collision_free(self, robot_id, start, end, step_size: float = 0.05):
        return True

    def get_ee_pose(self, ctx, robot_id):
        return None

    def get_link_pose(self, ctx, robot_id, link_name):
        return []

    def get_jacobian(self, ctx, robot_id):
        return []

    def get_visualization_url(self):
        return None

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        return None

    def publish_visualization(self, ctx=None) -> None:
        return None

    def show_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        return None

    def hide_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        return None

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        return None

    def close(self) -> None:
        return None


class FakeViz:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def get_visualization_url(self):
        return None

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        self.calls.append(("initialize_scene", scene))

    def publish_visualization(self, ctx=None) -> None:
        return None

    def show_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        self.calls.append(("show_preview", tuple(group_ids)))

    def hide_preview(self, group_ids: Sequence[PlanningGroupID]) -> None:
        self.calls.append(("hide_preview", tuple(group_ids)))

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        return None

    def close(self) -> None:
        self.calls.append(("close", None))


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/tmp/arm.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0, 0, 0, 1])),
        joint_names=["j1", "j2"],
        end_effector_link="ee",
        base_link="base",
    )


def _robot_config_named(name: str, joint_names: list[str]) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path(f"/tmp/{name}.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0, 0, 0, 1])),
        joint_names=joint_names,
        end_effector_link="ee",
        base_link="base",
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

    assert fake_viz.calls[0][0] == "initialize_scene"
    scene = fake_viz.calls[0][1]
    assert isinstance(scene, PlanningSceneInfo)
    assert scene.robots["robot-1"].name == "arm"


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


def test_current_global_joint_state_uses_fresh_monitored_state_only() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    robot_id = monitor.add_robot(_robot_config_named("arm", ["j1", "j2"]))
    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.1, 0.2])  # pyright: ignore[reportPrivateUsage]

    current = monitor.current_global_joint_state(max_age=0.5)

    assert current is not None
    assert current.name == ["arm/j1", "arm/j2"]
    assert current.position == [0.1, 0.2]

    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.1, 0.2], stale=True)  # pyright: ignore[reportPrivateUsage]
    fake_world.calls.clear()

    assert monitor.current_global_joint_state(max_age=0.5) is None
    assert not any(call[0] in {"get_live_context", "get_joint_state"} for call in fake_world.calls)


def test_check_collision_fills_unmentioned_joints_in_one_world_context() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    left_id = monitor.add_robot(_robot_config_named("left", ["j1", "j2"]))
    right_id = monitor.add_robot(_robot_config_named("right", ["j3"]))
    monitor._state_monitors[left_id] = _FakeStateMonitor([0.0, 9.0])  # pyright: ignore[reportPrivateUsage]
    monitor._state_monitors[right_id] = _FakeStateMonitor([8.0])  # pyright: ignore[reportPrivateUsage]

    result = monitor.check_collision(JointState(name=["left/j1"], position=[1.0]))

    assert result.status == "VALID"
    set_joint_calls = [call for call in fake_world.calls if call[0] == "set_joint_state"]
    assert len(set_joint_calls) == 2
    assert {call[1] for call in set_joint_calls} == {"scratch"}
    left_state = set_joint_calls[0][3]
    right_state = set_joint_calls[1][3]
    assert left_state.name == ["j1", "j2"]
    assert left_state.position == [1.0, 9.0]
    assert right_state.name == ["j3"]
    assert right_state.position == [8.0]


def test_check_collision_reports_expected_statuses() -> None:
    fake_world = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    robot_id = monitor.add_robot(_robot_config_named("arm", ["j1"]))
    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.0])  # pyright: ignore[reportPrivateUsage]

    duplicate = monitor.check_collision(JointState(name=["arm/j1", "arm/j1"], position=[1.0, 2.0]))
    assert duplicate.status == "INVALID"

    local_name = monitor.check_collision(JointState(name=["j1"], position=[1.0]))
    assert local_name.status == "INVALID"

    unknown = monitor.check_collision(JointState(name=["arm/missing"], position=[1.0]))
    assert unknown.status == "INVALID"

    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.0], stale=True)  # pyright: ignore[reportPrivateUsage]
    stale = monitor.check_collision(JointState(name=["arm/j1"], position=[1.0]))
    assert stale.status == "STALE_STATE"

    monitor._state_monitors[robot_id] = _FakeStateMonitor([0.0])  # pyright: ignore[reportPrivateUsage]
    fake_world.collision_free_by_robot[robot_id] = False
    collision = monitor.check_collision(JointState(name=["arm/j1"], position=[1.0]))
    assert collision.status == "COLLISION"
    assert collision.collision_free is False
