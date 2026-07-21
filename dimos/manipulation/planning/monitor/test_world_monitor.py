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

from dimos.manipulation.planning import factory as planning_factory
from dimos.manipulation.planning.monitor import world_monitor as world_monitor_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle, PlanningSceneInfo
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class FakeWorld:
    def __init__(self) -> None:
        self.calls: list[tuple[str, Any]] = []
        self.obstacles: dict[str, Obstacle] = {}
        self._next_obstacle_id = 1

    def add_robot(self, config):
        self.calls.append(("add_robot", config))
        return "robot-1"

    def get_robot_ids(self):
        return []

    def get_robot_config(self, robot_id):
        return None

    def get_joint_limits(self, robot_id):
        return ([], [])

    def add_obstacle(self, obstacle):
        self.calls.append(("add_obstacle", obstacle))
        if any(candidate.name == obstacle.name for candidate in self.obstacles.values()):
            return None
        obstacle_id = f"world-obstacle-{self._next_obstacle_id}"
        self._next_obstacle_id += 1
        self.obstacles[obstacle_id] = obstacle
        return obstacle_id

    def remove_obstacle(self, obstacle_id):
        self.calls.append(("remove_obstacle", obstacle_id))
        return self.obstacles.pop(obstacle_id, None) is not None

    def update_obstacle_pose(self, obstacle_id, pose):
        return True

    def clear_obstacles(self) -> None:
        self.obstacles.clear()
        return None

    def get_obstacles(self):
        return list(self.obstacles.values())

    def finalize(self) -> None:
        return None

    @property
    def is_finalized(self):
        return True

    def get_live_context(self):
        return None

    def scratch_context(self):
        return self

    def sync_from_joint_state(self, robot_id, joint_state) -> None:
        return None

    def set_joint_state(self, ctx, robot_id, joint_state) -> None:
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

    def show_preview(self, robot_id) -> None:
        return None

    def hide_preview(self, robot_id) -> None:
        return None

    def animate_path(self, robot_id, path, duration: float = 3.0) -> None:
        return None

    def close(self) -> None:
        return None


class FakeViz:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []
        self.added_obstacles: list[tuple[str, Obstacle]] = []
        self.removed_obstacles: list[str] = []

    def get_visualization_url(self):
        return None

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        self.calls.append(("initialize_scene", scene))

    def publish_visualization(self, ctx=None) -> None:
        return None

    def show_preview(self, robot_id) -> None:
        self.calls.append(("show_preview", robot_id))

    def hide_preview(self, robot_id) -> None:
        self.calls.append(("hide_preview", robot_id))

    def animate_path(self, robot_id, path, duration: float = 3.0) -> None:
        return None

    def close(self) -> None:
        self.calls.append(("close", None))

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        self.added_obstacles.append((obstacle_id, obstacle))
        self.calls.append(("add_vis_obstacle", obstacle_id, obstacle))

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        self.removed_obstacles.append(obstacle_id)
        self.calls.append(("remove_vis_obstacle", obstacle_id))

    def clear_vis_obstacles(self) -> None:
        self.calls.append(("clear_vis_obstacles",))


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/tmp/arm.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0, 0, 0, 1])),
        joint_names=["j1", "j2"],
        end_effector_link="ee",
        base_link="base",
    )


def test_world_monitor_add_robot_records_scene_without_visualization_probe() -> None:
    fake_world: Any = FakeWorld()
    fake_viz = FakeViz()

    monitor = world_monitor_module.WorldMonitor(world=fake_world, visualization=fake_viz)  # type: ignore[arg-type]

    monitor.add_robot(_robot_config())
    assert fake_world.calls[0][0] == "add_robot"
    assert fake_viz.calls == []
    assert monitor.planning_scene_info().robots["robot-1"].name == "arm"


def test_world_monitor_syncs_planning_scene_to_visualization() -> None:
    fake_world: Any = FakeWorld()
    fake_viz = FakeViz()

    monitor = world_monitor_module.WorldMonitor(world=fake_world, visualization=fake_viz)  # type: ignore[arg-type]
    monitor.add_robot(_robot_config())
    monitor.finalize(fake_viz)  # type: ignore[arg-type]

    assert fake_viz.calls[0][0] == "initialize_scene"
    scene = fake_viz.calls[0][1]
    assert isinstance(scene, PlanningSceneInfo)
    assert scene.robots["robot-1"].name == "arm"


def test_world_monitor_coordinates_obstacle_visualization_after_world_mutation() -> None:
    fake_world: Any = FakeWorld()
    fake_viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(
        world=fake_world,
        visualization=fake_viz,
    )  # type: ignore[arg-type]
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )

    assert monitor.add_obstacle(obstacle) == "world-obstacle-1"
    assert fake_world.calls[-1] == ("add_obstacle", obstacle)
    assert fake_viz.added_obstacles == [("world-obstacle-1", obstacle)]
    assert monitor.remove_obstacle("world-obstacle-1") is True
    assert fake_world.calls[-1] == ("remove_obstacle", "world-obstacle-1")
    assert fake_viz.removed_obstacles == ["world-obstacle-1"]


def test_world_monitor_does_not_forward_duplicate_adds() -> None:
    fake_world: Any = FakeWorld()
    fake_viz = FakeViz()
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    monitor = world_monitor_module.WorldMonitor(
        world=fake_world,
        visualization=fake_viz,
    )  # type: ignore[arg-type]
    monitor.add_obstacle(obstacle)
    assert monitor.add_obstacle(obstacle) == ""
    assert fake_viz.added_obstacles == [("world-obstacle-1", obstacle)]


def test_world_monitor_clear_obstacles_forwards_removals_to_visualization() -> None:
    fake_world: Any = FakeWorld()
    fake_viz = FakeViz()
    monitor = world_monitor_module.WorldMonitor(
        world=fake_world,
        visualization=fake_viz,
    )  # type: ignore[arg-type]
    for name in ("first", "second"):
        monitor.add_obstacle(
            Obstacle(
                name=name,
                obstacle_type=ObstacleType.BOX,
                pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
                dimensions=(0.1, 0.2, 0.3),
            )
        )

    fake_world.obstacles["native-obstacle-id"] = Obstacle(
        name="native-name",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )

    monitor.clear_obstacles()
    assert fake_world.obstacles == {}
    assert fake_viz.removed_obstacles == []
    assert fake_viz.calls[-1] == ("clear_vis_obstacles",)


def test_world_monitor_clear_resets_obstacle_source_mappings() -> None:
    fake_world: Any = FakeWorld()
    monitor = world_monitor_module.WorldMonitor(world=fake_world)  # type: ignore[arg-type]
    monitor.start_obstacle_monitor()
    obstacle_monitor = monitor.obstacle_monitor
    assert obstacle_monitor is not None
    assert (
        obstacle_monitor.add_static_obstacle(
            "tracked",
            "box",
            PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
            (1, 1, 1),
        )
        == "world-obstacle-1"
    )

    monitor.clear_obstacles()

    assert obstacle_monitor.remove_static_obstacle("tracked") is False
    assert fake_world.obstacles == {}


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
