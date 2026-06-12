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

from dimos.manipulation.planning.monitor import world_monitor as world_monitor_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class FakeWorld:
    def __init__(self) -> None:
        self.calls: list[tuple[str, Any]] = []

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


class FakeViz:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def get_visualization_url(self):
        return None

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

    def register_robot(self, robot_id, config) -> None:
        self.calls.append(("register_robot", robot_id, config))


def _robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/tmp/arm.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0, 0, 0, 1])),
        joint_names=["j1", "j2"],
        end_effector_link="ee",
        base_link="base",
    )


def test_world_monitor_selection_and_register_order(monkeypatch) -> None:
    fake_world = FakeWorld()
    fake_viz = FakeViz()
    created = {}

    def fake_create_world(*, backend: str = "drake", enable_viz: bool = False, **kwargs):
        created["world"] = (backend, enable_viz, kwargs)
        return fake_world

    def fake_create_visualization(
        backend, *, world, world_monitor, manipulation_module=None, config=None
    ):
        created["viz"] = (backend, world, world_monitor, manipulation_module, config)
        return fake_viz

    monkeypatch.setattr(world_monitor_module, "create_world", fake_create_world)
    monkeypatch.setattr(world_monitor_module, "create_visualization", fake_create_visualization)

    monitor = world_monitor_module.WorldMonitor(visualization_backend="viser", enable_viz=True)
    assert created["world"][1] is False
    assert created["viz"][0] == "viser"
    assert created["viz"][1] is fake_world

    monitor.add_robot(_robot_config())
    assert fake_world.calls[0][0] == "add_robot"
    assert fake_viz.calls[0][0] == "register_robot"
    assert fake_viz.calls[0][1] == "robot-1"


def test_world_monitor_default_meshcat_enables_viz(monkeypatch) -> None:
    created = {}

    def fake_create_world(*, backend: str = "drake", enable_viz: bool = False, **kwargs):
        created["world"] = (backend, enable_viz)
        return FakeWorld()

    def fake_create_visualization(
        backend, *, world, world_monitor, manipulation_module=None, config=None
    ):
        created["viz"] = backend
        return None

    monkeypatch.setattr(world_monitor_module, "create_world", fake_create_world)
    monkeypatch.setattr(world_monitor_module, "create_visualization", fake_create_visualization)

    world_monitor_module.WorldMonitor(enable_viz=True)
    assert created["world"] == ("drake", True)
    assert created["viz"] == "meshcat"
