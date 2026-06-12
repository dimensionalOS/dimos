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

from typing import Any, cast
from unittest.mock import MagicMock

from pydantic import ValidationError

from dimos.manipulation.manipulation_module import ManipulationModuleConfig
from dimos.manipulation.planning.spec.models import PlanningSceneInfo
from dimos.manipulation.planning.spec.protocols import VisualizationSpec, WorldSpec
from dimos.manipulation.visualization.config import (
    MeshcatVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.factory import create_manipulation_visualization
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig


class FakeVisualization:
    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        return None

    def get_visualization_url(self) -> str | None:
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


class FakeWorld(FakeVisualization):
    def add_robot(self, config):
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
        return cast("Any", self)

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


def test_config_defaults_to_no_visualization() -> None:
    config = ManipulationModuleConfig()

    assert isinstance(config.visualization, NoManipulationVisualizationConfig)
    assert config.visualization.requires_world_visualization is False


def test_config_rejects_unknown_visualization_backend() -> None:
    try:
        ManipulationModuleConfig(visualization={"backend": "bad"})
        raise AssertionError("expected ValidationError")
    except ValidationError as exc:
        assert "visualization" in str(exc)


def test_config_validates_viser_visualization() -> None:
    config = ManipulationModuleConfig(
        visualization={
            "backend": "viser",
            "visualization_host": "0.0.0.0",
            "visualization_port": "8096",
            "viser_panel_enabled": "false",
        },
    )

    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.visualization.host == "0.0.0.0"
    assert config.visualization.port == 8096
    assert config.visualization.panel_enabled is False


def test_config_meshcat_requires_world_visualization() -> None:
    config = ManipulationModuleConfig(visualization={"backend": "meshcat"})

    assert isinstance(config.visualization, MeshcatVisualizationConfig)
    assert config.visualization.requires_world_visualization is True


def test_create_visualization_none_returns_none() -> None:
    assert (
        create_manipulation_visualization(
            NoManipulationVisualizationConfig(), world=MagicMock(), world_monitor=MagicMock()
        )
        is None
    )


def test_create_visualization_meshcat_accepts_structural_world() -> None:
    fake_world = FakeWorld()
    assert isinstance(fake_world, VisualizationSpec)
    world_monitor = MagicMock()
    assert (
        create_manipulation_visualization(
            MeshcatVisualizationConfig(),
            world=cast("WorldSpec", fake_world),
            world_monitor=world_monitor,
        )
        is fake_world
    )


def test_create_visualization_meshcat_rejects_non_visualization_world() -> None:
    try:
        world_monitor = MagicMock()
        create_manipulation_visualization(
            MeshcatVisualizationConfig(),
            world=cast("WorldSpec", object()),
            world_monitor=world_monitor,
        )
        raise AssertionError("expected ValueError")
    except ValueError as exc:
        assert "implements VisualizationSpec" in str(exc)
