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

from contextlib import AbstractContextManager, nullcontext
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from numpy.typing import NDArray
from pydantic import ValidationError
import pytest

from dimos.manipulation.manipulation_module import ManipulationModuleConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import (
    JointPath,
    Obstacle,
    PlanningSceneInfo,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.manipulation.visualization.config import (
    MeshcatVisualizationConfig,
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.factory import create_manipulation_visualization
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class FakeVisualization:
    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        return None

    def get_visualization_url(self) -> str | None:
        return None

    def publish_visualization(self, ctx: object | None = None) -> None:
        return None

    def show_preview(self, robot_id: WorldRobotID) -> None:
        return None

    def hide_preview(self, robot_id: WorldRobotID) -> None:
        return None

    def animate_path(self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0) -> None:
        return None

    def close(self) -> None:
        return None

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        return None

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        return None

    def clear_vis_obstacles(self) -> None:
        return None


class FakeWorld:
    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        return "robot-1"

    def get_robot_ids(self) -> list[WorldRobotID]:
        return []

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        return RobotModelConfig(
            name="fake",
            model_path=Path("fake.urdf"),
            base_pose=PoseStamped(),
            joint_names=[],
            end_effector_link="ee_link",
        )

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return (np.array([], dtype=np.float64), np.array([], dtype=np.float64))

    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        return True

    def clear_obstacles(self) -> None:
        return None

    def get_obstacles(self) -> list[Obstacle]:
        return []

    def finalize(self) -> None:
        return None

    @property
    def is_finalized(self) -> bool:
        return True

    def get_live_context(self) -> object:
        return None

    def scratch_context(self) -> AbstractContextManager[object | None]:
        return nullcontext(None)

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        return None

    def set_joint_state(self, ctx: object, robot_id: WorldRobotID, joint_state: JointState) -> None:
        return None

    def get_joint_state(self, ctx: object, robot_id: WorldRobotID) -> JointState:
        return JointState({})

    def is_collision_free(self, ctx: object, robot_id: WorldRobotID) -> bool:
        return True

    def get_min_distance(self, ctx: object, robot_id: WorldRobotID) -> float:
        return 0.0

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        return True

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        return True

    def get_ee_pose(self, ctx: object, robot_id: WorldRobotID) -> PoseStamped:
        return PoseStamped()

    def get_link_pose(
        self, ctx: object, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        return np.eye(4, dtype=np.float64)

    def get_jacobian(self, ctx: object, robot_id: WorldRobotID) -> NDArray[np.float64]:
        return np.zeros((6, 0), dtype=np.float64)


class FakeMeshcatWorld(FakeWorld):
    def __init__(self) -> None:
        self.native_calls: list[str] = []
        self.visualization_calls: list[tuple[object, ...]] = []

    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        self.native_calls.append("add")
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        self.native_calls.append("remove")
        return True

    def clear_obstacles(self) -> None:
        self.native_calls.append("clear")

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        self.visualization_calls.append(("initialize_scene", scene))

    def get_visualization_url(self) -> str | None:
        self.visualization_calls.append(("get_visualization_url",))
        return "meshcat://test"

    def publish_visualization(self, ctx: object | None = None) -> None:
        self.visualization_calls.append(("publish_visualization", ctx))

    def show_preview(self, robot_id: WorldRobotID) -> None:
        self.visualization_calls.append(("show_preview", robot_id))

    def hide_preview(self, robot_id: WorldRobotID) -> None:
        self.visualization_calls.append(("hide_preview", robot_id))

    def animate_path(self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0) -> None:
        self.visualization_calls.append(("animate_path", robot_id, path, duration))

    def close(self) -> None:
        self.visualization_calls.append(("close",))

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        self.visualization_calls.append(("add_vis_obstacle", obstacle_id, obstacle))

    def remove_vis_obstacle(self, obstacle_id: str) -> None:
        self.visualization_calls.append(("remove_vis_obstacle", obstacle_id))

    def clear_vis_obstacles(self) -> None:
        self.visualization_calls.append(("clear_vis_obstacles",))


def test_config_defaults_to_no_visualization() -> None:
    config = ManipulationModuleConfig()

    assert isinstance(config.visualization, NoManipulationVisualizationConfig)
    assert config.visualization.requires_world_visualization is False


def test_config_rejects_unknown_visualization_backend() -> None:
    with pytest.raises(ValidationError, match="visualization"):
        ManipulationModuleConfig(visualization={"backend": "bad"})


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
            NoManipulationVisualizationConfig(),
            world=MagicMock(),
            world_monitor=MagicMock(),
            manipulation_module=MagicMock(),
        )
        is None
    )


def test_create_visualization_meshcat_accepts_structural_world() -> None:
    fake_world = FakeMeshcatWorld()
    world_monitor = MagicMock()
    visualization = create_manipulation_visualization(
        MeshcatVisualizationConfig(),
        world=fake_world,
        world_monitor=world_monitor,
        manipulation_module=MagicMock(),
    )
    assert visualization is fake_world
    assert isinstance(visualization, VisualizationSpec)
    scene = PlanningSceneInfo(robots={})
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(),
        dimensions=(1.0, 1.0, 1.0),
    )
    path = [JointState({"position": [0.1]})]
    visualization.initialize_scene(scene)
    assert visualization.get_visualization_url() == "meshcat://test"
    visualization.publish_visualization(None)
    visualization.show_preview("robot-1")
    visualization.hide_preview("robot-1")
    visualization.animate_path("robot-1", path, 2.5)
    visualization.close()
    visualization.add_vis_obstacle("box", obstacle)
    visualization.remove_vis_obstacle("box")
    visualization.clear_vis_obstacles()
    assert fake_world.visualization_calls == [
        ("initialize_scene", scene),
        ("get_visualization_url",),
        ("publish_visualization", None),
        ("show_preview", "robot-1"),
        ("hide_preview", "robot-1"),
        ("animate_path", "robot-1", path, 2.5),
        ("close",),
        ("add_vis_obstacle", "box", obstacle),
        ("remove_vis_obstacle", "box"),
        ("clear_vis_obstacles",),
    ]
    assert fake_world.native_calls == []


def test_create_visualization_meshcat_rejects_non_visualization_world() -> None:
    fake_world = FakeWorld()
    assert not isinstance(fake_world, VisualizationSpec)
    world_monitor = MagicMock()
    with pytest.raises(ValueError, match="implements VisualizationSpec"):
        create_manipulation_visualization(
            MeshcatVisualizationConfig(),
            world=fake_world,
            world_monitor=world_monitor,
            manipulation_module=MagicMock(),
        )
