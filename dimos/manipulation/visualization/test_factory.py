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
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    Obstacle,
    VisualizationSession,
    VisualizationStateFrame,
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
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class FakeVisualization:
    def initialize(self, session: VisualizationSession) -> None:
        return None

    def get_visualization_url(self) -> str | None:
        return None

    def update_state(self, frame: VisualizationStateFrame) -> None:
        return None

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        return None

    def cancel_preview_animation(self) -> None:
        return None

    def close(self) -> None:
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
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=(), base_link="base_link", tip_link="ee_link"
                )
            ],
        )

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return (np.array([], dtype=np.float64), np.array([], dtype=np.float64))

    def add_obstacle(self, obstacle: Obstacle) -> str:
        return "obstacle-1"

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

    def get_group_ee_pose(self, ctx: object, group_id: str) -> PoseStamped:
        return PoseStamped()

    def get_group_jacobian(self, ctx: object, group_id: str) -> NDArray[np.float64]:
        return np.zeros((6, 0), dtype=np.float64)


class FakeVisualizationWorld(FakeWorld, FakeVisualization):
    pass


def test_config_defaults_to_no_visualization() -> None:
    config = ManipulationModuleConfig()

    assert isinstance(config.visualization, NoManipulationVisualizationConfig)
    assert config.visualization.requires_world_visualization is False


def test_config_rejects_unknown_visualization_backend() -> None:
    with pytest.raises(ValidationError, match="visualization"):
        ManipulationModuleConfig.model_validate({"visualization": {"backend": "bad"}})


def test_config_validates_viser_visualization() -> None:
    config = ManipulationModuleConfig.model_validate(
        {
            "visualization": {
                "backend": "viser",
                "visualization_host": "0.0.0.0",
                "visualization_port": "8096",
                "viser_panel_enabled": "false",
            }
        },
    )

    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.visualization.host == "0.0.0.0"
    assert config.visualization.port == 8096
    assert config.visualization.panel_enabled is False


def test_config_meshcat_requires_world_visualization() -> None:
    config = ManipulationModuleConfig.model_validate({"visualization": {"backend": "meshcat"}})

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
    fake_world = FakeVisualizationWorld()
    assert isinstance(fake_world, VisualizationSpec)
    world_monitor = MagicMock()
    assert (
        create_manipulation_visualization(
            MeshcatVisualizationConfig(),
            world=fake_world,
            world_monitor=world_monitor,
            manipulation_module=MagicMock(),
        )
        is fake_world
    )


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


def test_create_viser_visualization_has_group_preview_protocol_without_legacy_path_api() -> None:
    pytest.importorskip("viser")

    visualization = create_manipulation_visualization(
        ViserVisualizationConfig(),
        world=FakeWorld(),
        world_monitor=MagicMock(),
        manipulation_module=MagicMock(),
    )

    assert isinstance(visualization, VisualizationSpec)
    assert isinstance(FakeVisualization(), VisualizationSpec)
    assert not hasattr(visualization, "animate_path")
