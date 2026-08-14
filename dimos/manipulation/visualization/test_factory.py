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
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import (
    Obstacle,
    PlanningSceneInfo,
    VisualizationSession,
    VisualizationStateFrame,
    WorldRobotID,
)
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.manipulation.planning.world.drake_world import DRAKE_AVAILABLE, DrakeWorld
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

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        return None

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        return None

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
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

    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        return True

    def update_obstacle(self, obstacle: Obstacle) -> bool:
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

    def initialize(self, session: VisualizationSession) -> None:
        self.visualization_calls.append(("initialize", session))

    def get_visualization_url(self) -> str | None:
        self.visualization_calls.append(("get_visualization_url",))
        return "meshcat://test"

    def update_state(self, frame: VisualizationStateFrame) -> None:
        self.visualization_calls.append(("update_state", frame))

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        self.visualization_calls.append(("animate_trajectory", trajectory, duration))

    def cancel_preview_animation(self, robot_ids: tuple[WorldRobotID, ...] | None = None) -> None:
        self.visualization_calls.append(("cancel_preview_animation", robot_ids))

    def close(self) -> None:
        self.visualization_calls.append(("close",))

    def add_vis_obstacle(self, obstacle_id: str, obstacle: Obstacle) -> None:
        self.visualization_calls.append(("add_vis_obstacle", obstacle_id, obstacle))

    def update_vis_obstacle(self, obstacle: Obstacle) -> None:
        self.visualization_calls.append(("update_vis_obstacle", obstacle))

    def update_vis_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> None:
        self.visualization_calls.append(("update_vis_obstacle_pose", obstacle_id, pose))

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
    fake_world = FakeMeshcatWorld()
    world_monitor = MagicMock()
    visualization = create_manipulation_visualization(
        MeshcatVisualizationConfig(),
        world=fake_world,
        world_monitor=world_monitor,
        manipulation_module=MagicMock(),
    )
    assert visualization is fake_world  # type: ignore[comparison-overlap]
    assert isinstance(visualization, VisualizationSpec)
    session = VisualizationSession(PlanningSceneInfo(robots={}), operator=object())
    frame = VisualizationStateFrame(joint_states={})
    trajectory = JointTrajectory(joint_names=["arm/j1"], points=[])
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(),
        dimensions=(1.0, 1.0, 1.0),
    )
    visualization.initialize(session)
    assert visualization.get_visualization_url() == "meshcat://test"
    visualization.update_state(frame)
    visualization.cancel_preview_animation()
    visualization.animate_trajectory(trajectory, 2.5)
    visualization.close()
    visualization.add_vis_obstacle("box", obstacle)
    visualization.remove_vis_obstacle("box")
    visualization.clear_vis_obstacles()
    assert fake_world.visualization_calls == [
        ("initialize", session),
        ("get_visualization_url",),
        ("update_state", frame),
        ("cancel_preview_animation", None),
        ("animate_trajectory", trajectory, 2.5),
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


@pytest.mark.skipif(
    not DRAKE_AVAILABLE, reason="Drake visualization tests require the manipulation extra"
)
def test_drake_meshcat_visualization_lifecycle_is_noop_without_meshcat() -> None:
    world = DrakeWorld(enable_viz=False)

    visualization = create_manipulation_visualization(
        MeshcatVisualizationConfig(),
        world=world,
        world_monitor=MagicMock(),
        manipulation_module=MagicMock(),
    )

    assert visualization is world
    assert isinstance(visualization, VisualizationSpec)
    assert world.get_visualization_url() is None
    world.initialize(VisualizationSession(PlanningSceneInfo(robots={}), operator=object()))
    world.update_state(VisualizationStateFrame(joint_states={}))
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(),
        dimensions=(1.0, 1.0, 1.0),
    )
    world.add_vis_obstacle("box", obstacle)
    world.remove_vis_obstacle("box")
    world.clear_vis_obstacles()
    world.cancel_preview_animation()
    world.close()


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
