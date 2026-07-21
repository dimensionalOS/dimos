# Copyright 2026 Dimensional Inc.
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

"""Monitor and preview unit tests for ManipulationModule."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from dimos.manipulation._test_manipulation_helpers import (
    close_test_runtimes,
    install_runtime,
    make_module as _make_module,
)
from dimos.manipulation.execution_runtime import prepare_generated_plan
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.fixture(autouse=True)
def _close_test_runtimes() -> Iterator[None]:
    yield
    close_test_runtimes()


@pytest.fixture
def robot_config_with_mapping() -> RobotModelConfig:
    """Create a robot config with joint name mapping."""
    return RobotModelConfig(
        name="left_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2", "joint3"),
                base_link="link_base",
                tip_link="link_tcp",
            )
        ],
        joint_name_mapping={
            "left/joint1": "joint1",
            "left/joint2": "joint2",
            "left/joint3": "joint3",
        },
        coordinator_task_name="traj_left",
    )


def _one_joint_config(name: str = "arm") -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/path"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base_link", tip_link="ee"
            )
        ],
        coordinator_task_name=f"traj_{name}",
    )


def _install_generated_plan(
    module: ManipulationModule,
    config: RobotModelConfig,
    traj_gen: MagicMock,
    *points: list[float],
) -> None:
    """Install a generated plan and enough monitor state to derive robot paths."""
    global_joint_names = [f"{config.name}/{joint}" for joint in config.joint_names]
    module._robots = {config.name: ("robot_id", config, traj_gen)}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([config])
    module._world_monitor.get_current_joint_state.return_value = JointState(
        name=config.joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    generated_plan = GeneratedPlan(
        trajectory=JointTrajectory(
            joint_names=global_joint_names,
            points=[
                TrajectoryPoint(
                    time_from_start=float(index),
                    positions=list(point),
                    velocities=[0.0 for _ in config.joint_names],
                )
                for index, point in enumerate(points)
            ],
        ),
        group_ids=(f"{config.name}/manipulator",),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=global_joint_names,
                position=list(point),
            )
            for point in points
        ],
    )
    install_runtime(module, [config])
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(generated_plan, module._execution_topology)
    assert token is not None
    assert module._execution_runtime.complete_planning(token, prepared).accepted


def _make_module_with_monitor(*configs: RobotModelConfig) -> ManipulationModule:
    """Create a ManipulationModule with a mocked world monitor and robots configured."""
    module = _make_module()
    module._world_monitor = MagicMock()
    module._init_joints = {}
    for config in configs:
        robot_id = f"robot_{config.name}"
        module._robots[config.name] = (robot_id, config, MagicMock())
    return module


def _make_joint_state(positions: list[float], name: list[str] | None = None) -> JointState:
    return JointState(name=name or [f"j{i}" for i in range(len(positions))], position=positions)


def _make_path(*points: list[float]) -> list[JointState]:
    return [_make_joint_state(list(point)) for point in points]


def _make_trajectory(*points: tuple[float, list[float]]) -> JointTrajectory:
    joint_names = [f"j{i}" for i in range(len(points[0][1]))] if points else []
    return JointTrajectory(
        joint_names=joint_names,
        points=[
            TrajectoryPoint(time_from_start=time_from_start, positions=positions)
            for time_from_start, positions in points
        ],
    )


def _make_world_monitor_with_viz(viz: VisualizationSpec | None) -> WorldMonitor:
    world = MagicMock()
    return WorldMonitor(
        world=world,
        visualization=viz,
    )


class FakeVisualization:
    def __init__(self) -> None:
        self.close_count = 0
        self.published = False
        self.preview_shown: list[str] = []
        self.preview_hidden: list[str] = []
        self.animations: list[tuple[str, list[JointState], float]] = []
        self.preview_animation_cancellations = 0

    def initialize(self, session) -> None:
        pass

    def get_visualization_url(self) -> str | None:
        return "123"

    def update_state(self, frame) -> None:
        self.published = True

    def animate_trajectory(
        self, trajectory: JointTrajectory, duration: float | None = None
    ) -> None:
        self.animations.append(
            (
                tuple(trajectory.joint_names),
                list(trajectory.points),
                duration if duration is not None else 0.0,
            )
        )

    def cancel_preview_animation(self) -> None:
        self.preview_animation_cancellations += 1

    def close(self) -> None:
        self.close_count += 1


class TestOnJointState:
    """Test _on_joint_state routing, splitting, and init capture."""

    pass


class TestWorldMonitorVisualization:
    pass


class TestManipulationPreview:
    def test_dismiss_preview_noop_without_monitor(self):
        module = _make_module()

        module._dismiss_preview(["arm/manipulator"])

    def test_dismiss_preview_routes_to_monitor(self):
        module = _make_module()
        module._world_monitor = MagicMock()

        module._dismiss_preview(["arm/manipulator"])

        module._world_monitor.cancel_preview_animation.assert_called_once_with()

    def test_preview_rejects_unaffected_compatibility_robot(self):
        module = _make_module()
        config = _one_joint_config()
        traj_gen = MagicMock()
        _install_generated_plan(module, config, traj_gen, [0.0], [1.0])

        assert module.preview_plan(robot_name="other") is False
        module._world_monitor.animate_trajectory.assert_not_called()
