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

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus
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
from dimos.robot.assets.model import RobotModel


@pytest.fixture
def canonical_model_config() -> RobotModelConfig:
    """Create a model whose joint names match coordinator-facing names."""
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path/to/robot.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["left/joint1", "left/joint2", "left/joint3"],
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("left/joint1", "left/joint2", "left/joint3"),
                base_link="link_base",
                tip_link="link_tcp",
            )
        ],
    )


def _one_joint_config(name: str = "arm") -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base_link", tip_link="ee"
            )
        ],
    )


def _install_generated_plan(
    module: ManipulationModule,
    config: RobotModelConfig,
    *points: list[float],
) -> None:
    """Install a canonical generated plan and monitor state."""
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry(config.planning_groups)
    module._world_monitor.get_current_joint_state.return_value = JointState(
        name=config.joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    module._last_plan = GeneratedPlan(
        trajectory=JointTrajectory(
            joint_names=config.joint_names,
            points=[
                TrajectoryPoint(
                    time_from_start=float(index),
                    positions=list(point),
                    velocities=[0.0 for _ in config.joint_names],
                )
                for index, point in enumerate(points)
            ],
        ),
        group_ids=("manipulator",),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=config.joint_names,
                position=list(point),
            )
            for point in points
        ],
    )


def _make_module_with_monitor(module_factory) -> ManipulationModule:
    """Create a ManipulationModule with a mocked world monitor."""
    module = module_factory()
    module._world_monitor = MagicMock()
    module._init_joints = None
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
    """Test complete canonical model-state routing and init capture."""

    def test_routes_canonical_state_in_model_order(self, canonical_model_config, module_factory):
        module = _make_module_with_monitor(module_factory)
        module.config.model = canonical_model_config

        msg = JointState(
            name=["left/joint3", "unrelated", "left/joint1", "left/joint2"],
            position=[0.3, 9.0, 0.1, 0.2],
            velocity=[3.0, 9.0, 1.0, 2.0],
        )
        module._on_joint_state(msg)

        state = module._world_monitor.on_joint_state.call_args.args[0]
        assert state.name == canonical_model_config.joint_names
        assert state.position == [0.1, 0.2, 0.3]
        assert state.velocity == [1.0, 2.0, 3.0]

    def test_skips_incomplete_model_state(self, canonical_model_config, module_factory):
        module = _make_module_with_monitor(module_factory)
        module.config.model = canonical_model_config

        msg = JointState(
            name=["left/joint1", "left/joint2"],
            position=[0.5, 0.6],
        )
        module._on_joint_state(msg)

        module._world_monitor.on_joint_state.assert_not_called()

    def test_captures_init_joints_once(self, canonical_model_config, module_factory):
        module = _make_module_with_monitor(module_factory)
        module.config.model = canonical_model_config

        first_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(first_msg)
        assert module._init_joints is not None
        assert module._init_joints.position == [0.1, 0.2, 0.3]

        # Second call should NOT overwrite
        second_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.9, 0.8, 0.7],
        )
        module._on_joint_state(second_msg)
        assert module._init_joints.position == [0.1, 0.2, 0.3]

    def test_no_monitor_returns_early(self, canonical_model_config, module_factory):
        """When world_monitor is None, _on_joint_state returns without error."""
        module = module_factory()
        module.config.model = canonical_model_config
        module._world_monitor = None

        # Should not raise
        msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(msg)


class TestWorldMonitorVisualization:
    def test_visualization_routing_and_stop_all_monitors(self):
        viz = FakeVisualization()
        monitor = _make_world_monitor_with_viz(viz)
        state_monitor = MagicMock()
        state_monitor.get_current_positions.return_value = None
        obstacle_monitor = MagicMock()
        monitor._state_monitor = state_monitor
        monitor._obstacle_monitor = obstacle_monitor
        monitor._viz_thread = MagicMock()
        monitor._viz_thread.is_alive.return_value = False
        monitor._world.get_live_context.return_value = object()
        monitor._world.get_joint_state.return_value = JointState()

        assert monitor.get_visualization_url() == "123"
        monitor.update_visualization_state()
        monitor.cancel_preview_animation()
        path = _make_path([1.0], [2.0], [3.0])
        plan = GeneratedPlan(
            trajectory=JointTrajectory(),
            group_ids=("manipulator",),
            status=PlanningStatus.SUCCESS,
            path=path,
        )
        monitor.animate_trajectory(plan.trajectory, 4.5)
        assert monitor.visualization is viz
        assert viz.published is True
        assert viz.preview_animation_cancellations == 2
        assert viz.animations == [(tuple(), [], 4.5)]

        monitor.stop_all_monitors()

        assert viz.close_count == 1
        state_monitor.stop.assert_called_once()
        obstacle_monitor.stop.assert_called_once()

    def test_visualization_none_is_noop(self):
        monitor = _make_world_monitor_with_viz(None)

        assert monitor.get_visualization_url() is None
        monitor.update_visualization_state()
        monitor.cancel_preview_animation()
        monitor.animate_trajectory(JointTrajectory(), 1.0)
        monitor.start_visualization_thread()
        assert monitor._viz_thread is None


class TestManipulationPreview:
    def test_preview_without_a_plan_returns_rejected(self, module_factory):
        module = module_factory()

        result = module.preview_plan()

        assert result.status is CommandStatus.REJECTED
        assert not result.succeeded
        assert result.message == "No generated plan to preview"

    def test_clear_planned_path_invalidates_before_dismissing_preview(self, module_factory):
        module = module_factory()
        plan = GeneratedPlan(trajectory=JointTrajectory(), group_ids=("manipulator",), path=[])
        module._last_plan = plan
        module._world_monitor = MagicMock()
        plan_during_dismissal: list[GeneratedPlan | None] = []
        module._world_monitor.cancel_preview_animation.side_effect = (
            lambda: plan_during_dismissal.append(module._last_plan)
        )

        assert module.clear_planned_path().succeeded

        assert plan_during_dismissal == [None]
        module._world_monitor.cancel_preview_animation.assert_called_once_with()
        assert module._last_plan is None

    def test_clear_planned_path_clears_without_a_world_monitor(self, module_factory):
        module = module_factory()
        module._last_plan = GeneratedPlan(
            trajectory=JointTrajectory(), group_ids=("manipulator",), path=[]
        )

        assert module.clear_planned_path().succeeded
        assert module._last_plan is None

    def test_dismiss_preview_noop_without_monitor(self, module_factory):
        module = module_factory()

        module._dismiss_preview(["manipulator"])

    def test_dismiss_preview_routes_to_monitor(self, module_factory):
        module = module_factory()
        module._world_monitor = MagicMock()

        module._dismiss_preview(["manipulator"])

        module._world_monitor.cancel_preview_animation.assert_called_once_with()

    def test_preview_routes_one_complete_plan_with_default_duration(self, module_factory):
        module = module_factory()
        config = _one_joint_config()
        _install_generated_plan(module, config, [0.0], [2.0])

        assert module.preview_plan().succeeded

        module._world_monitor.animate_trajectory.assert_called_once_with(
            module._last_plan.trajectory, None
        )
