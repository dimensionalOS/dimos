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

from dimos.manipulation._test_manipulation_helpers import make_module as _make_module
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
    module._last_plan = GeneratedPlan(
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

    def test_routes_positions_to_monitor(self, robot_config_with_mapping):
        """Joint positions from aggregated message are routed to the correct monitor."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
            velocity=[1.0, 2.0, 3.0],
        )
        module._on_joint_state(msg)

        # Verify world_monitor received the sub-message
        module._world_monitor.on_joint_state.assert_called_once()
        call_args = module._world_monitor.on_joint_state.call_args
        sub_msg = call_args[0][0]
        assert sub_msg.position == [0.1, 0.2, 0.3]
        assert sub_msg.velocity == [1.0, 2.0, 3.0]
        assert call_args[1]["robot_id"] == "robot_left_arm"

    def test_skips_robot_with_missing_joints(self, robot_config_with_mapping):
        """Robots whose joints are absent from the message are skipped."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        # Message has none of left_arm's joints
        msg = JointState(
            name=["right/joint1", "right/joint2"],
            position=[0.5, 0.6],
        )
        module._on_joint_state(msg)

        module._world_monitor.on_joint_state.assert_not_called()

    def test_captures_init_joints_on_first_call(self, robot_config_with_mapping):
        """First joint state is stored as init joints; subsequent calls don't overwrite."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        first_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(first_msg)
        assert "left_arm" in module._init_joints
        assert module._init_joints["left_arm"].position == [0.1, 0.2, 0.3]

        # Second call should NOT overwrite
        second_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.9, 0.8, 0.7],
        )
        module._on_joint_state(second_msg)
        assert module._init_joints["left_arm"].position == [0.1, 0.2, 0.3]

    def test_multi_robot_splits_correctly(self):
        """With two robots, each gets only its own joints from the aggregated message."""
        left_config = RobotModelConfig(
            name="left",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("j1", "j2"), base_link="base", tip_link="ee"
                )
            ],
            joint_name_mapping={"left/j1": "j1", "left/j2": "j2"},
            coordinator_task_name="traj_left",
        )
        right_config = RobotModelConfig(
            name="right",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("j1", "j2"), base_link="base", tip_link="ee"
                )
            ],
            joint_name_mapping={"right/j1": "j1", "right/j2": "j2"},
            coordinator_task_name="traj_right",
        )
        module = _make_module_with_monitor(left_config, right_config)

        msg = JointState(
            name=["left/j1", "left/j2", "right/j1", "right/j2"],
            position=[1.0, 2.0, 3.0, 4.0],
            velocity=[0.1, 0.2, 0.3, 0.4],
        )
        module._on_joint_state(msg)

        assert module._world_monitor.on_joint_state.call_count == 2

        # Collect calls by robot_id
        calls = {
            call[1]["robot_id"]: call[0][0]
            for call in module._world_monitor.on_joint_state.call_args_list
        }
        assert calls["robot_left"].position == [1.0, 2.0]
        assert calls["robot_right"].position == [3.0, 4.0]
        assert calls["robot_left"].velocity == [0.1, 0.2]
        assert calls["robot_right"].velocity == [0.3, 0.4]

    def test_no_monitor_returns_early(self, robot_config_with_mapping):
        """When world_monitor is None, _on_joint_state returns without error."""
        module = _make_module()
        module._robots = {"left_arm": ("id", robot_config_with_mapping, MagicMock())}
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
        obstacle_monitor = MagicMock()
        monitor._state_monitors = {"robot": state_monitor}
        monitor._obstacle_monitor = obstacle_monitor
        monitor._viz_thread = MagicMock()
        monitor._viz_thread.is_alive.return_value = False

        assert monitor.get_visualization_url() == "123"
        monitor.update_visualization_state()
        monitor.cancel_preview_animation()
        path = _make_path([1.0], [2.0], [3.0])
        plan = GeneratedPlan(
            trajectory=JointTrajectory(),
            group_ids=("robot/group",),
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
    def test_clear_planned_path_invalidates_before_dismissing_preview(self):
        module = _make_module()
        plan = GeneratedPlan(trajectory=JointTrajectory(), group_ids=("arm/manipulator",), path=[])
        module._last_plan = plan
        module._world_monitor = MagicMock()
        plan_during_dismissal: list[GeneratedPlan | None] = []
        module._world_monitor.cancel_preview_animation.side_effect = (
            lambda: plan_during_dismissal.append(module._last_plan)
        )

        assert module.clear_planned_path() is True

        assert plan_during_dismissal == [None]
        module._world_monitor.cancel_preview_animation.assert_called_once_with()
        assert module._last_plan is None

    def test_clear_planned_path_clears_without_a_world_monitor(self):
        module = _make_module()
        module._last_plan = GeneratedPlan(
            trajectory=JointTrajectory(), group_ids=("arm/manipulator",), path=[]
        )

        assert module.clear_planned_path() is True
        assert module._last_plan is None

    def test_dismiss_preview_noop_without_monitor(self):
        module = _make_module()

        module._dismiss_preview(["arm/manipulator"])

    def test_dismiss_preview_routes_to_monitor(self):
        module = _make_module()
        module._world_monitor = MagicMock()

        module._dismiss_preview(["arm/manipulator"])

        module._world_monitor.cancel_preview_animation.assert_called_once_with()

    def test_preview_routes_one_complete_plan_with_default_duration(self):
        module = _make_module()
        config = _one_joint_config()
        traj_gen = MagicMock()
        _install_generated_plan(module, config, traj_gen, [0.0], [2.0])

        assert module.preview_plan() is True

        module._world_monitor.animate_trajectory.assert_called_once_with(
            module._last_plan.trajectory, None
        )

    def test_preview_robot_name_validates_affectedness_without_trimming(self):
        module = _make_module()
        left = _one_joint_config("left")
        right = _one_joint_config("right")
        traj_gen = MagicMock()
        module._robots = {
            "left": ("left_id", left, traj_gen),
            "right": ("right_id", right, traj_gen),
        }
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([left, right])
        module._last_plan = GeneratedPlan(
            trajectory=JointTrajectory(),
            group_ids=("left/manipulator", "right/manipulator"),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(name=["left/j0", "right/j0"], position=[0.0, 0.0]),
                JointState(name=["left/j0", "right/j0"], position=[1.0, 1.0]),
            ],
        )

        assert module.preview_plan(duration=2.5, robot_name="left") is True

        module._world_monitor.animate_trajectory.assert_called_once_with(
            module._last_plan.trajectory, 2.5
        )

    def test_preview_rejects_unaffected_compatibility_robot(self):
        module = _make_module()
        config = _one_joint_config()
        traj_gen = MagicMock()
        _install_generated_plan(module, config, traj_gen, [0.0], [1.0])

        assert module.preview_plan(robot_name="other") is False
        module._world_monitor.animate_trajectory.assert_not_called()
