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

"""Unit tests for the ManipulationModule."""

from __future__ import annotations

from pathlib import Path
import threading
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, PlanningSceneInfo
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.fixture
def robot_config():
    """Create a robot config for testing."""
    return RobotModelConfig(
        name="test_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="link_tcp",
        base_link="link_base",
        max_velocity=1.0,
        max_acceleration=2.0,
        coordinator_task_name="traj_arm",
    )


@pytest.fixture
def robot_config_with_mapping():
    """Create a robot config with joint name mapping (dual-arm scenario)."""
    return RobotModelConfig(
        name="left_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="link_tcp",
        base_link="link_base",
        joint_name_mapping={
            "left/joint1": "joint1",
            "left/joint2": "joint2",
            "left/joint3": "joint3",
        },
        coordinator_task_name="traj_left",
    )


@pytest.fixture
def simple_trajectory():
    """Create a simple trajectory for testing."""
    return JointTrajectory(
        joint_names=["joint1", "joint2", "joint3"],
        points=[
            TrajectoryPoint(
                positions=[0.0, 0.0, 0.0], velocities=[0.0, 0.0, 0.0], time_from_start=0.0
            ),
            TrajectoryPoint(
                positions=[0.5, 0.5, 0.5], velocities=[0.0, 0.0, 0.0], time_from_start=1.0
            ),
        ],
    )


class _ManipulationModuleHarness(ManipulationModule):
    def __init__(self) -> None:
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""
        self._planning_epoch = 0
        self._robots = {}
        self._planned_paths = {}
        self._planned_trajectories = {}
        self._world_monitor = None
        self._planner = None
        self._kinematics = None
        self._coordinator_client = None
        self.config = MagicMock(planning_timeout=10.0)


def _make_module() -> ManipulationModule:
    """Create a lightweight ManipulationModule harness for behavior tests."""
    return _ManipulationModuleHarness()


class TestStateMachine:
    """Test state transitions."""

    def test_cancel_interrupts_active_work(self):
        """Cancel works for executing motion and in-progress planning."""
        module = _make_module()

        module._state = ManipulationState.IDLE
        assert module.cancel() is False

        module._state = ManipulationState.PLANNING
        assert module.cancel() is True
        assert module._state == ManipulationState.IDLE
        assert module._planning_epoch == 1

        module._state = ManipulationState.EXECUTING
        assert module.cancel() is True
        assert module._state == ManipulationState.IDLE

    def test_reset_not_during_execution(self):
        """Reset works in any state except EXECUTING."""
        module = _make_module()

        module._state = ManipulationState.FAULT
        module._error_message = "Error"
        result = module.reset()
        assert result.is_success()
        assert module._state == ManipulationState.IDLE
        assert module._error_message == ""

        module._state = ManipulationState.EXECUTING
        result = module.reset()
        assert not result.is_success()
        assert result.error_code == "INVALID_STATE"

    def test_fail_sets_fault_state(self):
        """_fail helper sets FAULT state and message."""
        module = _make_module()
        module._state = ManipulationState.PLANNING

        result = module._fail("Test error")
        assert result is False
        assert module._state == ManipulationState.FAULT
        assert module._error_message == "Test error"

    def test_begin_planning_state_checks(self, robot_config):
        """_begin_planning only allowed from IDLE or COMPLETED."""
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}

        # From IDLE - OK
        module._state = ManipulationState.IDLE
        assert module._begin_planning() == ("test_arm", "robot_id")
        assert module._state == ManipulationState.PLANNING

        # From COMPLETED - OK
        module._state = ManipulationState.COMPLETED
        assert module._begin_planning() == ("test_arm", "robot_id")

        # From EXECUTING - Fail
        module._state = ManipulationState.EXECUTING
        assert module._begin_planning() is None


class TestRobotSelection:
    """Test robot selection logic."""

    def test_single_robot_default(self, robot_config):
        """Single robot is used by default."""
        module = _make_module()
        module._robots = {"arm": ("id", robot_config, MagicMock())}

        result = module._get_robot()
        assert result is not None
        assert result[0] == "arm"

    def test_multiple_robots_require_name(self, robot_config):
        """Multiple robots require explicit name."""
        module = _make_module()
        module._robots = {
            "left": ("id1", robot_config, MagicMock()),
            "right": ("id2", robot_config, MagicMock()),
        }

        # No name - fails
        assert module._get_robot() is None

        # With name - works
        result = module._get_robot("left")
        assert result is not None
        assert result[0] == "left"


class PlanningInitializationHarness:
    def __init__(self, mocker: MockerFixture) -> None:
        self.mock_world = MagicMock()
        self.mock_world_monitor = MagicMock(spec=WorldMonitor)
        self.mock_world_monitor.add_robot.return_value = "robot_id"
        self.planning_specs = MagicMock(
            world_monitor=self.mock_world_monitor,
            planner=MagicMock(),
            kinematics=MagicMock(),
        )
        self.mock_planning_specs = mocker.patch(
            "dimos.manipulation.manipulation_module.create_planning_specs",
            return_value=self.planning_specs,
        )
        mocker.patch(
            "dimos.manipulation.manipulation_module.create_world",
            return_value=self.mock_world,
        )
        mocker.patch("dimos.manipulation.manipulation_module.create_manipulation_visualization")
        mocker.patch("dimos.manipulation.manipulation_module.JointTrajectoryGenerator")


@pytest.fixture
def planning_initialization(mocker: MockerFixture) -> PlanningInitializationHarness:
    return PlanningInitializationHarness(mocker)


class TestPlanningInitialization:
    """Test planning backend configuration wiring."""

    def test_default_kinematics_config_uses_pink(self) -> None:
        """Pink IK is the default solver for manipulation modules."""
        config = ManipulationModuleConfig()

        assert isinstance(config.kinematics, PinkKinematicsConfig)

    def test_kinematics_config_is_passed_to_factory(
        self, robot_config, planning_initialization: PlanningInitializationHarness
    ):
        """ManipulationModule config selects the requested IK backend."""
        module = _make_module()
        kinematics = PinkKinematicsConfig(max_iterations=100, dt=0.02)
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics=kinematics,
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="drake",
            planner_name="rrt_connect",
            kinematics_name=None,
            kinematics=kinematics,
        )

    def test_legacy_kinematics_name_still_selects_backend(
        self, robot_config, planning_initialization: PlanningInitializationHarness
    ):
        """The old kinematics_name field remains a compatibility shim."""
        module = _make_module()
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics_name="pink",
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="drake",
            planner_name="rrt_connect",
            kinematics_name="pink",
            kinematics=module.config.kinematics,
        )

    def test_nested_kinematics_config_parses_cli_override_shape(self) -> None:
        """Pydantic parses the nested CLI config shape used by -o overrides."""
        config = ManipulationModuleConfig(
            kinematics={
                "backend": "pink",
                "max_iterations": "100",
                "dt": "0.02",
                "posture_cost": "0.0",
            }
        )

        assert isinstance(config.kinematics, PinkKinematicsConfig)
        assert config.kinematics.max_iterations == 100
        assert config.kinematics.dt == 0.02
        assert config.kinematics.posture_cost == 0.0

    def test_solve_ik_rpc_calls_configured_backend(self, robot_config):
        """solve_ik returns the backend IKResult without path planning."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        current = JointState(name=robot_config.joint_names, position=[0.0, 0.0, 0.0])
        module._world_monitor.get_current_joint_state.return_value = current
        expected = IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(name=robot_config.joint_names, position=[0.1, 0.2, 0.3]),
            position_error=0.0001,
            orientation_error=0.0002,
            iterations=3,
            message="ok",
        )
        module._kinematics = MagicMock()
        module._kinematics.solve.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose)

        assert result is expected
        assert module._state == ManipulationState.COMPLETED
        assert module._planned_paths == {}
        module._kinematics.solve.assert_called_once()
        _, kwargs = module._kinematics.solve.call_args
        assert kwargs["world"] is module._world_monitor.world
        assert kwargs["robot_id"] == "robot_id"
        assert kwargs["seed"] is current
        assert kwargs["check_collision"] is True
        assert kwargs["target_pose"].frame_id == "world"
        assert kwargs["target_pose"].position.x == 0.45

    def test_solve_ik_rpc_returns_failure_without_joint_state(self, robot_config):
        """solve_ik reports a failed IKResult when no seed state is available."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.get_current_joint_state.return_value = None
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose)

        assert result.status == IKStatus.NO_SOLUTION
        assert result.message == "No joint state"
        assert module._state == ManipulationState.IDLE
        module._kinematics.solve.assert_not_called()

    def test_solve_ik_rpc_uses_explicit_seed(self, robot_config):
        """solve_ik initializes the backend from an explicit seed when provided."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names, position=[0.0, 0.0, 0.0]
        )
        explicit_seed = JointState(name=robot_config.joint_names, position=[0.2, 0.1, 0.0])
        expected = IKResult(status=IKStatus.SUCCESS, joint_state=explicit_seed)
        module._kinematics = MagicMock()
        module._kinematics.solve.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose, seed=explicit_seed)

        assert result is expected
        _, kwargs = module._kinematics.solve.call_args
        assert kwargs["seed"] is explicit_seed
        module._world_monitor.get_current_joint_state.assert_not_called()


class TestJointNameTranslation:
    """Test trajectory joint name translation for coordinator."""

    def test_no_mapping_returns_original(self, robot_config, simple_trajectory):
        """Without mapping, trajectory is returned unchanged."""
        module = _make_module()

        result = module._translate_trajectory_to_coordinator(simple_trajectory, robot_config)
        assert result is simple_trajectory  # Same object

    def test_mapping_translates_names(self, robot_config_with_mapping, simple_trajectory):
        """With mapping, joint names are translated."""
        module = _make_module()

        result = module._translate_trajectory_to_coordinator(
            simple_trajectory, robot_config_with_mapping
        )
        assert result.joint_names == ["left/joint1", "left/joint2", "left/joint3"]
        assert len(result.points) == 2  # Points preserved


class TestExecute:
    """Test coordinator execution."""

    def test_execute_requires_trajectory(self, robot_config):
        """Execute fails without planned trajectory."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {}

        assert module.execute() is False

    def test_execute_requires_task_name(self):
        """Execute fails without coordinator_task_name."""
        module = _make_module()
        config_no_task = RobotModelConfig(
            name="arm",
            model_path=Path("/path"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1"],
            end_effector_link="ee",
        )
        module._robots = {"arm": ("id", config_no_task, MagicMock())}
        module._planned_trajectories = {"arm": MagicMock()}

        assert module.execute() is False

    def test_execute_success(self, robot_config, simple_trajectory):
        """Successful execute calls coordinator via task_invoke."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        assert module.execute() is True
        assert module._state == ManipulationState.COMPLETED
        mock_client.task_invoke.assert_called_once_with(
            "traj_arm", "execute", {"trajectory": simple_trajectory}
        )

    def test_execute_rejected(self, robot_config, simple_trajectory):
        """Rejected execution sets FAULT state."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        assert module.execute() is False
        assert module._state == ManipulationState.FAULT


class TestRobotModelConfigMapping:
    """Test RobotModelConfig joint name mapping helpers."""

    def test_bidirectional_mapping(self, robot_config_with_mapping):
        """Test URDF <-> coordinator name translation."""
        config = robot_config_with_mapping

        # Coordinator -> URDF
        assert config.get_urdf_joint_name("left/joint1") == "joint1"
        assert config.get_urdf_joint_name("unknown") == "unknown"

        # URDF -> Coordinator
        assert config.get_coordinator_joint_name("joint1") == "left/joint1"
        assert config.get_coordinator_joint_name("unknown") == "unknown"


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

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        pass

    def get_visualization_url(self) -> str | None:
        return "123"

    def publish_visualization(self, ctx: object | None = None) -> None:
        self.published = True

    def show_preview(self, robot_id: str) -> None:
        self.preview_shown.append(robot_id)

    def hide_preview(self, robot_id: str) -> None:
        self.preview_hidden.append(robot_id)

    def animate_path(self, robot_id: str, path: list[JointState], duration: float = 3.0) -> None:
        self.animations.append((robot_id, path, duration))

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
            end_effector_link="ee",
            base_link="base",
            joint_name_mapping={"left/j1": "j1", "left/j2": "j2"},
            coordinator_task_name="traj_left",
        )
        right_config = RobotModelConfig(
            name="right",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            end_effector_link="ee",
            base_link="base",
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
        monitor.publish_visualization()
        monitor.show_preview("robot")
        monitor.hide_preview("robot")
        path = _make_path([1.0], [2.0], [3.0])
        monitor.animate_path("robot", path, 4.5)
        assert monitor.visualization is viz
        assert viz.published is True
        assert viz.preview_shown == ["robot"]
        assert viz.preview_hidden == ["robot"]
        assert viz.animations == [("robot", path, 4.5)]

        monitor.stop_all_monitors()

        assert viz.close_count == 1
        state_monitor.stop.assert_called_once()
        obstacle_monitor.stop.assert_called_once()

    def test_visualization_none_is_noop(self):
        monitor = _make_world_monitor_with_viz(None)

        assert monitor.get_visualization_url() is None
        monitor.publish_visualization()
        monitor.show_preview("robot")
        monitor.hide_preview("robot")
        monitor.animate_path("robot", [1], 1.0)
        monitor.start_visualization_thread()
        assert monitor._viz_thread is None


class TestManipulationPreview:
    def test_dismiss_preview_noop_without_monitor(self):
        module = _make_module()

        module._dismiss_preview("robot_id")

    def test_dismiss_preview_routes_to_monitor(self):
        module = _make_module()
        module._world_monitor = MagicMock()

        module._dismiss_preview("robot_id")

        module._world_monitor.hide_preview.assert_called_once_with("robot_id")
        module._world_monitor.publish_visualization.assert_called_once_with()

    def test_preview_path_uses_trajectory_duration_and_interpolates(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}
        module._planned_paths = {"arm": _make_path([0.0], [2.0])}
        module._planned_trajectories = {"arm": _make_trajectory((0.0, [0.0]), (2.0, [2.0]))}

        assert module.preview_path(robot_name="arm", target_fps=2.0) is True

        module._world_monitor.animate_path.assert_called_once()
        robot_id, preview_path, duration = module._world_monitor.animate_path.call_args.args
        assert robot_id == "robot_id"
        assert duration == 2.0
        assert [state.position for state in preview_path] == [[0.0], [0.5], [1.0], [1.5], [2.0]]

    def test_preview_path_explicit_duration_overrides_and_fps_densifies(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}
        module._planned_paths = {"arm": _make_path([0.0], [9.0])}
        module._planned_trajectories = {"arm": _make_trajectory((0.0, [0.0]), (9.0, [9.0]))}

        assert module.preview_path(duration=1.5, robot_name="arm", target_fps=2.0) is True

        module._world_monitor.animate_path.assert_called_once()
        robot_id, preview_path, duration = module._world_monitor.animate_path.call_args.args
        assert robot_id == "robot_id"
        assert duration == 1.5
        assert [state.position for state in preview_path] == [[0.0], [3.0], [6.0], [9.0]]

    def test_preview_path_missing_trajectory_uses_default_duration(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}
        module._planned_paths = {"arm": _make_path([0.0], [1.0])}
        module._planned_trajectories = {}

        assert module.preview_path(robot_name="arm", target_fps=10.0) is True

        module._world_monitor.animate_path.assert_called_once_with(
            "robot_id", module._planned_paths["arm"], 3.0
        )

    def test_preview_path_skips_interpolation_for_nonpositive_fps_or_duration(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}
        module._planned_paths = {"arm": _make_path([0.0], [1.0])}
        module._planned_trajectories = {"arm": _make_trajectory((0.0, [0.0]), (2.0, [1.0]))}

        assert module.preview_path(robot_name="arm", target_fps=0.0) is True
        assert module.preview_path(duration=0.0, robot_name="arm", target_fps=20.0) is True

        assert (
            module._world_monitor.animate_path.call_args_list[0].args[1]
            == module._planned_paths["arm"]
        )
        assert (
            module._world_monitor.animate_path.call_args_list[1].args[1]
            == module._planned_paths["arm"]
        )

    def test_preview_path_returns_false_for_missing_inputs(self):
        module = _make_module()
        module._planned_paths = {"arm": _make_path([0.0], [1.0])}
        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}

        assert module.preview_path(robot_name="arm") is False

        module._world_monitor = MagicMock()
        module._robots = {}
        assert module.preview_path(robot_name="arm") is False

        module._robots = {"arm": ("robot_id", MagicMock(), MagicMock())}
        module._planned_paths = {"arm": []}
        assert module.preview_path(robot_name="arm") is False
