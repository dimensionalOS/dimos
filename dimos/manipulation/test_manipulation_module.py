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

"""
Integration tests for ManipulationModule.

These tests verify the full planning stack with Drake backend.
They require Drake to be installed and will be skipped otherwise.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import importlib.util
from pathlib import Path
import threading
import time
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationState,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.data import get_data


def _drake_available() -> bool:
    return importlib.util.find_spec("pydrake") is not None


def _xarm_urdf_available() -> bool:
    try:
        desc_path = get_data("xarm_description")
        model_path = desc_path / "urdf/xarm_device.urdf.xacro"
        return model_path.exists()
    except Exception:
        return False


def _get_xarm7_config() -> RobotModelConfig:
    """Create XArm7 robot config for testing."""
    desc_path = get_data("xarm_description")
    return RobotModelConfig(
        name="test_arm",
        model_path=desc_path / "urdf/xarm_device.urdf.xacro",
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        end_effector_link="link7",
        base_link="link_base",
        package_paths={"xarm_description": desc_path},
        xacro_args={"dof": "7", "limited": "true"},
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping={
            "arm/joint1": "joint1",
            "arm/joint2": "joint2",
            "arm/joint3": "joint3",
            "arm/joint4": "joint4",
            "arm/joint5": "joint5",
            "arm/joint6": "joint6",
            "arm/joint7": "joint7",
        },
        coordinator_task_name="traj_arm",
    )


@pytest.fixture
def xarm7_config():
    return _get_xarm7_config()


@pytest.fixture
def joint_state_zeros():
    """Create a JointState message with zeros for XArm7."""
    return JointState(
        name=[
            "arm/joint1",
            "arm/joint2",
            "arm/joint3",
            "arm/joint4",
            "arm/joint5",
            "arm/joint6",
            "arm/joint7",
        ],
        position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


@pytest.fixture
def module(xarm7_config):
    """Create a started ManipulationModule with ports disabled."""
    mod = ManipulationModule(
        robots=[xarm7_config],
        planning_timeout=10.0,
        enable_viz=False,
    )
    mod.joint_state = None
    mod.objects = None
    mod.start()
    yield mod
    mod.stop()


class _FakeWorld:
    def __init__(self) -> None:
        self.animated: list[tuple[str, int, float]] = []

    def animate_path(self, robot_id: str, path: list[JointState], duration: float) -> None:
        self.animated.append((robot_id, len(path), duration))


class _FakeWorldMonitor:
    def __init__(self) -> None:
        self.world = _FakeWorld()
        self.dismissed: list[str] = []
        self._states = {
            "left_id": JointState(name=["j1", "j2"], position=[0.0, 0.0]),
            "right_id": JointState(name=["j1", "j2"], position=[0.0, 0.0]),
        }

    def dismiss_preview(self, robot_id: str) -> None:
        self.dismissed.append(robot_id)

    def get_current_joint_state(self, robot_id: str) -> JointState | None:
        return self._states.get(robot_id)

    def stop_all_monitors(self) -> None:
        pass


class _PlanControl:
    def __init__(self, release: threading.Event | None = None, succeed: bool = True) -> None:
        self.started = threading.Event()
        self.release = release or threading.Event()
        self.succeed = succeed


class _GatedPlanner:
    def __init__(self, controls: list[_PlanControl]) -> None:
        self.controls = controls
        self.calls: list[tuple[str, JointState]] = []
        self._lock = threading.Lock()

    def plan_joint_path(
        self,
        world: object,
        robot_id: str,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        with self._lock:
            call_index = len(self.calls)
            self.calls.append((robot_id, goal))
            control = self.controls[call_index]

        control.started.set()
        if not control.release.wait(timeout=2.0):
            raise TimeoutError(f"test planner was not released for {robot_id}")

        if not control.succeed:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="synthetic planning failure",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[start, goal],
            planning_time=0.01,
            path_length=1.0,
            iterations=1,
            message="synthetic success",
        )


def _mock_robot_config(name: str) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/tmp/mock.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j1", "j2"],
        end_effector_link="tool",
        coordinator_task_name=f"{name}_task",
    )


@pytest.fixture
def async_module():
    """Create a ManipulationModule wired with fake world/planner components."""
    mod = ManipulationModule(robots=[], planning_timeout=1.0, enable_viz=False)
    mod._world_monitor = _FakeWorldMonitor()
    mod._planning_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="test-plan")
    for name, robot_id in (("left_arm", "left_id"), ("right_arm", "right_id")):
        config = _mock_robot_config(name)
        mod._robots[name] = (
            robot_id,
            config,
            JointTrajectoryGenerator(num_joints=2, max_velocity=1.0, max_acceleration=2.0),
        )
    try:
        yield mod
    finally:
        mod.stop()


class TestAsyncPlanningUnit:
    """Deterministic unit tests for ManipulationModule async planning semantics."""

    def test_plan_accepts_quickly_and_path_appears_after_completion(self, async_module):
        control = _PlanControl()
        async_module._planner = _GatedPlanner([control])

        start = time.monotonic()
        accepted = async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        elapsed = time.monotonic() - start

        assert accepted is True
        assert elapsed < 0.5
        assert control.started.wait(timeout=0.5)
        assert async_module.get_state() == ManipulationState.PLANNING.name
        assert async_module.has_planned_path("left_arm") is False

        control.release.set()

        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)
        status = async_module.get_planning_status("left_arm")
        assert status["active"] is False
        assert status["done"] is True
        assert status["success"] is True
        assert status["duration_s"] is not None
        assert async_module.get_state() == ManipulationState.COMPLETED.name
        assert async_module.has_planned_path("left_arm") is True

    def test_distinct_robots_can_have_active_plans_simultaneously(self, async_module):
        release = threading.Event()
        left = _PlanControl(release=release)
        right = _PlanControl(release=release)
        planner = _GatedPlanner([left, right])
        async_module._planner = planner

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.2, 0.2]), "right_arm"
        )

        assert left.started.wait(timeout=0.5)
        assert right.started.wait(timeout=0.5)
        assert len(planner.calls) == 2
        assert async_module.get_planning_status("left_arm")["active"] is True
        assert async_module.get_planning_status("right_arm")["active"] is True

        release.set()

        assert async_module.wait_for_planning_completion(None, timeout=1.0)
        assert async_module.has_planned_path("left_arm") is True
        assert async_module.has_planned_path("right_arm") is True

    def test_plan_for_other_robot_is_accepted_while_one_robot_is_planning(self, async_module):
        left = _PlanControl()
        right = _PlanControl()
        async_module._planner = _GatedPlanner([left, right])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert left.started.wait(timeout=0.5)

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.2, 0.2]), "right_arm"
        )
        assert right.started.wait(timeout=0.5)

        left.release.set()
        right.release.set()
        assert async_module.wait_for_planning_completion(None, timeout=1.0)

    def test_same_robot_rejects_duplicate_active_plan(self, async_module):
        control = _PlanControl()
        async_module._planner = _GatedPlanner([control])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert control.started.wait(timeout=0.5)

        duplicate = async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.2, 0.2]), "left_arm"
        )

        assert duplicate is False
        control.release.set()
        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)

    def test_failed_planning_sets_fault_and_records_error(self, async_module):
        control = _PlanControl(succeed=False)
        async_module._planner = _GatedPlanner([control])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert control.started.wait(timeout=0.5)
        control.release.set()

        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)
        status = async_module.get_planning_status("left_arm")
        assert status["success"] is False
        assert "Planning failed" in status["error"]
        assert async_module.get_state() == ManipulationState.FAULT.name
        assert "Planning failed" in async_module.get_error()

    def test_fault_rejects_new_plans_until_reset(self, async_module):
        failed = _PlanControl(succeed=False)
        after_reset = _PlanControl()
        async_module._planner = _GatedPlanner([failed, after_reset])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert failed.started.wait(timeout=0.5)
        failed.release.set()
        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)
        assert async_module.get_state() == ManipulationState.FAULT.name

        assert (
            async_module.plan_to_joints(
                JointState(name=["j1", "j2"], position=[0.2, 0.2]), "right_arm"
            )
            is False
        )

        assert async_module.reset().startswith("Reset")
        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.2, 0.2]), "right_arm"
        )
        assert after_reset.started.wait(timeout=0.5)
        after_reset.release.set()
        assert async_module.wait_for_planning_completion("right_arm", timeout=1.0)
        assert async_module.get_state() == ManipulationState.COMPLETED.name

    def test_late_invalidated_job_does_not_overwrite_newer_plan(self, async_module):
        first = _PlanControl()
        second = _PlanControl()
        async_module._planner = _GatedPlanner([first, second])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert first.started.wait(timeout=0.5)
        first_status = async_module.get_planning_status("left_arm")
        assert first_status["active"] is True

        assert async_module.reset().startswith("Reset")
        assert async_module.get_state() == ManipulationState.IDLE.name
        assert async_module.get_planning_status("left_arm")["invalidated"] is True

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.2, 0.2]), "left_arm"
        )
        assert second.started.wait(timeout=0.5)
        second_request_id = async_module.get_planning_status("left_arm")["request_id"]
        assert second_request_id != first_status["request_id"]

        first.release.set()
        second.release.set()

        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)
        status = async_module.get_planning_status("left_arm")
        assert status["request_id"] == second_request_id
        assert status["success"] is True
        assert async_module._planned_paths["left_arm"][-1].position == [0.2, 0.2]
        assert async_module._planned_trajectories["left_arm"].points[-1].positions == [0.2, 0.2]

    def test_preview_and_execute_reject_only_robot_with_active_plan(self, async_module):
        right_done = _PlanControl()
        left_active = _PlanControl()
        async_module._planner = _GatedPlanner([right_done, left_active])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.3, 0.3]), "right_arm"
        )
        assert right_done.started.wait(timeout=0.5)
        right_done.release.set()
        assert async_module.wait_for_planning_completion("right_arm", timeout=1.0)

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert left_active.started.wait(timeout=0.5)

        coordinator = MagicMock()
        coordinator.task_invoke.return_value = True
        async_module._coordinator_client = coordinator

        assert async_module.preview_path(0.1, "left_arm") is False
        assert async_module.execute("left_arm") is False
        assert async_module.preview_path(0.1, "right_arm") is True
        assert async_module.execute("right_arm") is True

        left_active.release.set()
        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)

    def test_clear_planned_path_rejects_active_plan(self, async_module):
        control = _PlanControl()
        async_module._planner = _GatedPlanner([control])

        assert async_module.plan_to_joints(
            JointState(name=["j1", "j2"], position=[0.1, 0.1]), "left_arm"
        )
        assert control.started.wait(timeout=0.5)

        assert async_module.clear_planned_path("left_arm") is False

        control.release.set()
        assert async_module.wait_for_planning_completion("left_arm", timeout=1.0)
        assert async_module.has_planned_path("left_arm") is True
        assert async_module.clear_planned_path("left_arm") is True
        assert async_module.has_planned_path("left_arm") is False


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestManipulationModuleIntegration:
    """Integration tests for ManipulationModule with real Drake backend."""

    def test_module_initialization(self, module):
        """Test module initializes with real Drake world."""
        assert module._state == ManipulationState.IDLE
        assert module._world_monitor is not None
        assert module._planner is not None
        assert module._kinematics is not None
        assert "test_arm" in module._robots

    def test_joint_state_sync(self, module, joint_state_zeros):
        """Test joint state synchronization to Drake world."""
        module._on_joint_state(joint_state_zeros)

        joints = module.get_current_joints()
        assert joints is not None
        assert len(joints) == 7
        assert all(abs(j) < 0.01 for j in joints)

    def test_collision_check(self, module, joint_state_zeros):
        """Test collision checking at a configuration."""
        module._on_joint_state(joint_state_zeros)

        is_free = module.is_collision_free([0.0] * 7)
        assert is_free is True

    def test_plan_to_joints(self, module, joint_state_zeros):
        """Test planning to a joint configuration."""
        module._on_joint_state(joint_state_zeros)

        target = JointState(position=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        success = module.plan_to_joints(target)

        assert success is True
        assert module.wait_for_planning_completion(timeout=15.0) is True
        assert module.get_state() == ManipulationState.COMPLETED.name
        assert module.has_planned_path() is True

        assert "test_arm" in module._planned_trajectories
        traj = module._planned_trajectories["test_arm"]
        assert len(traj.points) > 1
        assert traj.duration > 0

    def test_add_and_remove_obstacle(self, module, joint_state_zeros):
        """Test adding and removing obstacles."""
        module._on_joint_state(joint_state_zeros)

        pose = Pose(
            position=Vector3(0.5, 0.0, 0.3),
            orientation=Quaternion(),  # default is identity (w=1)
        )
        obstacle_id = module.add_obstacle("test_box", pose, "box", [0.1, 0.1, 0.1])

        assert obstacle_id != ""
        assert obstacle_id is not None

        removed = module.remove_obstacle(obstacle_id)
        assert removed is True

    def test_robot_info(self, module):
        """Test getting robot information."""
        info = module.get_robot_info()

        assert info is not None
        assert info["name"] == "test_arm"
        assert len(info["joint_names"]) == 7
        assert info["end_effector_link"] == "link7"
        assert info["coordinator_task_name"] == "traj_arm"
        assert info["has_joint_name_mapping"] is True

    def test_ee_pose(self, module, joint_state_zeros):
        """Test getting end-effector pose."""
        module._on_joint_state(joint_state_zeros)

        pose = module.get_ee_pose()

        assert pose is not None
        assert hasattr(pose, "x")
        assert hasattr(pose, "y")
        assert hasattr(pose, "z")

    def test_trajectory_name_translation(self, module, joint_state_zeros):
        """Test that trajectory joint names are translated for coordinator."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True
        assert module.wait_for_planning_completion(timeout=15.0) is True

        traj = module._planned_trajectories["test_arm"]
        robot_config = module._robots["test_arm"][1]

        translated = module._translate_trajectory_to_coordinator(traj, robot_config)

        for name in translated.joint_names:
            assert name.startswith("arm_")  # Should have arm_ prefix


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestCoordinatorIntegration:
    """Test coordinator integration with mocked RPC client."""

    def test_execute_with_mock_coordinator(self, module, joint_state_zeros):
        """Test execute sends trajectory to coordinator."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True
        assert module.wait_for_planning_completion(timeout=15.0) is True

        # Mock the coordinator client
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        result = module.execute()

        assert result is True
        assert module.get_state() == ManipulationState.COMPLETED.name

        # Verify coordinator was called
        mock_client.task_invoke.assert_called_once()
        call_args = mock_client.task_invoke.call_args
        task_name, method_name, kwargs = call_args[0]

        assert task_name == "traj_arm"
        assert method_name == "execute"
        trajectory = kwargs["trajectory"]
        assert len(trajectory.points) > 1
        # Joint names should be translated
        assert all(n.startswith("arm_") for n in trajectory.joint_names)

    def test_execute_rejected_by_coordinator(self, module, joint_state_zeros):
        """Test handling of coordinator rejection."""
        module._on_joint_state(joint_state_zeros)

        module.plan_to_joints(JointState(position=[0.05] * 7))
        assert module.wait_for_planning_completion(timeout=15.0) is True

        # Mock coordinator to reject
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        result = module.execute()

        assert result is False
        assert module.get_state() == ManipulationState.FAULT.name
        assert "rejected" in module._error_message.lower()

    def test_state_transitions_during_execution(self, module, joint_state_zeros):
        """Test state transitions during plan and execute."""
        assert module._state == ManipulationState.IDLE

        module._on_joint_state(joint_state_zeros)

        # Plan - should go through PLANNING -> COMPLETED
        module.plan_to_joints(JointState(position=[0.05] * 7))
        assert module.wait_for_planning_completion(timeout=15.0) is True
        assert module.get_state() == ManipulationState.COMPLETED.name

        # Reset works from COMPLETED
        module.reset()
        assert module.get_state() == ManipulationState.IDLE.name

        # Plan again
        module.plan_to_joints(JointState(position=[0.05] * 7))
        assert module.wait_for_planning_completion(timeout=15.0) is True

        # Mock coordinator
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        # Execute - should go to EXECUTING then COMPLETED
        module.execute()
        assert module.get_state() == ManipulationState.COMPLETED.name
