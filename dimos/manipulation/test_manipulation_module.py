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

import importlib.util
import threading
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.manipulation import manipulation_module as module_impl
from dimos.manipulation._test_manipulation_helpers import FakeCoordinatorGateway
from dimos.manipulation.execution_runtime import (
    ExecutionRuntime,
    LifecycleState,
    Outcome,
    PreparedPlan,
)
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted


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
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"),
                base_link="link_base",
                tip_link="link7",
            )
        ],
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
        visualization={"backend": "none"},
    )
    gateway = FakeCoordinatorGateway()
    mod._runtime_factory = lambda _factory, **kwargs: ExecutionRuntime(lambda: gateway, **kwargs)
    mod._runtime_gateway = gateway
    mod.coordinator_joint_state = None
    mod.objects = None
    mod.start()
    yield mod
    mod.stop()


def test_execute_explicit_plan_cannot_dispatch_replaced_ready_plan(monkeypatch) -> None:
    """A READY replacement between observation and submission cannot switch plans."""

    plan_a = object()
    plan_b = object()
    prepared_a = object()
    ready_b = PreparedPlan(generated_plan=plan_b, entries=())  # type: ignore[arg-type]
    ready_a = PreparedPlan(generated_plan=plan_a, entries=())  # type: ignore[arg-type]
    dispatched: list[object] = []
    ready_dispatches: list[object] = []

    class FakeRuntime:
        def snapshot(self):
            # Model the replacement at precisely the old snapshot/command seam.
            self.current_ready = ready_b
            return SimpleNamespace(ready_plan=ready_a)

        def execute_ready(self):
            ready_dispatches.append(self.current_ready.generated_plan)
            return SimpleNamespace(accepted=True, value=object())

        def execute_explicit(self, prepared):
            dispatched.append(prepared)
            return SimpleNamespace(accepted=True, value=object())

    runtime = FakeRuntime()
    module = object.__new__(ManipulationModule)
    module._execution_runtime = runtime
    module._execution_topology = object()
    monkeypatch.setattr(module_impl, "prepare_generated_plan", lambda plan, topology: prepared_a)

    assert module._submit_execution(plan_a) is not None
    assert dispatched == [prepared_a]
    assert ready_dispatches == []


def test_execute_waits_for_its_correlated_dispatch_result() -> None:
    released = threading.Event()
    entered = threading.Event()
    expected_handle = object()
    waits: list[object] = []

    class FakeRuntime:
        def execute_ready(self):
            return SimpleNamespace(accepted=True, value=expected_handle)

        def wait_for_dispatch(self, handle, *, timeout):
            waits.append(handle)
            entered.set()
            assert released.wait(1)
            return SimpleNamespace(
                accepted=True,
                value=SimpleNamespace(outcome=Outcome.ACCEPTED),
            )

    module = object.__new__(ManipulationModule)
    module._execution_runtime = FakeRuntime()
    module._execution_topology = object()
    result: list[bool] = []
    caller = threading.Thread(target=lambda: result.append(module.execute()))
    caller.start()
    assert entered.wait(1)
    assert result == []
    released.set()
    caller.join(1)
    assert result == [True]
    assert waits == [expected_handle]


def test_preview_waits_for_terminal_before_reporting_completion(monkeypatch) -> None:
    released = threading.Event()
    entered = threading.Event()
    expected_handle = object()
    preview_calls: list[tuple[object, object]] = []

    class FakeRuntime:
        def execute_explicit(self, prepared):
            return SimpleNamespace(accepted=True, value=expected_handle)

        def wait_for_dispatch(self, handle, *, timeout):
            return SimpleNamespace(
                accepted=True,
                value=SimpleNamespace(outcome=Outcome.ACCEPTED),
            )

        def wait_for_terminal(self, handle, *, timeout):
            entered.set()
            assert released.wait(1)
            return SimpleNamespace(
                accepted=True,
                value=SimpleNamespace(outcome=Outcome.COMPLETED, diagnostic=""),
            )

    plan = object()
    ready = PreparedPlan(generated_plan=plan, entries=())  # type: ignore[arg-type]
    runtime = FakeRuntime()
    module = object.__new__(ManipulationModule)
    module._execution_runtime = runtime
    module._execution_topology = object()
    module.preview_path = lambda duration=None, robot_name=None, target_fps=30: (
        preview_calls.append((duration, robot_name)) or True
    )
    monkeypatch.setattr(module_impl, "prepare_generated_plan", lambda value, topology: value)

    # The preview helper obtains the READY generated plan only to bind it through
    # the runtime's explicit-plan path; terminal publication is the completion gate.
    runtime.snapshot = lambda: SimpleNamespace(ready_plan=ready)
    result: list[Any] = []
    caller = threading.Thread(target=lambda: result.append(module._preview_execute_wait()))
    caller.start()
    assert entered.wait(1)
    assert result == []
    assert preview_calls == [(0.5, None)]
    released.set()
    caller.join(1)
    assert len(result) == 1 and result[0].is_success()


def test_execution_projection_helpers_use_runtime_snapshot_only() -> None:
    snapshot = SimpleNamespace(
        state=LifecycleState.READY,
        diagnostic="runtime diagnostic",
        ready_plan_id="ready-1",
        ready_plan=PreparedPlan(generated_plan=SimpleNamespace(path=[]), entries=()),
        operation=None,
    )

    class FakeRuntime:
        def __init__(self) -> None:
            self.snapshot_calls = 0

        def snapshot(self):
            self.snapshot_calls += 1
            return snapshot

    runtime = FakeRuntime()
    module = object.__new__(ManipulationModule)
    module._execution_runtime = runtime
    module._execution_topology = None

    assert module.get_state() == LifecycleState.READY.name
    assert module.get_error() == "runtime diagnostic"
    assert module.has_planned_path() is False
    projection = module.get_execution_snapshot()
    assert projection.ready_plan_id == "ready-1"
    assert projection.has_ready_plan is True
    assert runtime.snapshot_calls == 4
    assert not hasattr(module, "_execution_state")
    assert not hasattr(module, "_execution_error")


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestManipulationModuleIntegration:
    """Integration tests for ManipulationModule with real Drake backend."""

    def test_module_initialization(self, module):
        """Test module initializes with real Drake world."""
        assert module.get_state() == LifecycleState.IDLE.name
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
        assert module.get_state() == LifecycleState.READY.name
        assert module.has_planned_path() is True

        ready = module._execution_runtime.snapshot().ready_plan
        assert isinstance(ready, PreparedPlan)
        assert len(ready.generated_plan.trajectory.points) > 1
        assert ready.generated_plan.trajectory.duration > 0
        assert ready.generated_plan.group_ids == ("test_arm/manipulator",)

    def test_plan_to_explicit_joint_target(self, module, joint_state_zeros):
        """Test planning to an explicit planning-group joint target."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joint_targets(
            {"test_arm/manipulator": JointState(position=[0.05] * 7)}
        )

        assert success is True
        assert module.get_state() == LifecycleState.READY.name
        ready = module._execution_runtime.snapshot().ready_plan
        assert isinstance(ready, PreparedPlan)
        assert ready.generated_plan.group_ids == ("test_arm/manipulator",)
        assert module.has_planned_path() is True

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
        groups = info["planning_groups"]
        assert len(groups) == 1
        assert groups[0].id == "test_arm/manipulator"

        all_groups = module.list_planning_groups()
        assert [group.id for group in all_groups] == ["test_arm/manipulator"]

    def test_ee_pose(self, module, joint_state_zeros):
        """Test getting end-effector pose."""
        module._on_joint_state(joint_state_zeros)

        pose = module.get_ee_pose()

        assert pose is not None
        assert hasattr(pose, "x")
        assert hasattr(pose, "y")
        assert hasattr(pose, "z")

    def test_prepared_plan_contains_coordinator_joint_names(self, module, joint_state_zeros):
        """Runtime materialization translates selected joints for the gateway."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True

        ready = module._execution_runtime.snapshot().ready_plan
        assert isinstance(ready, PreparedPlan)
        assert ready.entries[0].request["trajectory"].joint_names == list(
            module._robots["test_arm"][1].joint_name_mapping.values()
        )


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestCoordinatorIntegration:
    """Test coordinator integration through the runtime gateway."""

    def test_execute_with_mock_coordinator(self, module, joint_state_zeros):
        """Test execute sends trajectory to coordinator."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True

        result = module.execute()

        assert result is True
        assert module.get_state() == LifecycleState.RUNNING.name

        task_name, request = module._runtime_gateway.execute_calls[0]

        assert task_name == "traj_arm"
        trajectory = request["trajectory"]
        assert len(trajectory.points) > 1
        robot_config = module._robots["test_arm"][1]
        assert trajectory.joint_names == list(robot_config.joint_name_mapping.values())

    def test_execute_rejected_by_coordinator(self, module, joint_state_zeros):
        """Test handling of coordinator rejection."""
        module._on_joint_state(joint_state_zeros)

        module.plan_to_joints(JointState(position=[0.05] * 7))

        module._runtime_gateway.execute_outcome = Outcome.REJECTED

        result = module.execute()

        assert result is False
        assert module.get_state() == LifecycleState.IDLE.name
        assert "rejected" in module.get_error().lower()

    def test_state_transitions_during_execution(self, module, joint_state_zeros):
        """Test state transitions during plan and execute."""
        assert module.get_state() == LifecycleState.IDLE.name

        module._on_joint_state(joint_state_zeros)

        # Plan commits a runtime-owned READY plan.
        module.plan_to_joints(JointState(position=[0.05] * 7))
        assert module.get_state() == LifecycleState.READY.name

        # Reset clears the READY plan.
        module.reset()
        assert module.get_state() == LifecycleState.IDLE.name

        # Plan again
        module.plan_to_joints(JointState(position=[0.05] * 7))

        # Execute is accepted by the runtime gateway.
        module.execute()
        assert module.get_state() == LifecycleState.RUNNING.name
