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

"""Tests for the single-owner manipulation execution lifecycle."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationState
from dimos.manipulation.manipulation_spec import ExecutionStatus
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.assets.model import RobotModel


def _plan() -> GeneratedPlan:
    names = ["arm/j0"]
    return GeneratedPlan(
        group_ids=("arm/manipulator",),
        trajectory=JointTrajectory(
            joint_names=names,
            points=[
                TrajectoryPoint(positions=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[1.0], time_from_start=1.0),
            ],
        ),
        status=PlanningStatus.SUCCESS,
    )


@pytest.fixture
def module(module_factory) -> ManipulationModule:
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.IDLE)
    manipulation = module_factory(coordinator)
    config = RobotModelConfig(
        name="arm",
        model=RobotModel.from_file(Path("/path/to/robot.urdf")),
        joint_names=["j0"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("j0",),
                base_link="base",
                tip_link="tool",
            )
        ],
    )
    manipulation._robots = {"arm": ("arm_id", config)}
    manipulation.config = manipulation.config.model_copy(update={"robots": [config]})
    manipulation._initialize_execution()
    return manipulation


def test_execute_without_pending_plan_is_rejected(module) -> None:
    result = module.execute(blocking=False)

    assert result.status is ExecutionStatus.NO_PLAN


def test_nonblocking_execute_returns_acceptance_and_consumes_plan(module) -> None:
    module._last_plan = _plan()

    accepted = module.execute(blocking=False)
    repeated = module.execute(blocking=False)

    assert accepted.status is ExecutionStatus.ACCEPTED
    assert repeated.status is ExecutionStatus.NO_PLAN
    module._control_coordinator.execute_trajectory.assert_called_once()


def test_rejected_dispatch_also_consumes_plan(module) -> None:
    module._control_coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.START_STATE_MISMATCH
    )
    module._last_plan = _plan()

    rejected = module.execute(blocking=False)
    repeated = module.execute(blocking=False)

    assert rejected.status is ExecutionStatus.REJECTED
    assert repeated.status is ExecutionStatus.NO_PLAN
    assert module._state is ManipulationState.IDLE


def test_blocking_execute_waits_for_terminal_jtt_status(module) -> None:
    module._last_plan = _plan()
    module._control_coordinator.task_invoke.side_effect = [
        TrajectoryStatus(state=TrajectoryState.EXECUTING, progress=0.5),
        TrajectoryStatus(state=TrajectoryState.COMPLETED, progress=1.0),
    ]

    result = module.execute(blocking=True, timeout=1.0)

    assert result.status is ExecutionStatus.COMPLETED
    assert result.trajectory_status is not None
    assert module._state is ManipulationState.COMPLETED


def test_execution_timeout_does_not_cancel_motion(module) -> None:
    module._control_coordinator.task_invoke.return_value = TrajectoryStatus(
        state=TrajectoryState.EXECUTING
    )
    module._last_plan = _plan()

    result = module.execute(blocking=True, timeout=0.0)

    assert result.status is ExecutionStatus.TIMED_OUT
    module._control_coordinator.cancel_trajectory.assert_not_called()
    assert module._execution_manager.status is ExecutionStatus.EXECUTING


def test_wait_preserves_aborted_jtt_terminal_state(module) -> None:
    module._last_plan = _plan()
    module.execute(blocking=False)

    module._control_coordinator.task_invoke.return_value = TrajectoryStatus(
        state=TrajectoryState.ABORTED
    )
    result = module.wait_for_execution(timeout=0.0)

    assert result.status is ExecutionStatus.ABORTED
    assert module._state is ManipulationState.IDLE


def test_uncertain_dispatch_faults_and_consumes_plan(module) -> None:
    module._control_coordinator.execute_trajectory.side_effect = TimeoutError("timed out")
    module._last_plan = _plan()

    result = module.execute(blocking=False)

    assert result.status is ExecutionStatus.UNCERTAIN
    assert module._state is ManipulationState.FAULT
    assert module._last_plan is None
    assert "timed out" in module.get_error()


def test_safe_cancel_clears_uncertain_dispatch_fault(module) -> None:
    module._control_coordinator.execute_trajectory.side_effect = TimeoutError("timed out")
    module._last_plan = _plan()
    module.execute(blocking=False)

    result = module.cancel()
    snapshot = module.get_state()

    assert result.status is ExecutionStatus.NO_EXECUTION
    assert snapshot.operation_status.name == "IDLE"
    assert snapshot.execution_status is ExecutionStatus.NO_EXECUTION
    assert snapshot.error is None


def test_uncertain_cancel_faults_module(module) -> None:
    module._control_coordinator.cancel_trajectory.side_effect = TimeoutError("timed out")

    result = module.cancel()

    assert result.status is ExecutionStatus.UNCERTAIN
    assert module._state is ManipulationState.FAULT
    assert "timed out" in module.get_error()
