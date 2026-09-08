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

"""Tests for ManipulationModule plan-execution result projection."""

from unittest.mock import MagicMock

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationState
from dimos.manipulation.manipulation_spec import ExecutionStatus
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def _plan(final_position: float = 1.0) -> GeneratedPlan:
    names = ["arm/j0"]
    trajectory = JointTrajectory(
        joint_names=names,
        points=[
            TrajectoryPoint(
                positions=[0.0],
                velocities=[0.0],
                time_from_start=0.0,
            ),
            TrajectoryPoint(
                positions=[final_position],
                velocities=[0.0],
                time_from_start=1.0,
            ),
        ],
    )
    return GeneratedPlan(
        group_ids=("manipulator",),
        trajectory=trajectory,
        path=[
            JointState(name=names, position=[0.0]),
            JointState(name=names, position=[final_position]),
        ],
        status=PlanningStatus.SUCCESS,
    )


def _module_with_coordinator(
    coordinator: MagicMock,
    module_factory,
) -> ManipulationModule:
    module = module_factory(coordinator)
    return module


def _coordinator(
    *,
    execute_status: TrajectoryExecutionStatus = TrajectoryExecutionStatus.ACCEPTED,
    cancel_status: TrajectoryCancellationStatus = (TrajectoryCancellationStatus.ALREADY_STOPPED),
) -> MagicMock:
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(execute_status)
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(cancel_status)
    return coordinator


def test_execute_consumes_cached_plan_after_dispatch(
    module_factory,
) -> None:
    coordinator = _coordinator()
    module = _module_with_coordinator(coordinator, module_factory)
    plan = _plan()
    module._last_plan = plan

    assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED
    assert module.execute(blocking=False).status is ExecutionStatus.NO_PLAN
    assert module._last_plan is None
    coordinator.execute_trajectory.assert_called_once_with(plan.trajectory)


def test_known_coordinator_rejection_restores_previous_state(
    module_factory,
) -> None:
    coordinator = _coordinator(execute_status=TrajectoryExecutionStatus.START_STATE_MISMATCH)
    module = _module_with_coordinator(coordinator, module_factory)
    module._last_plan = _plan()
    module._state = ManipulationState.COMPLETED

    assert module.execute(blocking=False).status is ExecutionStatus.REJECTED

    assert module._state is ManipulationState.IDLE
    assert module._last_plan is None


def test_uncertain_execute_projects_to_fault(module_factory) -> None:
    coordinator = _coordinator()
    coordinator.execute_trajectory.side_effect = TimeoutError("timed out")
    module = _module_with_coordinator(coordinator, module_factory)
    module._last_plan = _plan()

    assert module.execute(blocking=False).status is ExecutionStatus.UNCERTAIN

    assert module._state is ManipulationState.FAULT
    assert "timed out" in module.get_error()


def test_uncertain_cancel_projects_to_fault(module_factory) -> None:
    coordinator = _coordinator()
    coordinator.cancel_trajectory.side_effect = TimeoutError("timed out")
    module = _module_with_coordinator(coordinator, module_factory)
    module._state = ManipulationState.EXECUTING

    assert module.cancel().status is ExecutionStatus.UNCERTAIN

    assert module._state is ManipulationState.FAULT
    assert "timed out" in module.get_error()
