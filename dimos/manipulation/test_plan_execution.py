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
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.manipulation.visualization.operator import ManipulationOperator
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.assets.model import PlanarBaseDefinition


def _plan(final_position: float = 1.0, joint_name: str = "arm/j0") -> GeneratedPlan:
    names = [joint_name]
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


def test_execute_rejects_planar_base_plan_but_keeps_it_for_preview(
    module_factory,
) -> None:
    coordinator = _coordinator()
    module = _module_with_coordinator(coordinator, module_factory)
    planar_base = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    module.config.model.model = module.config.model.model.with_planar_base(planar_base)
    plan = _plan(joint_name=planar_base.joint_names[0])
    module._last_plan = plan
    module._state = ManipulationState.COMPLETED

    result = module.execute(blocking=False)

    assert result.status is ExecutionStatus.REJECTED
    assert "planning and preview only" in result.message
    assert module._last_plan is plan
    assert module._state is ManipulationState.COMPLETED
    coordinator.execute_trajectory.assert_not_called()


def test_execute_sends_planar_base_columns_to_the_base_task(module_factory) -> None:
    coordinator = _coordinator()
    state = TrajectoryState.EXECUTING
    coordinator.task_invoke.side_effect = lambda task, method, args: (
        TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
        if method == "execute"
        else TrajectoryStatus(state=state)
    )
    module = _module_with_coordinator(coordinator, module_factory)
    planar_base = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    module.config.model.model = module.config.model.model.with_planar_base(planar_base)
    module.config.model.joint_names = [*planar_base.joint_names, "arm/j0"]
    module.config.base_trajectory_task = "base_traj"
    module._initialize_execution()
    names = ["arm/j0", *planar_base.joint_names]
    points = [
        TrajectoryPoint(positions=[0.0] * 4, time_from_start=0.0),
        TrajectoryPoint(positions=[1.0] * 4, time_from_start=1.0),
    ]
    module._last_plan = GeneratedPlan(
        group_ids=("manipulator",),
        trajectory=JointTrajectory(joint_names=names, points=points),
        path=[JointState(name=names, position=point.positions) for point in points],
        status=PlanningStatus.SUCCESS,
    )

    assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED

    assert coordinator.execute_trajectory.call_args.args[0].joint_names == ["arm/j0"]
    task, _, args = next(
        c.args for c in coordinator.task_invoke.call_args_list if c.args[1] == "execute"
    )
    assert task == "base_traj"
    assert args["trajectory"].joint_names == list(planar_base.joint_names)
    state = TrajectoryState.COMPLETED
    assert module.wait_for_execution(timeout=1.0).status is ExecutionStatus.COMPLETED


def test_execute_allows_arm_only_plan_from_planar_model(module_factory) -> None:
    coordinator = _coordinator()
    module = _module_with_coordinator(coordinator, module_factory)
    planar_base = PlanarBaseDefinition(
        velocity_limits=(1.0, 1.0, 2.0),
        acceleration_limits=(2.0, 2.0, 4.0),
    )
    module.config.model.model = module.config.model.model.with_planar_base(planar_base)
    plan = _plan()
    module._last_plan = plan

    result = module.execute(blocking=False)

    assert result.status is ExecutionStatus.ACCEPTED
    coordinator.execute_trajectory.assert_called_once_with(plan.trajectory)


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


@pytest.mark.parametrize("reader", ["operator", "snapshot"])
@pytest.mark.parametrize(
    ("terminal", "operation"),
    [
        (TrajectoryState.COMPLETED, "COMPLETED"),
        (TrajectoryState.ABORTED, "IDLE"),
        (TrajectoryState.FAULT, "FAULT"),
    ],
)
def test_status_refresh_observes_nonblocking_execution(module_factory, reader, terminal, operation):
    coordinator = _coordinator()
    module = module_factory(coordinator)
    operator = ManipulationOperator(module, MagicMock())

    def read_status():
        if reader == "operator":
            return operator.status().state
        return module.get_state().operation_status.name

    assert operator.execute(_plan()) is True
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)
    assert read_status() == "EXECUTING"

    coordinator.task_invoke.return_value = TrajectoryStatus(state=terminal)
    assert read_status() == operation
    assert module.get_state().execution_status.name == terminal.name
    coordinator.task_invoke.reset_mock()
    assert read_status() == operation
    coordinator.task_invoke.assert_not_called()

    assert operator.execute(_plan()) is True


def test_status_refresh_reports_coordinator_failure(module_factory):
    coordinator = _coordinator()
    module = module_factory(coordinator)
    operator = ManipulationOperator(module, MagicMock())
    assert operator.execute(_plan()) is True
    coordinator.task_invoke.side_effect = TimeoutError("status unavailable")

    status = operator.status()

    assert status.state == "FAULT"
    assert "status unavailable" in status.error
    assert module.get_state().execution_status is ExecutionStatus.UNCERTAIN
