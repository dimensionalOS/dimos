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

from types import SimpleNamespace

import pytest

from dimos.manipulation.manipulation_spec import ExecutionStatus
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, IKResult, PlanningResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


@pytest.fixture
def sequence_planner(module_factory, mocker):
    module = module_factory()
    states = [JointState(name=["arm/j0"], position=[v]) for v in [0.0, 0.1, 0.2]]
    world = mocker.Mock()
    world.current_model_joint_state.return_value = states[0]
    world.planning_groups.select.return_value = SimpleNamespace(joint_names=("arm/j0",))
    mocker.patch.object(module, "_world_monitor", world)
    planner = mocker.Mock()
    planner.plan_selected_joint_path.side_effect = [
        PlanningResult(PlanningStatus.SUCCESS, path=states[:2]),
        PlanningResult(PlanningStatus.SUCCESS, path=states[1:]),
    ]
    mocker.patch.object(module, "_planner", planner)
    ik = mocker.patch.object(
        module,
        "inverse_kinematics",
        side_effect=[IKResult(IKStatus.SUCCESS, joint_state=state) for state in states[1:]],
    )
    parametrizer = mocker.Mock()
    parametrizer.materialize_plan.return_value = GeneratedPlan(
        group_ids=("manipulator",),
        trajectory=JointTrajectory(),
        path=states,
        status=PlanningStatus.SUCCESS,
    )
    mocker.patch.object(module, "_trajectory_parametrizer", parametrizer)
    return module, states, planner, ik, parametrizer


def test_sequence_plans_from_each_endpoint_before_materializing(sequence_planner):
    module, states, planner, ik, parametrizer = sequence_planner

    result = module.plan_pose_sequence([PoseStamped(), PoseStamped()], "manipulator")

    assert result.succeeded
    assert ik.call_args_list[1].kwargs["seed"] is states[1]
    assert planner.plan_selected_joint_path.call_args_list[1].kwargs["start"] is states[1]
    module._control_coordinator.execute_trajectory.assert_not_called()
    assert parametrizer.materialize_plan.call_args.kwargs["result"].path == states


def test_sequence_failure_never_exposes_a_partial_plan(sequence_planner):
    module, states, planner, ik, parametrizer = sequence_planner
    planner.plan_selected_joint_path.side_effect = [
        PlanningResult(PlanningStatus.SUCCESS, path=states[:2]),
        PlanningResult(PlanningStatus.NO_SOLUTION, message="blocked"),
    ]

    result = module.plan_pose_sequence([PoseStamped(), PoseStamped()], "manipulator")

    assert not result.succeeded
    assert ik.call_args_list[1].kwargs["seed"] is states[1]
    parametrizer.materialize_plan.assert_not_called()
    assert module.execute().status is ExecutionStatus.NO_PLAN
    module._control_coordinator.execute_trajectory.assert_not_called()


@pytest.mark.parametrize(
    "status", [ExecutionStatus.IDLE, ExecutionStatus.COMPLETED, ExecutionStatus.ABORTED]
)
def test_shutdown_does_not_call_a_coordinator_after_terminal_execution(
    module_factory, mocker, status
):
    module = module_factory()
    manager = mocker.Mock(status=status)
    mocker.patch.object(module, "_execution_manager", manager)

    module.stop()

    manager.cancel.assert_not_called()


def test_shutdown_cancels_active_execution(module_factory, mocker):
    module = module_factory()
    manager = mocker.Mock(status=ExecutionStatus.EXECUTING)
    mocker.patch.object(module, "_execution_manager", manager)

    module.stop()

    manager.cancel.assert_called_once_with()
