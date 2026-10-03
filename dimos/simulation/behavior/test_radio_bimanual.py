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

"""Demonstration-derived ordering, per-arm grippers and fail-closed ownership."""

import math

import pytest

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
)
from dimos.manipulation.sdk import Arm
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.demo_radio import radio_blueprint
from dimos.simulation.behavior.r1pro_model import simulation_model_config
from dimos.simulation.behavior.radio_baselines import PressIntent
from dimos.simulation.behavior.radio_bimanual import (
    BimanualRadioFlow,
    BimanualRadioIntent,
    BimanualRadioManipulationModule,
    RadioPoseIntent,
)
from dimos.simulation.behavior.radio_motion import RadioCoordinator, RadioManipulationModule
from dimos.simulation.behavior.radio_policy import RadioPolicyModule
from dimos.simulation.behavior.types import TaskSelection


@pytest.fixture
def runtime(mocker):
    # Mock the external RPC transport, keeping normal module initialization.
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    module = BimanualRadioManipulationModule(
        model=simulation_model_config(), rpc_transport=ZenohRPC
    )
    module._control_coordinator = mocker.Mock()
    module._world_monitor = mocker.Mock()
    module._control_coordinator.task_invoke.return_value = True
    try:
        yield module
    finally:
        module.stop()


@pytest.mark.parametrize(
    "group,task", [("left_arm", "r1pro_left_gripper"), ("right_arm", "r1pro_right_gripper")]
)
def test_sdk_gripper_command_routes_only_to_selected_hand(runtime, group, task):
    info = PlanningGroupInfo(group, (), "world", f"{group}_tip", True)
    arm = Arm(runtime, info)
    assert arm.set_gripper_position(0.25).succeeded
    runtime._control_coordinator.task_invoke.assert_called_once_with(
        task, "set_normalized", {"values": [0.25]}
    )


@pytest.mark.parametrize("group,value", [(None, 0), ("torso", 0), ("left_arm", math.nan)])
def test_ambiguous_or_invalid_gripper_commands_rejected(runtime, group, value):
    result = runtime.set_gripper_position(value, planning_group=group)
    assert result.status is CommandStatus.REJECTED
    runtime._control_coordinator.task_invoke.assert_not_called()


def test_feedback_is_per_hand_and_torso_has_no_gripper(runtime, mocker):
    snapshot = ManipulationSnapshot(
        1,
        OperationStatus.IDLE,
        None,
        False,
        ExecutionStatus.IDLE,
        {
            group: PlanningGroupState(None, None, None)
            for group in ("left_arm", "right_arm", "torso")
        },
    )
    mocker.patch.object(RadioManipulationModule, "get_state", return_value=snapshot)
    runtime._control_coordinator.task_invoke.side_effect = [[0.25], [0.75]]
    actual = runtime.get_state()
    assert [actual.groups[g].gripper_position for g in ("left_arm", "right_arm", "torso")] == [
        0.25,
        0.75,
        None,
    ]
    assert [call.args[0] for call in runtime._control_coordinator.task_invoke.call_args_list] == [
        "r1pro_left_gripper",
        "r1pro_right_gripper",
    ]


@pytest.fixture
def flow(mocker):
    proxy = mocker.Mock()
    proxy.set_gripper_position.return_value = CommandResult(CommandStatus.SUCCEEDED)
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    right = Arm(proxy, PlanningGroupInfo("right_arm", (), "world", "right_gripper_link", True))
    left = Arm(proxy, PlanningGroupInfo("left_arm", (), "world", "left_gripper_link", True))
    right_state = mocker.patch.object(right, "state")
    left_state = mocker.patch.object(left, "state")
    right_state.side_effect = [PlanningGroupState(None, None, x) for x in (1, 0, 1)]
    left_state.return_value = PlanningGroupState(None, None, 0)
    pose = RadioPoseIntent((0.4, 0.1, 0.6), (0, 0, 0, 1), "synthetic CPU intent")
    move = mocker.Mock()
    hold = mocker.Mock(return_value=True)
    press = mocker.Mock(
        return_value=PressIntent(
            (0.45, 0.1, 0.7), (0, 0, 0, 1), (1, 0, 0), "fresh held sensor fixture"
        )
    )
    steps = mocker.Mock(side_effect=[100, 107, 108])
    runner = BimanualRadioFlow(
        right,
        left,
        BimanualRadioIntent(pose, pose, pose, pose),
        move,
        hold,
        press,
        steps,
        mocker.Mock(return_value=True),
    )
    return runner, proxy, move, hold, press, right_state


def test_grasp_hold_press_retract_replace_uses_fresh_held_target(flow):
    runner, proxy, move, _, press, _ = flow
    for _ in range(12):
        runner.tick()
    assert runner.completed
    assert [c.args[0].info.id for c in move.call_args_list] == [
        "right_arm",
        "right_arm",
        "right_arm",
        "left_arm",
        "left_arm",
        "left_arm",
        "right_arm",
    ]
    assert [c.args[3] for c in move.call_args_list] == [False, False, True, True, True, True, True]
    assert move.call_args_list[3].args[1].position == pytest.approx((0.438, 0.1, 0.7))
    assert move.call_args_list[4].args[1].position == (0.45, 0.1, 0.7)
    assert [c.kwargs["planning_group"] for c in proxy.set_gripper_position.call_args_list] == [
        "right_arm",
        "right_arm",
        "left_arm",
        "right_arm",
    ]
    assert [c.args[0] for c in proxy.set_gripper_position.call_args_list] == [1, 0, 0, 1]
    press.assert_called_once_with()
    proxy.cancel.assert_not_called()
    assert not runner.holding


def test_command_acceptance_does_not_advance_without_measured_gripper(flow):
    runner, proxy, move, _, _, state = flow
    state.side_effect = None
    state.return_value = PlanningGroupState(None, None, None)
    runner.tick()
    runner.tick()
    assert runner.index == 0
    assert proxy.set_gripper_position.call_count == 1
    move.assert_not_called()


def test_closed_fingers_do_not_count_as_physical_grasp(flow):
    runner, proxy, move, hold, press, _ = flow
    hold.return_value = False
    for _ in range(3):
        runner.tick()
    with pytest.raises(RuntimeError, match="without verified physical"):
        runner.tick()
    assert move.call_count == 2
    press.assert_not_called()
    assert runner.tick() == runner.halted
    assert proxy.set_gripper_position.call_count == 2
    proxy.cancel.assert_called_once_with()


def test_verified_handle_grasp_can_block_full_finger_closure(flow):
    runner, _, _, hold, _, state = flow
    for _ in range(3):
        runner.tick()
    state.side_effect = None
    state.return_value = PlanningGroupState(None, None, 0.35)
    runner.tick()
    assert runner.index == 4
    assert runner.holding
    assert runner.evidence[-1]["object_blocked_closure"]
    hold.assert_called_once_with()


def test_blocked_closure_without_verified_retention_waits(flow):
    runner, proxy, _, hold, _, state = flow
    for _ in range(3):
        runner.tick()
    state.side_effect = None
    state.return_value = PlanningGroupState(None, None, 0.35)
    hold.return_value = False
    runner.tick()
    runner.tick()
    assert runner.index == 3
    assert not runner.holding
    assert proxy.set_gripper_position.call_count == 2


@pytest.mark.parametrize("measured", [None, math.nan, -0.1, 1.0])
def test_unmeasured_or_open_gripper_cannot_admit_lift_even_with_hold_callback(flow, measured):
    runner, _, _, hold, _, state = flow
    for _ in range(3):
        runner.tick()
    state.side_effect = None
    state.return_value = PlanningGroupState(None, None, measured)
    runner.tick()
    assert runner.index == 3
    assert not runner.holding
    hold.assert_not_called()


def test_collision_or_attachment_admission_failure_never_starts_press(flow):
    runner, proxy, move, _, press, _ = flow
    for _ in range(4):
        runner.tick()
    move.side_effect = RuntimeError("held collision representation unavailable")
    with pytest.raises(RuntimeError, match="held collision"):
        runner.tick()
    press.assert_not_called()
    assert runner.holding
    # Preserve the holding gripper on failure, rather than drop the object.
    assert [c.args[0] for c in proxy.set_gripper_position.call_args_list] == [1, 0]


@pytest.mark.parametrize(
    "status", [ExecutionStatus.ABORTED, ExecutionStatus.UNCERTAIN, ExecutionStatus.FAULT]
)
def test_cancel_halts_next_stage_and_records_confirmed_or_uncertain_stop(flow, status):
    runner, proxy, move, _, _, _ = flow
    proxy.cancel.return_value = ExecutionResult(status)
    result = runner.cancel()
    assert result.status is status
    assert runner.tick() == "cancelled"
    assert runner.evidence[-1]["stop_confirmed"] is (status is ExecutionStatus.ABORTED)
    move.assert_not_called()
    proxy.set_gripper_position.assert_not_called()


def test_total_deadline_stops_flow_without_opening_hold(flow, mocker):
    runner, proxy, move, _, _, _ = flow
    mocker.patch(
        "dimos.simulation.behavior.radio_bimanual.time.monotonic", return_value=runner.deadline + 1
    )
    with pytest.raises(TimeoutError, match="deadline"):
        runner.tick()
    move.assert_not_called()
    proxy.set_gripper_position.assert_not_called()
    proxy.cancel.assert_called_once_with()


def test_cancel_during_checked_move_never_dispatches_grasp(flow):
    runner, proxy, move, _, _, _ = flow
    runner.tick()
    proxy.cancel.return_value = ExecutionResult(ExecutionStatus.UNCERTAIN)
    move.side_effect = lambda *args: runner.cancel()
    with pytest.raises(RuntimeError, match="Cancellation requested"):
        runner.tick()
    assert runner.index == 1
    assert runner.tick() == "cancelled"
    assert move.call_count == 1
    assert proxy.set_gripper_position.call_count == 1
    assert runner.evidence[-1]["stop_confirmed"] is False


def test_placement_without_support_keeps_holding_gripper_closed(flow, mocker):
    runner, proxy, _, _, _, _ = flow
    for _ in range(11):
        runner.tick()
    mocker.patch.object(runner, "support_verified", return_value=False)
    with pytest.raises(RuntimeError, match="Placement support"):
        runner.tick()
    assert runner.holding
    assert [c.args[0] for c in proxy.set_gripper_position.call_args_list] == [1, 0, 0]


def test_cancellation_rpc_error_is_unconfirmed_stop(flow):
    runner, proxy, move, _, _, _ = flow
    proxy.cancel.side_effect = RuntimeError("RPC transport lost")
    assert runner.cancel().status is ExecutionStatus.UNCERTAIN
    assert runner.evidence[-1]["stop_confirmed"] is False
    assert runner.tick() == "cancelled"
    move.assert_not_called()


@pytest.mark.parametrize("timeout", [0, 121, math.nan])
def test_invalid_deadline_rejected_before_motion(flow, timeout):
    runner, proxy, move, hold, press, _ = flow
    with pytest.raises(ValueError, match="deadline"):
        BimanualRadioFlow(
            runner.right,
            runner.left,
            runner.intent,
            move,
            hold,
            press,
            lambda: 0,
            runner.support_verified,
            timeout=timeout,
        )
    proxy.set_gripper_position.assert_not_called()


def test_dual_blueprint_has_disjoint_grippers_and_no_base_trajectory():
    bp = radio_blueprint(TaskSelection(activity="turning_on_radio"), bimanual=True)
    atoms = {atom.module: atom for atom in bp.blueprints}
    tasks = atoms[RadioCoordinator].kwargs["tasks"]
    assert [(t.name, t.joint_names) for t in tasks[1:]] == [
        ("r1pro_left_gripper", ["r1pro/left_gripper_finger_joint1"]),
        ("r1pro_right_gripper", ["r1pro/right_gripper_finger_joint1"]),
    ]
    model = atoms[BimanualRadioManipulationModule].kwargs["model"]
    assert {g.name for g in model.planning_groups} == {"left_arm", "right_arm", "torso"}
    assert not any("base_" in n for n in tasks[0].joint_names)
    assert atoms[BehaviorConnection].kwargs["policy_hide_toggle_markers"] is True


def test_dual_blueprint_binds_policy_to_right_arm_with_explicit_torso():
    bp = radio_blueprint(
        TaskSelection(activity="turning_on_radio"),
        arm="right_arm",
        bimanual=True,
        policy_supervisor=True,
        policy_auxiliary_groups=("torso",),
    )
    atoms = {atom.module: atom for atom in bp.blueprints}
    assert atoms[RadioPolicyModule].kwargs == {
        "arm": "right_arm",
        "auxiliary_groups": ("torso",),
        "motion_contract": "checkpoint",
    }
    assert atoms[BehaviorConnection].kwargs["policy_hide_toggle_markers"] is True
    assert {
        g.name for g in atoms[BimanualRadioManipulationModule].kwargs["model"].planning_groups
    } == {
        "left_arm",
        "right_arm",
        "torso",
    }


@pytest.mark.parametrize("groups", [("base",), ("left_arm",), ("torso", "torso")])
def test_policy_auxiliary_selection_rejects_base_other_arm_and_duplicates(groups):
    with pytest.raises(ValueError, match="base and other arm stay fixed"):
        radio_blueprint(TaskSelection(activity="turning_on_radio"), policy_auxiliary_groups=groups)
