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

from dataclasses import replace

import numpy as np
import pytest

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    ManipulationSpec,
    MoveResult,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.porcelain.dimos import Dimos
from dimos.sdk.manipulation import Arm, MotionError


@pytest.fixture
def rpc(mocker):
    proxy = mocker.Mock(spec=ManipulationSpec)
    proxy.list_planning_groups.return_value = (
        PlanningGroupInfo("arm", ("j0", "j1"), "base", "tip", True),
    )
    proxy.get_state.return_value = ManipulationSnapshot(
        timestamp=1.0,
        operation_status=OperationStatus.IDLE,
        error=None,
        has_pending_plan=False,
        execution_status=ExecutionStatus.IDLE,
        groups={
            "arm": PlanningGroupState(
                joints=JointState(name=["j0", "j1"], position=[0.1, 0.2]),
                end_effector_pose=PoseStamped(
                    frame_id="world", position=[0.4, 0.0, 0.3], orientation=[0.0, 1.0, 0.0, 0.0]
                ),
                gripper_position=0.5,
                joint_presets={
                    "home": JointState(name=["j0", "j1"], position=[0.0, 0.5]),
                    "init": JointState(name=["j0", "j1"], position=[0.1, 0.2]),
                },
            ),
        },
    )
    proxy.plan_to_joints.return_value = PlanResult(PlanStatus.SUCCEEDED)
    proxy.plan_to_poses.return_value = PlanResult(PlanStatus.SUCCEEDED)
    proxy.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    proxy.move_linear.return_value = MoveResult(
        PlanResult(PlanStatus.SUCCEEDED),
        ExecutionResult(ExecutionStatus.COMPLETED),
        (0.0, 0.0, 0.01),
        False,
    )
    proxy.set_gripper_position.return_value = CommandResult(CommandStatus.SUCCEEDED)
    return proxy


@pytest.fixture
def app(mocker, rpc):
    app = mocker.Mock(spec=Dimos)
    app.get_module.return_value = rpc
    return app


@pytest.fixture
def arm(app):
    return Arm.from_app(app)


def test_discovery_binds_unique_pose_group_without_requiring_gripper(app, rpc):
    info = replace(rpc.list_planning_groups.return_value[0], has_gripper=False)
    rpc.list_planning_groups.return_value = (
        PlanningGroupInfo("base", ("wheel",), "world", None, False),
        info,
    )

    arm = Arm.from_app(app)

    assert arm.info == info
    assert arm.rpc is rpc
    app.get_module.assert_called_once_with(ManipulationSpec, instance_name=None)
    rpc.execute.assert_not_called()
    app.run.assert_not_called()
    app.stop.assert_not_called()


@pytest.mark.parametrize("group", [None, "missing", "base"])
def test_invalid_or_ambiguous_selection_lists_group_ids(app, rpc, group):
    info = rpc.list_planning_groups.return_value[0]
    rpc.list_planning_groups.return_value = (
        replace(info, id="left"),
        replace(info, id="right"),
        replace(info, id="base", tip_frame=None),
    )

    with pytest.raises(ValueError, match="left.*right.*base"):
        Arm.from_app(app, group=group)


def test_no_pose_groups_raises(app, rpc):
    rpc.list_planning_groups.return_value = ()

    with pytest.raises(ValueError, match="available arms: \\[\\]"):
        Arm.from_app(app)


def test_explicit_group_and_module_selection(app, rpc):
    info = rpc.list_planning_groups.return_value[0]
    rpc.list_planning_groups.return_value = (replace(info, id="left"), replace(info, id="right"))

    arm = Arm.from_app(app, group="right", instance_name="robot/motion")

    assert arm.info.id == "right"
    app.get_module.assert_called_once_with(ManipulationSpec, instance_name="robot/motion")


def test_joints_are_fresh_arrays_in_declared_order(arm, rpc):
    state = rpc.get_state.return_value.groups["arm"]
    state.joints.name[:] = ["j1", "j0"]

    positions = arm.joints()
    np.testing.assert_array_equal(positions, [0.2, 0.1])
    positions[0] = 99.0

    np.testing.assert_array_equal(arm.joints(), [0.2, 0.1])
    assert arm.state() is state
    assert arm.pose() is state.end_effector_pose


@pytest.mark.parametrize("method,field", [("joints", "joints"), ("pose", "end_effector_pose")])
def test_missing_telemetry_raises(arm, rpc, method, field):
    snapshot = rpc.get_state.return_value
    rpc.get_state.return_value = replace(
        snapshot, groups={"arm": replace(snapshot.groups["arm"], **{field: None})}
    )

    with pytest.raises(RuntimeError, match="unavailable"):
        getattr(arm, method)()


def test_disappeared_group_raises(arm, rpc):
    rpc.get_state.return_value = replace(rpc.get_state.return_value, groups={})

    with pytest.raises(RuntimeError, match="State is unavailable.*arm"):
        arm.state()


@pytest.mark.parametrize("convert", [list, tuple, np.array])
def test_joint_inputs_plan_before_blocking_execution(arm, rpc, convert):
    result = arm.move_joints(convert([0.3, 0.4]), speed_scale=0.2, timeout=10.0)

    target = rpc.plan_to_joints.call_args.args[0]["arm"]
    assert target.name == ["j0", "j1"]
    assert target.position == [0.3, 0.4]
    assert rpc.plan_to_joints.call_args.kwargs == {"speed_scale": 0.2}
    assert [call[0] for call in rpc.method_calls][-2:] == ["plan_to_joints", "execute"]
    rpc.execute.assert_called_once_with(blocking=True, timeout=10.0)
    assert result is rpc.execute.return_value


@pytest.mark.parametrize("positions", [[0.0], [[0.0, 0.1]], [0.0, np.nan], [np.inf, 0], [1j, 0]])
def test_invalid_joint_inputs_never_plan_or_execute(arm, rpc, positions):
    with pytest.raises(ValueError):
        arm.move_joints(positions)

    rpc.plan_to_joints.assert_not_called()
    rpc.execute.assert_not_called()


@pytest.mark.parametrize("convert", [list, tuple, np.array])
def test_pose_preserves_current_orientation(arm, rpc, convert):
    result = arm.move_pose(convert([0.5, 0.1, 0.4]), speed_scale=0.3, timeout=20.0)

    target = rpc.plan_to_poses.call_args.args[0]["arm"]
    assert target.frame_id == "world"
    np.testing.assert_array_equal(target.position.to_numpy(), [0.5, 0.1, 0.4])
    assert target.orientation.to_tuple() == (0.0, 1.0, 0.0, 0.0)
    assert rpc.plan_to_poses.call_args.kwargs == {"speed_scale": 0.3}
    rpc.execute.assert_called_once_with(blocking=True, timeout=20.0)
    assert result is rpc.execute.return_value


@pytest.mark.parametrize("convert", [list, tuple, np.array])
def test_explicit_orientation_needs_no_state_read(arm, rpc, convert):
    arm.move_pose([0.4, 0.0, 0.3], orientation=convert([0.0, 0.0, 0.0, 1.0]))

    target = rpc.plan_to_poses.call_args.args[0]["arm"]
    assert target.orientation.to_tuple() == (0.0, 0.0, 0.0, 1.0)
    rpc.get_state.assert_not_called()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"position": [0.1, 0.2]},
        {"position": [0.1, 0.2, np.inf]},
        {"position": [0.1, 0.2, 0.3], "orientation": [0, 0, 1]},
        {"position": [0.1, 0.2, 0.3], "orientation": [[0, 0, 0, 1]]},
        {"position": [0.1, 0.2, 0.3], "orientation": [0, 0, 0, np.nan]},
    ],
)
def test_invalid_pose_inputs_never_plan_or_execute(arm, rpc, kwargs):
    with pytest.raises(ValueError):
        arm.move_pose(**kwargs)

    rpc.plan_to_poses.assert_not_called()
    rpc.execute.assert_not_called()


@pytest.mark.parametrize(
    "method,rpc_method", [("move_joints", "plan_to_joints"), ("move_pose", "plan_to_poses")]
)
def test_plan_failure_stops_sequence_and_retains_result(arm, rpc, method, rpc_method):
    failure = PlanResult(PlanStatus.FAILED, "no solution")
    getattr(rpc, rpc_method).return_value = failure
    target = [0.1, 0.2] if method == "move_joints" else [0.1, 0.2, 0.3]

    with pytest.raises(MotionError, match="no solution") as error:
        getattr(arm, method)(target)

    assert error.value.operation == method
    assert error.value.result is failure
    rpc.execute.assert_not_called()


@pytest.mark.parametrize(
    "status",
    [
        ExecutionStatus.ACCEPTED,
        ExecutionStatus.TIMED_OUT,
        ExecutionStatus.UNCERTAIN,
        ExecutionStatus.FAULT,
    ],
)
def test_noncompleted_execution_raises_without_retry_or_cancel(arm, rpc, status):
    result = ExecutionResult(status)
    rpc.execute.return_value = result

    with pytest.raises(MotionError) as error:
        arm.move_joints([0.2, 0.3])

    assert error.value.result is result
    rpc.execute.assert_called_once_with(blocking=True, timeout=None)
    assert rpc.plan_to_joints.call_count == 1
    rpc.cancel.assert_not_called()


def test_transport_timeout_is_preserved_without_retry(arm, rpc):
    failure = TimeoutError("transport timed out")
    rpc.execute.side_effect = failure

    with pytest.raises(TimeoutError) as error:
        arm.move_joints([0.2, 0.3])

    assert error.value is failure
    assert rpc.execute.call_count == 1
    rpc.cancel.assert_not_called()


@pytest.mark.parametrize("options,checked", [({}, False), ({"check_collision": True}, True)])
def test_linear_delegates_directly_and_preserves_collision_default(arm, rpc, options, checked):
    result = arm.move_linear(dz=0.01, speed_scale=0.2, timeout=10.0, **options)

    rpc.move_linear.assert_called_once_with(
        dx=0.0,
        dy=0.0,
        dz=0.01,
        planning_group="arm",
        check_collision=checked,
        speed_scale=0.2,
        blocking=True,
        timeout=10.0,
    )
    assert result is rpc.move_linear.return_value
    rpc.plan_to_joints.assert_not_called()
    rpc.plan_to_poses.assert_not_called()
    rpc.execute.assert_not_called()


@pytest.mark.parametrize(
    "plan,execution",
    [
        (PlanResult(PlanStatus.FAILED), None),
        (PlanResult(PlanStatus.SUCCEEDED), None),
        (PlanResult(PlanStatus.SUCCEEDED), ExecutionResult(ExecutionStatus.ACCEPTED)),
        (PlanResult(PlanStatus.SUCCEEDED), ExecutionResult(ExecutionStatus.TIMED_OUT)),
    ],
)
def test_linear_failure_retains_whole_result(arm, rpc, plan, execution):
    result = MoveResult(plan, execution, (0.0, 0.0, 0.01), False)
    rpc.move_linear.return_value = result

    with pytest.raises(MotionError) as error:
        arm.move_linear(dz=0.01)

    assert error.value.operation == "move_linear"
    assert error.value.result is result
    rpc.cancel.assert_not_called()


def test_nonfinite_linear_delta_is_rejected_locally(arm, rpc):
    with pytest.raises(ValueError, match="finite"):
        arm.move_linear(dz=np.nan)

    rpc.move_linear.assert_not_called()


@pytest.mark.parametrize("method,position", [("open_gripper", 1.0), ("close_gripper", 0.0)])
def test_gripper_shortcuts_use_normalized_positions(arm, rpc, method, position):
    result = getattr(arm, method)()

    rpc.set_gripper_position.assert_called_once_with(position, planning_group="arm")
    assert result is rpc.set_gripper_position.return_value


@pytest.mark.parametrize("position", [-0.1, 1.1, np.nan, np.inf])
def test_invalid_gripper_position_never_commands(arm, rpc, position):
    with pytest.raises(ValueError, match="between 0 and 1"):
        arm.set_gripper_position(position)

    rpc.set_gripper_position.assert_not_called()


def test_gripperless_arm_rejects_gripper_command(rpc):
    arm = Arm(rpc, replace(rpc.list_planning_groups.return_value[0], has_gripper=False))

    with pytest.raises(ValueError, match="no gripper"):
        arm.open_gripper()

    rpc.set_gripper_position.assert_not_called()


def test_gripper_failure_retains_result(arm, rpc):
    failure = CommandResult(CommandStatus.FAILED, "gripper unavailable")
    rpc.set_gripper_position.return_value = failure

    with pytest.raises(MotionError, match="gripper unavailable") as error:
        arm.set_gripper_position(0.5)

    assert error.value.result is failure


def test_home_uses_server_preset_and_preserves_names(arm, rpc):
    target = rpc.get_state.return_value.groups["arm"].joint_presets["home"]

    result = arm.home(speed_scale=0.2, timeout=10.0)

    rpc.plan_to_joints.assert_called_once_with({"arm": target}, speed_scale=0.2)
    rpc.execute.assert_called_once_with(blocking=True, timeout=10.0)
    assert result is rpc.execute.return_value


def test_init_preset_is_fetched_fresh_each_time(arm, rpc):
    first = rpc.get_state.return_value
    new_target = JointState(name=["j1", "j0"], position=[0.7, 0.8])
    second = replace(
        first, groups={"arm": replace(first.groups["arm"], joint_presets={"init": new_target})}
    )
    rpc.get_state.side_effect = [first, second]

    arm.move_to_preset("init")
    arm.move_to_preset("init")

    assert (
        rpc.plan_to_joints.call_args_list[0].args[0]["arm"]
        is first.groups["arm"].joint_presets["init"]
    )
    assert rpc.plan_to_joints.call_args_list[1].args[0]["arm"] is new_target


def test_missing_preset_lists_available_without_moving(arm, rpc):
    with pytest.raises(ValueError, match="Unknown joint preset 'missing'.*home.*init"):
        arm.move_to_preset("missing")

    rpc.plan_to_joints.assert_not_called()
    rpc.execute.assert_not_called()


def test_failed_preset_plan_never_executes(arm, rpc):
    failure = PlanResult(PlanStatus.FAILED, "preset unreachable")
    rpc.plan_to_joints.return_value = failure

    with pytest.raises(MotionError, match="preset unreachable") as error:
        arm.home()

    assert error.value.operation == "move_to_preset('home')"
    assert error.value.result is failure
    rpc.execute.assert_not_called()
