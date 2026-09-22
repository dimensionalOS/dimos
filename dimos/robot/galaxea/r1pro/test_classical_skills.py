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

"""Classical commands preserve grip force and cancellation across state transitions."""

from concurrent.futures import CancelledError
import json

import numpy as np
import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.galaxea.r1pro.apartment_navigation import CLASSICAL_POSITION_TASK
from dimos.robot.galaxea.r1pro.classical_skills import R1ProClassicalSkills
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS


@pytest.fixture
def skills(mocker):
    module = R1ProClassicalSkills()
    module._sim = mocker.Mock()
    module._control = mocker.Mock()
    module._control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    module._control.task_invoke.return_value = TrajectoryState.COMPLETED
    yield module
    module.stop()


def test_opening_left_preserves_other_grip_and_torso_preload(skills, mocker):
    positions = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.0] * 18 + [0.012, 0.017], strict=True))
    commands = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.01] + [0.0] * 19, strict=True))
    skills._sim.primitive_state.return_value = dict(
        joint_positions=positions, joint_commands=commands
    )
    drive = mocker.patch.object(skills, "_drive")
    mocker.patch.object(skills, "_pause")
    report = {}
    skills._gripper("left", 0.05, report)
    drive.assert_called_once_with([[0.01] + [0.0] * 19, [0.01] + [0.0] * 17 + [0.05, 0.0]], report)


def test_cancelled_motion_never_reaches_the_coordinator(skills):
    skills._cancel.set()
    with pytest.raises(CancelledError):
        skills._drive([[0.0] * 20, [0.01] * 18 + [0.0, 0.0]], {})
    skills._control.execute_trajectory.assert_not_called()


def test_empty_plan_is_rejected_before_execution(skills):
    with pytest.raises(RuntimeError, match="no executable waypoints"):
        skills._drive([], {})
    skills._control.execute_trajectory.assert_not_called()


def test_scene_fault_prevents_motion(skills):
    skills._sim.primitive_state.return_value = dict(error="lost cargo")
    with pytest.raises(RuntimeError, match="lost cargo"):
        skills._drive([[0.0] * 20, [0.01] * 18 + [0.0, 0.0]], {})
    skills._control.execute_trajectory.assert_not_called()


def test_small_joint_error_does_not_skip_a_needed_tcp_correction(skills, mocker):
    joints = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.0] * 20, strict=True))
    target = np.eye(4)
    displaced = target.copy()
    displaced[0, 3] = 0.01
    skills._sim.primitive_state.side_effect = [
        dict(joint_positions=joints),
        dict(tcp_poses={"left": displaced.tolist()}),
        dict(tcp_poses={"left": target.tolist()}),
        dict(joint_positions=joints),
    ]
    mocker.patch.object(skills, "_drive")
    skills._prepare_posture(
        dict(
            object="object_1",
            reachability=dict(arm="left", ready_joints=[0.0] * 20, pregrasp=target.tolist()),
        ),
        {},
    )
    skills._sim.classical_align.assert_called_once_with(0, "left", target.tolist())


def test_contact_loss_during_transfer_stops_the_segment(skills, mocker):
    positions = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.0] * 20, strict=True))
    before = dict(
        active=("place", "right"), objects=[dict(index=0, grasped=True, contacting_arms=["right"])]
    )
    during = dict(
        error=None,
        sim_time=2.0,
        joint_positions=positions,
        objects=[dict(index=0, grasped=False, contacting_arms=[])],
    )
    skills._sim.primitive_state.side_effect = [before, during]
    mocker.patch.object(skills, "_pause")
    with pytest.raises(RuntimeError, match="lost two-finger contact"):
        skills._drive([[0.0] * 20, [0.001] * 18 + [0.0, 0.0]], dict(phase="preplace"))


def test_aborted_trajectory_is_not_success_even_at_target(skills, mocker):
    positions = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.0] * 20, strict=True))
    state = dict(active=None, error=None, sim_time=2.0, joint_positions=positions, objects=[])
    skills._sim.primitive_state.return_value = state
    skills._control.task_invoke.return_value = TrajectoryState.ABORTED
    mocker.patch.object(skills, "_pause")
    with pytest.raises(RuntimeError, match="stopped before completing"):
        skills._drive([[0.0] * 20, [0.001] * 18 + [0.0, 0.0]], {})


@pytest.mark.parametrize("unsettled", ["velocity", "command"])
@pytest.mark.parametrize("point_count", [1, 2])
def test_completed_task_waits_for_delivered_and_settled_endpoint(
    skills, mocker, unsettled, point_count
):
    positions = dict.fromkeys(R1PRO_PICK_PLACE_JOINTS, 0.0)
    states = [dict(active=None, objects=[])]
    for tick in range(1, 7):
        states.append(
            dict(
                error=None,
                sim_time=float(tick),
                joint_positions=positions,
                joint_commands=dict.fromkeys(
                    positions, 0.005 if tick <= 3 and unsettled == "command" else 0.0
                ),
                joint_velocities=dict.fromkeys(
                    positions, 0.01 if tick <= 3 and unsettled == "velocity" else 0.0
                ),
            )
        )
    skills._sim.primitive_state.side_effect = states
    mocker.patch.object(skills, "_pause")

    report = dict(phase="preplace")
    skills._drive([list(np.zeros(20))] * point_count, report)

    assert skills._sim.primitive_state.call_count == 7
    assert json.loads(json.dumps(report))["last_trajectory_tracking"]["commands_delivered"] is True


@pytest.mark.parametrize("supports", [[], ["wrong_table"]])
def test_missing_intended_support_never_opens_hand(skills, mocker, supports):
    skills._sim.primitive_state.return_value = {
        "objects": [{"support_geoms": supports, "upright": True}],
        "tcp_poses": {"left": np.eye(4).tolist()},
    }
    mocker.patch.object(skills, "_pause")
    line = mocker.patch.object(skills, "_line")
    chosen = dict(index=0, arm="left", tcp=np.eye(4).tolist(), region={"support_geoms": ["table"]})
    with pytest.raises(RuntimeError, match="surface|support contact"):
        skills._seek_support(chosen, {})
    skills._control.execute_trajectory.assert_not_called()
    if not supports:
        # An under-tracking arm must not accumulate unexecuted descents into
        # a target that penetrates the support in the geometric planner.
        assert len(line.call_args_list) == 20
        assert all(call.args[2][2][3] == pytest.approx(-0.0005) for call in line.call_args_list)


def test_confirmed_support_finishes_without_further_descent(skills, mocker):
    skills._sim.primitive_state.return_value = {
        "objects": [{"support_geoms": ["table"], "upright": True}],
        "tcp_poses": {"right": np.eye(4).tolist()},
    }
    mocker.patch.object(skills, "_pause")
    line = mocker.patch.object(skills, "_line")
    report = {}
    skills._seek_support(
        dict(index=0, arm="right", tcp=np.eye(4).tolist(), region={"support_geoms": ["table"]}),
        report,
    )
    assert report["support_before_release"] == ["table"]
    line.assert_not_called()


def test_departing_checked_navigation_corridor_fails_before_route_continues(skills, mocker):
    path = [[0, 0, 0], [1, 0, 0]]
    skills._sim.validate_object_navigation.return_value = path
    skills._sim.primitive_state.side_effect = [
        dict(base_pose=[0, 0, 0], sim_time=1.0),
        dict(base_pose=[0.5, 0.05, 0], sim_time=2.0, error=None),
    ]
    mocker.patch.object(skills, "_pause")
    report = {}

    with pytest.raises(RuntimeError, match="checked tracking allowance"):
        skills._follow(path, report)

    assert report["commanded_paths"] == [path]
    assert report["max_navigation_tracking_error_m"] == pytest.approx(0.05)


def test_local_positioning_checks_sdk_plan_then_follows_measured_progress(skills, mocker):
    names = list(R1PRO_PLANAR_BASE.joint_names)
    trajectory = JointTrajectory(
        joint_names=[names[2], names[0], names[1]],
        points=[
            TrajectoryPoint(positions=[0, 0, 0], time_from_start=0),
            TrajectoryPoint(positions=[0.4, 0.1, 0.2], time_from_start=1),
        ],
    )
    skills._manipulation = mocker.Mock()
    skills._manipulation.plan_to_joints.return_value = mocker.Mock(
        succeeded=True, plan=mocker.Mock(trajectory=trajectory, plan_id="checked-local")
    )
    skills._sim.primitive_state.side_effect = [
        {"base_pose": [0, 0, 0]},
        {"base_pose": [0.1, 0.2, 0.4]},
    ]
    follow = mocker.patch.object(skills, "_execute_base_path")
    mocker.patch.object(skills, "_pause")
    report = {}

    skills._position_base({"base_waypoints": [[0, 0, 0], [0.1, 0.2, 0.4]]}, report)

    skills._sim.validate_primitive_base_plan.assert_called_once_with(trajectory)
    follow.assert_called_once_with(
        [[0, 0, 0], [0.1, 0.2, 0.4]],
        report,
        tracking_limit=0.015,
        task_name=CLASSICAL_POSITION_TASK,
        arrival_tolerance=0.03,
    )
    skills._manipulation.execute.assert_not_called()


def test_local_arrival_accepts_eighteen_mm_and_checks_the_measured_footprint(skills, mocker):
    skills._sim.primitive_state.side_effect = [
        dict(base_pose=[0, 0, 0], sim_time=1.0),
        dict(base_pose=[1.018, 0, 0], sim_time=2.0, error=None),
        dict(base_pose=[1.018, 0, 0], sim_time=2.5, error=None),
    ]
    mocker.patch.object(skills, "_pause")
    report = {}

    skills._execute_base_path(
        [[0, 0, 0], [1, 0, 0]],
        report,
        tracking_limit=0.015,
        task_name=CLASSICAL_POSITION_TASK,
        arrival_tolerance=0.03,
    )

    checked = skills._sim.validate_primitive_base_plan.call_args.args[0]
    assert checked.points[0].positions == [1.018, 0, 0]
    assert report["base_arrivals"][0]["position_error_m"] == pytest.approx(0.018)
    skills._control.task_invoke.assert_any_call(CLASSICAL_POSITION_TASK, "cancel", {})
    skills._sim.stop_primitive_base.assert_called_once()


def test_local_arrival_rejects_excess_drift_after_stopping(skills, mocker):
    skills._sim.primitive_state.side_effect = [
        dict(base_pose=[0, 0, 0], sim_time=1.0),
        dict(base_pose=[1.018, 0, 0], sim_time=2.0, error=None),
        dict(base_pose=[1.045, 0, 0], sim_time=2.5, error=None),
    ]
    mocker.patch.object(skills, "_pause")

    with pytest.raises(RuntimeError, match="outside the docking tolerance"):
        skills._execute_base_path(
            [[0, 0, 0], [1, 0, 0]],
            {},
            tracking_limit=0.015,
            task_name=CLASSICAL_POSITION_TASK,
            arrival_tolerance=0.03,
        )

    skills._sim.validate_primitive_base_plan.assert_not_called()
    skills._sim.stop_primitive_base.assert_called_once()


def test_local_arrival_tolerance_does_not_accept_an_obstructed_footprint(skills, mocker):
    skills._sim.primitive_state.return_value = dict(
        base_pose=[1.018, 0, 0], sim_time=1.0, error=None
    )
    skills._sim.validate_primitive_base_plan.side_effect = RuntimeError("blocked footprint")
    mocker.patch.object(skills, "_pause")
    report = {}

    with pytest.raises(RuntimeError, match="blocked footprint"):
        skills._execute_base_path(
            [[0, 0, 0], [1, 0, 0]],
            report,
            tracking_limit=0.015,
            task_name=CLASSICAL_POSITION_TASK,
            arrival_tolerance=0.03,
        )

    assert "base_arrivals" not in report
    skills._sim.stop_primitive_base.assert_called_once()


def test_transit_stops_at_accepted_endpoint_before_follower_overshoot(skills, mocker):
    path = [[0, 0, 0], [1, 0, 0]]
    skills._sim.validate_object_navigation.return_value = path
    skills._sim.primitive_state.side_effect = [
        dict(base_pose=[0, 0, 0], sim_time=1.0),
        dict(base_pose=[0.98, 0, 0], sim_time=2.0, error=None),
        dict(base_pose=[0.981, 0, 0], sim_time=2.5, error=None),
    ]
    mocker.patch.object(skills, "_pause")
    report = {}

    skills._follow(path, report)

    skills._sim.stop_primitive_base.assert_called_once()
    assert report["base_arrivals"][0]["position_error_m"] == pytest.approx(0.019)
    checked = skills._sim.validate_primitive_base_plan.call_args.args[0]
    assert checked.points[0].positions == [0.981, 0, 0]


@pytest.fixture
def pick_operation(skills, mocker):
    row = dict(
        id="object_1",
        object="task_object_1",
        index=0,
        released=True,
        settled=True,
        upright=True,
        support_geoms=["table"],
    )
    initial = dict(objects=[row], held_objects={"right": None, "left": "object_2"})
    held = dict(error=None, complete=True, held_objects={"right": "object_1", "left": "object_2"})
    skills._sim.primitive_state.side_effect = [
        initial,
        dict(base_pose=[0, 0, 0]),
        dict(objects=[dict(grasping_arms=["right"])]),
        held,
    ]
    skills._sim.tray_state.return_value = dict(held=False)
    skills._sim.classical_carry_posture.return_value = [[0.0] * 18 + [0.0, 0.0]] * 2
    skills._grasp_generator = mocker.Mock()
    skills._grasp_generator.propose_grasps.return_value = []
    chosen = dict(
        arm="right", base_pose=[0, 0, 0], source_position=[0.3, -0.3, 0.8], tcp=np.eye(4).tolist()
    )
    mocker.patch.object(skills, "_assess_pick", return_value=[chosen])
    for method in ("_stop_control", "_pause", "_position_base", "_prepare_posture", "_gripper"):
        mocker.patch.object(skills, method)

    def line(index, side, target, report):
        report["motion_started"] = True

    mocker.patch.object(skills, "_line", side_effect=line)
    start = mocker.patch.object(skills, "_start", return_value="accepted")
    assert skills.pick_object("object_1", "right") == "accepted"
    return start.call_args.args[1]


def test_pick_finishes_at_verified_lift_without_implicit_posture_motion(
    skills, pick_operation, mocker
):
    drive = mocker.patch.object(skills, "_drive")
    report = {}
    pick_operation(report)

    assert report["lift_verified"] is True
    assert report["phase"] == "holding"
    drive.assert_not_called()
    skills._sim.classical_carry_posture.assert_not_called()
    skills._sim.classical_init_posture.assert_not_called()
    skills._sim.finish_object_navigation.assert_not_called()
    skills._gripper.assert_called_once_with("right", 0.0, report)
    skills._sim.prepare_object_navigation.assert_not_called()


@pytest.mark.parametrize("error", [RuntimeError("lift blocked"), CancelledError("cancelled")])
def test_failed_pick_lift_is_not_success_and_retains_recovery_guard(
    skills, pick_operation, mocker, tmp_path, error
):
    skills._sim.prepare_primitive_session.return_value = dict(output=str(tmp_path))
    skills._sim.save_classical_state.return_value = "saved.npz"

    def interrupted_lift(index, side, target, report):
        report["motion_started"] = True
        if report["phase"] == "lift":
            raise error

    mocker.patch.object(skills, "_line", side_effect=interrupted_lift)

    skills._run(pick_operation)

    status = skills._status()
    assert status["success"] is False
    assert status["recovery_required"] is True
    assert status["phase"] == "lift"
    assert status["error"] == str(error)
    skills._sim.finish_object_navigation.assert_not_called()
    skills._sim.classical_carry_posture.assert_not_called()


@pytest.mark.parametrize("lost_side", ["left", "right"])
def test_prepare_carry_verifies_both_hands_before_success(skills, mocker, lost_side):
    held = {"right": "object_1", "left": "object_2"}
    changed = dict(held, **{lost_side: None})
    skills._sim.primitive_state.side_effect = [
        dict(error=None, held_objects=held),
        dict(error=None, held_objects=changed),
    ]
    skills._sim.classical_carry_posture.return_value = [[0.0] * 20] * 2
    mocker.patch.object(skills, "_drive")
    mocker.patch.object(skills, "_pause")
    report = {}

    with pytest.raises(RuntimeError, match="Cargo ownership changed"):
        skills._prepare_carry(report, phase="retract_to_carry")

    assert "carry_posture" not in report
    skills._sim.finish_object_navigation.assert_not_called()


def test_cargo_lost_between_lift_verification_and_retraction_cannot_become_success(skills, mocker):
    skills._sim.primitive_state.return_value = dict(
        error=None, held_objects={"right": None, "left": "object_2"}
    )
    mocker.patch.object(skills, "_pause")
    report = {}

    with pytest.raises(RuntimeError, match="Cargo ownership changed before retracting"):
        skills._prepare_carry(
            report,
            phase="retract_to_carry",
            expected_held={"right": "object_1", "left": "object_2"},
        )

    skills._sim.classical_carry_posture.assert_not_called()
    skills._sim.finish_object_navigation.assert_not_called()
    assert "carry_posture" not in report


def test_carry_contact_loss_is_detected_even_after_active_primitive_is_cleared(skills, mocker):
    joints = dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.0] * 20, strict=True))
    skills._sim.primitive_state.side_effect = [
        dict(error=None, active=None, objects=[dict(index=0, held_by="right")]),
        dict(
            error=None, sim_time=2.0, joint_positions=joints, objects=[dict(index=0, held_by=None)]
        ),
    ]
    mocker.patch.object(skills, "_pause")

    with pytest.raises(RuntimeError, match="Held object lost contact"):
        skills._drive([[0.0] * 20, [0.001] * 18 + [0, 0]], dict(phase="retract_to_carry"))


@pytest.fixture
def init_operation(skills, mocker):
    target = np.linspace(-0.4, 0.4, 18).tolist()
    grips = [0.0, 0.018]
    held = {"right": "object_1", "left": "object_2"}
    initial = dict(
        error=None,
        held_objects=held,
        base_pose=[1.2, -0.3, 0.7],
        joint_positions=dict(zip(R1PRO_PICK_PLACE_JOINTS, [0.5] * 18 + grips, strict=True)),
    )
    final = dict(
        initial,
        joint_positions=dict(zip(R1PRO_PICK_PLACE_JOINTS, target + grips, strict=True)),
    )
    # Measured startup joints, not the preload-biased actuator endpoint, define init.
    endpoint = (np.asarray(target) + 0.004).tolist() + grips
    points = [[0.5] * 18 + grips, endpoint]
    skills._sim.tray_state.return_value = dict(held=False, finger_contacts=[])
    skills._sim.primitive_state.side_effect = [initial, final]
    skills._sim.classical_init_posture.return_value = dict(waypoints=points, target_joints=target)
    for method in ("_stop_control", "_pause", "_position_base", "_navigate", "_gripper", "_drive"):
        mocker.patch.object(skills, method)
    start = mocker.patch.object(skills, "_start", return_value="accepted")
    assert skills.return_to_init() == "accepted"
    assert start.call_args.args[0] == "return_to_init"
    return start.call_args.args[1], initial, final


def test_return_to_init_verifies_recorded_startup_without_resetting_or_releasing(
    skills, init_operation
):
    operation, initial, _ = init_operation
    report = {}

    operation(report)

    expected = skills._sim.classical_init_posture.return_value
    skills._drive.assert_called_once_with(expected["waypoints"], report)
    assert report["phase"] == "at_init"
    assert report["init_posture"] == dict(
        target_joints=expected["target_joints"],
        measured_joints=expected["target_joints"],
        verified=True,
        held_objects=initial["held_objects"],
    )
    skills._stop_control.assert_called_once()
    skills._sim.classical_init_posture.assert_called_once()
    skills._sim.finish_object_navigation.assert_called_once()
    skills._sim.reset.assert_not_called()
    skills._sim.classical_carry_posture.assert_not_called()
    skills._gripper.assert_not_called()
    skills._navigate.assert_not_called()
    skills._position_base.assert_not_called()


@pytest.mark.parametrize("side", ["left", "right"])
def test_return_to_init_requires_both_holds_to_survive(skills, init_operation, side):
    operation, _, final = init_operation
    final["held_objects"] = dict(final["held_objects"], **{side: None})
    report = {}

    with pytest.raises(RuntimeError, match="Cargo ownership changed"):
        operation(report)

    assert report["init_posture"]["verified"] is False
    skills._sim.finish_object_navigation.assert_not_called()


@pytest.mark.parametrize("index", [0, 4, 11])
@pytest.mark.parametrize("offset", [0.021, float("nan")])
def test_return_to_init_rejects_unreached_torso_or_arm_startup_joint(
    skills, init_operation, index, offset
):
    operation, _, final = init_operation
    final["joint_positions"][R1PRO_PICK_PLACE_JOINTS[index]] += offset
    report = {}

    with pytest.raises(RuntimeError, match="did not reach the fixed init posture"):
        operation(report)

    assert report["init_posture"]["verified"] is False
    skills._sim.finish_object_navigation.assert_not_called()


@pytest.mark.parametrize("offset", [[0.006, 0, 0], [0.004, 0.004, 0], [0, 0, 0.006]])
def test_return_to_init_fails_if_base_moves(skills, init_operation, offset):
    operation, _, final = init_operation
    final["base_pose"] = (np.asarray(final["base_pose"]) + offset).tolist()

    with pytest.raises(RuntimeError, match="Base moved"):
        operation({})

    skills._sim.finish_object_navigation.assert_not_called()


def test_return_to_init_accepts_same_base_heading_across_angle_wrap(skills, init_operation):
    operation, initial, final = init_operation
    initial["base_pose"] = [1.2, -0.3, np.pi]
    final["base_pose"] = [1.2, -0.3, -np.pi]
    report = {}

    operation(report)

    assert report["init_posture"]["verified"] is True
    skills._sim.finish_object_navigation.assert_called_once()


def test_return_to_init_is_not_verified_if_final_inventory_guard_fails(skills, init_operation):
    operation, _, _ = init_operation
    skills._sim.finish_object_navigation.side_effect = RuntimeError("inventory changed")
    report = {}

    with pytest.raises(RuntimeError, match="inventory changed"):
        operation(report)

    assert report["init_posture"]["verified"] is False
    assert report["phase"] == "return_to_init"


@pytest.mark.parametrize("when", ["before", "after"])
def test_return_to_init_preserves_scene_fault_and_does_not_clear_guard(
    skills, init_operation, when
):
    operation, initial, final = init_operation
    state = initial if when == "before" else final
    state["error"] = "lost cargo"

    with pytest.raises(RuntimeError, match="lost cargo"):
        operation({})

    assert skills._drive.call_count == int(when == "after")
    skills._sim.finish_object_navigation.assert_not_called()


@pytest.mark.parametrize(
    "error", [RuntimeError("init would tip the cup"), CancelledError("cancelled")]
)
def test_return_to_init_planning_failure_does_not_start_motion_or_reset(
    skills, init_operation, tmp_path, error
):
    operation, _, _ = init_operation
    skills._sim.classical_init_posture.side_effect = error
    skills._sim.prepare_primitive_session.return_value = dict(output=str(tmp_path))
    skills._sim.save_classical_state.return_value = "saved.npz"

    skills._run(operation)

    status = skills._status()
    assert status["success"] is False
    assert status["recovery_required"] is False
    assert status["error"] == str(error)
    skills._drive.assert_not_called()
    skills._sim.finish_object_navigation.assert_not_called()
    skills._sim.reset.assert_not_called()


@pytest.mark.parametrize("error", [RuntimeError("tracking fault"), CancelledError("cancelled")])
def test_return_to_init_interrupted_after_motion_keeps_recovery_required(
    skills, init_operation, tmp_path, mocker, error
):
    operation, _, _ = init_operation
    skills._sim.prepare_primitive_session.return_value = dict(output=str(tmp_path))
    skills._sim.save_classical_state.return_value = "saved.npz"

    def interrupted_drive(points, report):
        report["motion_started"] = True
        raise error

    mocker.patch.object(skills, "_drive", side_effect=interrupted_drive)

    skills._run(operation)

    status = skills._status()
    assert status["success"] is False
    assert status["recovery_required"] is True
    assert status["error"] == str(error)
    skills._sim.finish_object_navigation.assert_not_called()


@pytest.mark.parametrize(
    "tray", [dict(held=True, finger_contacts=[]), dict(held=False, finger_contacts=["left"])]
)
def test_return_to_init_rejects_held_or_contacted_tray(skills, mocker, tray):
    skills._sim.tray_state.return_value = tray
    start = mocker.patch.object(skills, "_start")

    result = json.loads(skills.return_to_init())

    assert result == dict(accepted=False, reason="Put down the tray before returning to init")
    start.assert_not_called()
    skills._sim.classical_init_posture.assert_not_called()


@pytest.mark.parametrize(
    ("state", "reason"),
    [
        ("busy", "busy_or_stopping"),
        ("closing", "busy_or_stopping"),
        ("recovery", "recovery_required"),
    ],
)
def test_return_to_init_cannot_bypass_action_admission(skills, state, reason):
    skills._sim.tray_state.return_value = dict(held=False, finger_contacts=[])
    skills._action["recovery_required"] = state == "recovery"
    skills._closing = state == "closing"
    if state == "busy":
        skills._done.clear()

    result = json.loads(skills.return_to_init())

    assert result["accepted"] is False
    assert result["reason"] == reason
    skills._sim.classical_init_posture.assert_not_called()
    skills._control.execute_trajectory.assert_not_called()


@pytest.mark.parametrize("phase", ["lower_to_support", "release", "retreat"])
@pytest.mark.parametrize("lose_other_hand", [False, True])
def test_placement_can_release_selected_cargo_but_keeps_other_hand_guarded(
    skills, mocker, phase, lose_other_hand
):
    joints = dict.fromkeys(R1PRO_PICK_PLACE_JOINTS, 0.0)
    skills._sim.primitive_state.side_effect = [
        dict(
            active=("place", "right"),
            objects=[
                dict(index=0, held_by="right", grasped=True, contacting_arms=["right"]),
                dict(index=1, held_by="left", grasped=True, contacting_arms=["left"]),
            ],
        ),
        *[
            dict(
                error=None,
                sim_time=float(tick),
                joint_positions=joints,
                objects=[
                    dict(
                        index=0,
                        held_by=None,
                        grasped=phase == "lower_to_support",
                        support_geoms=["target_surface"],
                    ),
                    dict(index=1, held_by=None if lose_other_hand else "left"),
                ],
            )
            for tick in range(1, 4)
        ],
    ]
    mocker.patch.object(skills, "_pause")
    report = dict(phase=phase)
    if lose_other_hand:
        with pytest.raises(RuntimeError, match="Held object lost contact"):
            skills._drive([[0.0] * 20], report)
    else:
        skills._drive([[0.0] * 20], report)
        assert len(report["trajectory_checks"]) == 1
