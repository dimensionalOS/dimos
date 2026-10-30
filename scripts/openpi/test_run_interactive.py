"""Tests for the OpenPI console's checked exit reset."""

from types import SimpleNamespace
from unittest.mock import call

import pytest

from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS
from scripts.openpi import run_interactive


def test_exit_reset_releases_hold_for_trajectory_and_reengages_it(mocker):
    app = mocker.MagicMock()
    mocker.patch.object(run_interactive.Dimos, "connect", return_value=app)
    app.OpenPIClient.is_paused.return_value = True
    app.ControlCoordinator.get_joint_positions.return_value = dict.fromkeys(
        DUAL_OPENYAM_ARM_JOINTS, 0.0
    )
    app.ControlCoordinator.execute_trajectory.return_value.status.name = "ACCEPTED"
    app.ControlCoordinator.task_invoke.side_effect = [
        SimpleNamespace(on=True, route="manual"),
        SimpleNamespace(on=False),
        True,
        True,
        [1.0],
        [1.0],
        SimpleNamespace(state=run_interactive.TrajectoryState.COMPLETED),
        SimpleNamespace(on=True),
    ]

    run_interactive.reset_arms_to_zero(resume_policy=False)

    app.ControlCoordinator.task_invoke.assert_has_calls(
        [
            call("operator_hold", "get_status"),
            call("operator_hold", "acknowledge"),
            call("left_gripper", "set_normalized", {"values": [1.0]}),
            call("right_gripper", "set_normalized", {"values": [1.0]}),
            call("left_gripper", "get_normalized"),
            call("right_gripper", "get_normalized"),
            call("joint_trajectory", "get_status", {"t_now": None}),
            call(
                "operator_hold",
                "request",
                {"route": "manual", "reason": "OpenPI console closed at zero pose"},
            ),
        ]
    )
    app.OpenPIClient.resume.assert_not_called()
    app.stop.assert_called_once_with()


def test_exit_reset_failure_requests_hold(mocker):
    app = mocker.MagicMock()
    mocker.patch.object(run_interactive.Dimos, "connect", return_value=app)
    app.OpenPIClient.is_paused.return_value = True
    app.ControlCoordinator.get_joint_positions.return_value = dict.fromkeys(
        DUAL_OPENYAM_ARM_JOINTS, 0.0
    )
    app.ControlCoordinator.execute_trajectory.return_value.status.name = "REJECTED"
    app.ControlCoordinator.execute_trajectory.return_value.message = "unsafe"
    app.ControlCoordinator.task_invoke.side_effect = [
        SimpleNamespace(on=True, route="manual"),
        SimpleNamespace(on=False),
        True,
        True,
        [1.0],
        [1.0],
        SimpleNamespace(on=True),
    ]

    with pytest.raises(RuntimeError, match="Zero trajectory rejected"):
        run_interactive.reset_arms_to_zero(resume_policy=False)

    app.ControlCoordinator.task_invoke.assert_any_call(
        "operator_hold", "request", {"route": "manual", "reason": "Reset failed"}
    )
    app.OpenPIClient.resume.assert_not_called()
    app.stop.assert_called_once_with()


def test_exit_reset_gripper_rejection_holds_pose_without_moving_arms(mocker):
    app = mocker.MagicMock()
    mocker.patch.object(run_interactive.Dimos, "connect", return_value=app)
    app.OpenPIClient.is_paused.return_value = True
    app.ControlCoordinator.get_joint_positions.return_value = dict.fromkeys(
        DUAL_OPENYAM_ARM_JOINTS, 0.0
    )
    app.ControlCoordinator.task_invoke.side_effect = [
        SimpleNamespace(on=True, route="manual"),
        SimpleNamespace(on=False),
        False,
        SimpleNamespace(on=True),
    ]

    with pytest.raises(RuntimeError, match="Could not command left gripper open"):
        run_interactive.reset_arms_to_zero(resume_policy=False)

    app.ControlCoordinator.execute_trajectory.assert_not_called()
    app.ControlCoordinator.task_invoke.assert_any_call(
        "operator_hold", "request", {"route": "manual", "reason": "Reset failed"}
    )
    app.stop.assert_called_once_with()


def test_startup_reset_opens_grippers_before_resuming_policy(mocker):
    app = mocker.MagicMock()
    mocker.patch.object(run_interactive.Dimos, "connect", return_value=app)
    app.OpenPIClient.is_paused.return_value = True
    app.ControlCoordinator.get_joint_positions.return_value = dict.fromkeys(
        DUAL_OPENYAM_ARM_JOINTS, 0.0
    )
    app.ControlCoordinator.execute_trajectory.return_value.status.name = "ACCEPTED"
    app.ControlCoordinator.task_invoke.side_effect = [
        True,
        True,
        [1.0],
        [1.0],
        SimpleNamespace(state=run_interactive.TrajectoryState.COMPLETED),
    ]

    run_interactive.reset_arms_to_zero(resume_policy=True)

    app.ControlCoordinator.task_invoke.assert_any_call(
        "left_gripper", "set_normalized", {"values": [1.0]}
    )
    app.ControlCoordinator.task_invoke.assert_any_call(
        "right_gripper", "set_normalized", {"values": [1.0]}
    )
    app.OpenPIClient.resume.assert_called_once_with()
    app.stop.assert_called_once_with()


def test_quit_pauses_policy_before_reset(mocker):
    pause = mocker.patch.object(run_interactive, "control", return_value=True)
    reset = mocker.patch.object(run_interactive, "reset_arms_to_zero")

    assert run_interactive.pause_and_reset()

    pause.assert_called_once_with("pause")
    reset.assert_called_once_with(resume_policy=False)
