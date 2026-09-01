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
from unittest.mock import Mock

from typer.testing import CliRunner

from dimos.cli import hardware_cli
from dimos.cli.hardware import g1 as g1_cli
from dimos.control.tasks.g1_sonic_wbc_task.sonic_diagnostics import (
    SonicDiagnosticCheck,
    SonicDiagnosticReport,
)
from dimos.robot.unitree.g1.manip_config import G1_READY_JOINTS, G1_READY_SPEED_SCALE

runner = CliRunner()


class _Client:
    def __init__(self, coordinator: Mock, manipulation: Mock | None = None) -> None:
        self.coordinator = coordinator
        self.manipulation = manipulation
        self.stopped = False

    def get_module(self, name: str) -> Mock:
        if name == "ControlCoordinator":
            return self.coordinator
        if name == "G1Manipulation" and self.manipulation is not None:
            return self.manipulation
        raise KeyError(name)

    def stop(self) -> None:
        self.stopped = True


def _state(*, armed: bool, dry_run: bool, arming: bool = False) -> dict[str, object]:
    return {
        "active": armed,
        "armed": armed,
        "arming": arming,
        "arm_pending": False,
        "dry_run": dry_run,
        "arming_duration": 10.0,
    }


def _coordinator(task_name: str = "groot_wbc") -> Mock:
    coordinator = Mock()
    coordinator.list_tasks.return_value = [task_name, "joint_trajectory"]
    coordinator.describe_task.side_effect = lambda name: {
        "task": name,
        "commands": (
            {
                "arm": {},
                "disarm": {},
                "set_dry_run": {},
                "state_snapshot": {},
            }
            if name == task_name
            else {"get_status": {}}
        ),
    }
    return coordinator


def test_hardware_namespace_exposes_g1_operator_commands() -> None:
    result = runner.invoke(hardware_cli.app, ["g1", "--help"])

    assert result.exit_code == 0, result.output
    for command in (
        "status",
        "arm",
        "enable",
        "activate",
        "ready",
        "disable",
        "sonic-doctor",
    ):
        assert command in result.output


def test_sonic_doctor_reports_all_checks_without_connecting_to_robot(mocker) -> None:
    doctor = mocker.patch.object(
        g1_cli,
        "_run_sonic_doctor",
        return_value=SonicDiagnosticReport(
            (
                SonicDiagnosticCheck("CUDA execution provider", True, "CUDA, CPU"),
                SonicDiagnosticCheck("planner latency", True, "p95=40.00 ms"),
            )
        ),
    )
    connect = mocker.patch.object(g1_cli.Dimos, "connect")

    result = runner.invoke(g1_cli.app, ["sonic-doctor"])

    assert result.exit_code == 0, result.output
    assert "PASS  CUDA execution provider: CUDA, CPU" in result.output
    assert "PASS  planner latency: p95=40.00 ms" in result.output
    assert "proceed to the MuJoCo soak test" in result.output
    doctor.assert_called_once_with()
    connect.assert_not_called()


def test_sonic_doctor_fails_closed_before_real_robot_control(mocker) -> None:
    mocker.patch.object(
        g1_cli,
        "_run_sonic_doctor",
        return_value=SonicDiagnosticReport(
            (
                SonicDiagnosticCheck("ONNX Runtime", True, "1.18.1"),
                SonicDiagnosticCheck("planner latency", False, "p95=180.00 ms"),
            )
        ),
    )

    result = runner.invoke(g1_cli.app, ["sonic-doctor"])

    assert result.exit_code == 1
    assert "FAIL  planner latency: p95=180.00 ms" in result.output
    assert "do not enable real-robot control" in result.output


def test_status_rejects_coordinator_without_required_rpcs(mocker) -> None:
    coordinator = Mock(spec=["task_invoke"])
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["status"])

    assert result.exit_code == 1
    assert "required G1 coordinator RPCs" in result.output
    assert client.stopped


def test_status_discovers_sonic_lifecycle_task(mocker) -> None:
    coordinator = _coordinator("sonic_teleop")
    coordinator.task_invoke.side_effect = [
        {
            **_state(armed=True, dry_run=False),
            "control_state": "control",
            "reference_source": "planner",
            "stream_active": True,
            "stream_backlog_frames": 3,
            "policy_timing": {
                "start_interval_ms": {
                    "samples": 250,
                    "mean": 25.0,
                    "p99": 30.0,
                }
            },
            "webxr_teleop": {
                "mode": "pose_transition",
                "sonic_pipeline": "sonic-v1.1",
                "pose_window_frames": 10,
                "buffered_frames": 7,
                "stream_ready": False,
                "pose_transition_progress": 0.4,
                "pose_transition_seconds": 0.5,
                "last_transition_reason": "operator_pose_toggle",
            },
        },
        {"state": "idle"},
    ]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["status"])

    assert result.exit_code == 0, result.output
    assert "controller:  sonic_teleop" in result.output
    assert "control:     control" in result.output
    assert "stream_lag:  3 frames (60 ms)" in result.output
    assert "policy_rate: 40.0 Hz (mean 25.00 ms, p99 30.00 ms)" in result.output
    assert "webxr:       pose_transition" in result.output
    assert "pipeline:    sonic-v1.1" in result.output
    assert "pose_buffer: 7/10 (waiting)" in result.output
    assert "reference_handoff: planner->pose 40% of 0.50s" in result.output
    assert "transition:  operator_pose_toggle" in result.output
    coordinator.task_invoke.assert_any_call("sonic_teleop", "state_snapshot", {})


def test_status_reports_pose_to_planner_handoff(mocker) -> None:
    coordinator = _coordinator("sonic_teleop")
    coordinator.task_invoke.side_effect = [
        {
            **_state(armed=True, dry_run=False),
            "control_state": "control",
            "reference_source": "webxr_pose_to_planner",
            "webxr_teleop": {
                "mode": "planner_transition",
                "sonic_pipeline": "sonic-v1.1",
                "pose_window_frames": 10,
                "buffered_frames": 0,
                "stream_ready": False,
                "pose_transition_progress": 0.6,
                "pose_transition_seconds": 0.5,
                "last_transition_reason": "body_tracking_stale",
            },
        },
        {"state": "idle"},
    ]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["status"])

    assert result.exit_code == 0, result.output
    assert "reference:   webxr_pose_to_planner" in result.output
    assert "webxr:       planner_transition" in result.output
    assert "reference_handoff: pose->planner 60% of 0.50s" in result.output
    assert "transition:  body_tracking_stale" in result.output


def test_status_reports_fresh_planner_prepare(mocker) -> None:
    coordinator = _coordinator("sonic_teleop")
    coordinator.task_invoke.side_effect = [
        {
            **_state(armed=True, dry_run=False),
            "control_state": "control",
            "reference_source": "webxr_pose_held_for_planner",
            "webxr_teleop": {
                "mode": "planner_prepare",
                "sonic_pipeline": "sonic-low-latency",
                "pose_window_frames": 4,
                "buffered_frames": 0,
                "stream_ready": False,
                "planner_prepare_age_seconds": 0.32,
                "last_transition_reason": "body_tracking_stale",
            },
        },
        {"state": "idle"},
    ]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["status"])

    assert result.exit_code == 0, result.output
    assert "webxr:       planner_prepare" in result.output
    assert "reference_handoff: holding pose; fresh planner pending (0.32s)" in result.output


def test_arm_rejects_stack_without_lifecycle_task(mocker) -> None:
    coordinator = _coordinator()
    coordinator.list_tasks.return_value = ["joint_trajectory"]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["arm"])

    assert result.exit_code == 1
    assert "no G1 policy task" in result.output
    coordinator.set_activated.assert_not_called()


def test_arm_rejects_multiple_lifecycle_tasks(mocker) -> None:
    coordinator = _coordinator()
    coordinator.list_tasks.return_value = ["groot_wbc", "sonic_teleop"]
    lifecycle = {
        "arm": {},
        "disarm": {},
        "set_dry_run": {},
        "state_snapshot": {},
    }
    coordinator.describe_task.side_effect = lambda name: {
        "task": name,
        "commands": lifecycle,
    }
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["arm"])

    assert result.exit_code == 1
    assert "multiple G1 policy lifecycle tasks" in result.output
    coordinator.set_activated.assert_not_called()


def test_arm_forces_dry_run_before_activation_and_waits_for_armed(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["arm"])

    assert result.exit_code == 0, result.output
    assert coordinator.method_calls.index(mocker.call.set_dry_run(True)) < (
        coordinator.method_calls.index(mocker.call.set_activated(True))
    )
    assert "armed in dry-run" in result.output
    assert client.stopped


def test_enable_rejects_robot_that_has_not_completed_arming(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=False, dry_run=True, arming=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["enable"])

    assert result.exit_code == 1
    assert "not fully armed" in result.output
    coordinator.set_dry_run.assert_not_called()


def test_activate_arms_confirms_and_enables_in_order(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.side_effect = [
        _state(armed=False, dry_run=True),
        _state(armed=True, dry_run=True),
        _state(armed=True, dry_run=False),
    ]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    confirm = mocker.patch.object(g1_cli.typer, "confirm", return_value=True)

    result = runner.invoke(g1_cli.app, ["activate"])

    assert result.exit_code == 0, result.output
    lifecycle_calls = [
        call
        for call in coordinator.method_calls
        if call
        in (
            mocker.call.task_invoke("groot_wbc", "state_snapshot", {}),
            mocker.call.set_dry_run(True),
            mocker.call.set_activated(True),
            mocker.call.set_dry_run(False),
        )
    ]
    assert lifecycle_calls == [
        mocker.call.task_invoke("groot_wbc", "state_snapshot", {}),
        mocker.call.set_dry_run(True),
        mocker.call.set_activated(True),
        mocker.call.task_invoke("groot_wbc", "state_snapshot", {}),
        mocker.call.set_dry_run(False),
        mocker.call.task_invoke("groot_wbc", "state_snapshot", {}),
    ]
    confirm.assert_called_once_with("Enable live G1 policy motor output?", default=False)
    assert "G1 activated" in result.output
    assert client.stopped


def test_activate_decline_leaves_robot_armed_in_dry_run(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.side_effect = [
        _state(armed=False, dry_run=True),
        _state(armed=True, dry_run=True),
    ]
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    mocker.patch.object(g1_cli.typer, "confirm", return_value=False)

    result = runner.invoke(g1_cli.app, ["activate"])

    assert result.exit_code == 1
    assert "remains armed in dry-run" in result.output
    assert coordinator.set_dry_run.call_args_list == [mocker.call(True)]
    assert client.stopped


def test_activate_unavailable_confirmation_leaves_dry_run_enabled(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    mocker.patch.object(g1_cli.typer, "confirm", side_effect=g1_cli.typer.Abort())

    result = runner.invoke(g1_cli.app, ["activate"])

    assert result.exit_code == 1
    coordinator.set_dry_run.assert_not_called()
    assert client.stopped


def test_activate_timeout_never_confirms_or_enables(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=False, dry_run=True, arming=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    mocker.patch.object(g1_cli.time, "monotonic", side_effect=[0.0, 16.0])
    confirm = mocker.patch.object(g1_cli.typer, "confirm")

    result = runner.invoke(g1_cli.app, ["activate", "--timeout", "15"])

    assert result.exit_code == 1
    assert "did not finish arming" in result.output
    confirm.assert_not_called()
    assert coordinator.set_dry_run.call_args_list == [mocker.call(True)]
    assert client.stopped


def test_activate_already_enabled_skips_arm_and_confirmation(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=False)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    confirm = mocker.patch.object(g1_cli.typer, "confirm")

    result = runner.invoke(g1_cli.app, ["activate"])

    assert result.exit_code == 0, result.output
    assert "already activated" in result.output
    coordinator.set_activated.assert_not_called()
    coordinator.set_dry_run.assert_not_called()
    confirm.assert_not_called()


def test_ready_plans_both_arms_at_conservative_speed(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=False)
    coordinator.get_active_tasks.return_value = ["groot_wbc"]
    manipulation = Mock()
    manipulation.plan_to_joints.return_value = SimpleNamespace(succeeded=True)
    manipulation.execute.return_value = SimpleNamespace(succeeded=True)
    client = _Client(coordinator, manipulation)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["ready"])

    assert result.exit_code == 0, result.output
    targets = manipulation.plan_to_joints.call_args.args[0]
    assert set(targets) == {"g1_upper_body/left_arm", "g1_upper_body/right_arm"}
    assert tuple(targets["g1_upper_body/left_arm"].position) == G1_READY_JOINTS["left_arm"]
    assert tuple(targets["g1_upper_body/right_arm"].position) == G1_READY_JOINTS["right_arm"]
    assert manipulation.plan_to_joints.call_args.kwargs == {"speed_scale": G1_READY_SPEED_SCALE}
    manipulation.execute.assert_called_once_with(blocking=True)


def test_activate_ready_enables_before_moving_arms(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.side_effect = [
        _state(armed=True, dry_run=True),
        _state(armed=True, dry_run=False),
        _state(armed=True, dry_run=False),
    ]
    coordinator.get_active_tasks.return_value = ["groot_wbc"]
    manipulation = Mock()
    manipulation.plan_to_joints.return_value = SimpleNamespace(succeeded=True)
    manipulation.execute.return_value = SimpleNamespace(succeeded=True)
    client = _Client(coordinator, manipulation)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    mocker.patch.object(g1_cli.typer, "confirm", return_value=True)

    result = runner.invoke(g1_cli.app, ["activate", "--ready"])

    assert result.exit_code == 0, result.output
    coordinator.set_dry_run.assert_called_once_with(False)
    manipulation.execute.assert_called_once_with(blocking=True)
    assert result.output.index("G1 live policy output enabled") < result.output.index(
        "G1 reached the ready pose"
    )


def test_activate_ready_requires_manipulation_before_enabling(mocker) -> None:
    coordinator = _coordinator("sonic_teleop")
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["activate", "--ready"])

    assert result.exit_code == 1
    assert "required G1 manipulation RPCs" in result.output
    coordinator.task_invoke.assert_not_called()
    coordinator.set_dry_run.assert_not_called()
    coordinator.set_activated.assert_not_called()


def test_activate_ready_failure_reports_that_motor_output_remains_enabled(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.side_effect = [
        _state(armed=True, dry_run=True),
        _state(armed=True, dry_run=False),
        _state(armed=True, dry_run=False),
    ]
    coordinator.get_active_tasks.return_value = ["groot_wbc"]
    manipulation = Mock()
    manipulation.plan_to_joints.return_value = SimpleNamespace(succeeded=False)
    client = _Client(coordinator, manipulation)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)
    mocker.patch.object(g1_cli.typer, "confirm", return_value=True)

    result = runner.invoke(g1_cli.app, ["activate", "--ready"])

    assert result.exit_code == 1
    assert "G1 policy motor output remains enabled" in result.output
    manipulation.execute.assert_not_called()


def test_ready_rejects_active_teleoperation_before_planning(mocker) -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=False)
    coordinator.get_active_tasks.return_value = ["groot_wbc", "teleop_g1"]
    manipulation = Mock()
    client = _Client(coordinator, manipulation)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["ready"])

    assert result.exit_code == 1
    assert "disengage both hands" in result.output
    manipulation.plan_to_joints.assert_not_called()


def test_disable_attempts_every_safety_action(mocker) -> None:
    coordinator = _coordinator()
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["disable"])

    assert result.exit_code == 0, result.output
    coordinator.cancel_trajectory.assert_called_once_with()
    coordinator.set_dry_run.assert_called_once_with(True)
    coordinator.set_activated.assert_called_once_with(False)
