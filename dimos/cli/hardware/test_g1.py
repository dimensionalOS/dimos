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


def test_hardware_namespace_exposes_g1_operator_commands() -> None:
    result = runner.invoke(hardware_cli.app, ["g1", "--help"])

    assert result.exit_code == 0, result.output
    for command in ("status", "arm", "enable", "ready", "disable"):
        assert command in result.output


def test_arm_forces_dry_run_before_activation_and_waits_for_armed(mocker) -> None:
    coordinator = Mock()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["arm"])

    assert result.exit_code == 0, result.output
    assert coordinator.method_calls[:2] == [
        mocker.call.set_dry_run(True),
        mocker.call.set_activated(True),
    ]
    assert "armed in dry-run" in result.output
    assert client.stopped


def test_enable_rejects_robot_that_has_not_completed_arming(mocker) -> None:
    coordinator = Mock()
    coordinator.task_invoke.return_value = _state(armed=False, dry_run=True, arming=True)
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["enable"])

    assert result.exit_code == 1
    assert "not fully armed" in result.output
    coordinator.set_dry_run.assert_not_called()


def test_ready_plans_both_arms_at_conservative_speed(mocker) -> None:
    coordinator = Mock()
    coordinator.task_invoke.return_value = _state(armed=True, dry_run=False)
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


def test_disable_attempts_every_safety_action(mocker) -> None:
    coordinator = Mock()
    client = _Client(coordinator)
    mocker.patch.object(g1_cli.Dimos, "connect", return_value=client)

    result = runner.invoke(g1_cli.app, ["disable"])

    assert result.exit_code == 0, result.output
    coordinator.cancel_trajectory.assert_called_once_with()
    coordinator.set_dry_run.assert_called_once_with(True)
    coordinator.set_activated.assert_called_once_with(False)
