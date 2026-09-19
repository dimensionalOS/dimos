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

from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.robot.unitree.go2.operator_controls import (
    Go2OperatorControls,
    OperatorCommand,
    decode_operator_command,
)


@pytest.fixture
def controls(monkeypatch: pytest.MonkeyPatch) -> Go2OperatorControls:
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    module = Go2OperatorControls()
    module.go2 = MagicMock()
    module.go2_operator_state = MagicMock()
    module.go2_operator_result = MagicMock()
    return module


@pytest.mark.parametrize(
    "value",
    [
        {"id": "x", "action": "light", "level": True},
        {"id": "x", "action": "light", "level": 11},
        {"id": "x", "action": "light", "level": 1.5},
        {"id": "x", "action": "arbitrary-api"},
        {"id": "", "action": "StandReady"},
    ],
)
def test_invalid_command_rejected(value):
    with pytest.raises(ValueError):
        decode_operator_command(value)


def test_stand_aborts_sequence_on_driver_rejection(controls):
    controls.go2.standup.return_value = False
    controls._command(OperatorCommand("request", "StandReady"))
    controls.go2.balance_stand.assert_not_called()
    assert not controls.go2_operator_result.publish.call_args.args[0].ok


def test_light_feedback_only_changes_on_success(controls):
    controls.go2.set_light.return_value = False
    controls._command(decode_operator_command({"id": "a", "action": "light", "level": 8}))
    assert controls._light_requested is None
    controls.go2.set_light.return_value = True
    controls._command(decode_operator_command({"id": "b", "action": "light", "level": 4}))
    controls.go2.set_light.assert_called_with(4)
    assert controls._light_requested == 4
    assert controls.go2_operator_result.publish.call_args.args[0].id == "b"


def test_busy_command_rejected_without_robot_call(controls):
    with controls._command_lock:
        controls._command(OperatorCommand("request", "Hello"))
    controls.go2.sport_command.assert_not_called()
    assert not controls.go2_operator_result.publish.call_args.args[0].ok


def test_driver_exception_releases_command_lock(controls):
    controls.go2.sport_command.side_effect = RuntimeError("link lost")
    controls._command(OperatorCommand("request", "Hello"))
    assert not controls._command_lock.locked()
    assert "unknown" in controls.go2_operator_result.publish.call_args.args[0].message


def test_missing_battery_is_not_zero(controls):
    controls.go2.battery_soc.return_value = None
    controls._publish_state()
    assert controls.go2_operator_state.publish.call_args.args[0].battery is None
