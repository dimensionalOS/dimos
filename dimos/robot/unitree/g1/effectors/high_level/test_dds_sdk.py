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

import pytest
from structlog.testing import capture_logs

pytest.importorskip("unitree_sdk2py.g1.loco.g1_loco_client")

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.effectors.high_level import dds_sdk


@pytest.fixture
def timer_factory(mocker):
    return mocker.patch("dimos.robot.unitree.motion_timer.threading.Timer")


@pytest.fixture
def sdk(mocker, timer_factory):
    mocker.patch.object(dds_sdk, "ChannelFactoryInitialize")
    switcher = mocker.patch.object(dds_sdk, "MotionSwitcherClient").return_value
    switcher.CheckMode.return_value = (0, {"name": "none"})
    switcher.SelectMode.return_value = (0, {})
    mocker.patch.object(dds_sdk, "LocoClient")
    mocker.patch.object(dds_sdk.time, "sleep")
    module = dds_sdk.G1HighLevelDdsSdk()
    try:
        yield module
    finally:
        module.stop()


def test_start_warns_when_cmd_vel_is_not_wired(sdk):
    with capture_logs() as logs:
        sdk.start()

    warnings = [entry["event"] for entry in logs if entry["log_level"] == "warning"]
    assert warnings == [
        "cmd_vel is not wired. Navigation velocity commands will not reach the G1. "
        "Connect cmd_vel in the blueprint."
    ]


def test_wired_cmd_vel_reaches_sdk_without_wiring_warning(sdk, mocker):
    transport = mocker.Mock()
    sdk.set_transport("cmd_vel", transport)
    with capture_logs() as logs:
        sdk.start()
    sdk.loco_client.Move.return_value = 0

    callback = transport.subscribe.call_args.args[0]
    callback(Twist(linear=Vector3(0.3, 0, 0)))

    sdk.loco_client.Move.assert_called_once_with(0.3, 0, 0, continous_move=True)
    assert [entry for entry in logs if entry["log_level"] == "warning"] == []
    sdk.stop()
    transport.subscribe.return_value.assert_called_once_with()


@pytest.mark.parametrize("code", [0, 7105, 3104, None])
def test_move_reports_sdk_status_and_always_arms_stop_timer(sdk, timer_factory, code):
    sdk.start()
    sdk.loco_client.Move.return_value = code
    with capture_logs() as logs:
        result = sdk.move(Twist(linear=Vector3(0.3, 0, 0)))

    assert result is (code in (0, None))
    assert [entry for entry in logs if entry["log_level"] == "warning"] == {
        0: [],
        7105: [{"event": "Move returned nonzero code", "code": 7105, "log_level": "warning"}],
        3104: [{"event": "Move returned nonzero code", "code": 3104, "log_level": "warning"}],
        None: [
            {
                "event": "Move returned no status code. Command acceptance is unknown",
                "log_level": "warning",
            }
        ],
    }[code]
    assert timer_factory.call_count == 1
    timer_factory.return_value.start.assert_called_once_with()
    assert timer_factory.call_args.args[0] == sdk.config.cmd_vel_timeout
    timer_factory.call_args.args[1]()
    sdk.loco_client.StopMove.assert_called_once_with()


@pytest.mark.parametrize("select_code, level", [(0, "warning"), (1, "error")])
def test_motion_mode_hints_match_documented_remote_sequence(sdk, select_code, level):
    switcher = dds_sdk.MotionSwitcherClient.return_value
    switcher.CheckMode.return_value = (0, {"name": "ai"})
    switcher.SelectMode.return_value = (select_code, {})

    with capture_logs() as logs:
        sdk.start()

    hints = [entry["event"] for entry in logs if "controller" in entry["event"]]
    assert len(hints) == (1 if select_code == 0 else 2)
    for hint in hints:
        assert "L2+B then L2+Up then R2+A" in hint
        assert "may vary based on the model" in hint
        assert "docs/platforms/humanoid/g1/index.md" in hint
        assert "L1" not in hint
    assert any(entry["log_level"] == level and "controller" in entry["event"] for entry in logs)


def test_missing_move_status_warns_only_once_and_preserves_auto_stop(sdk, timer_factory):
    sdk.start()
    sdk.loco_client.Move.return_value = None
    with capture_logs() as logs:
        assert sdk.move(Twist()) is True
        assert sdk.move(Twist()) is True

    assert [entry["event"] for entry in logs] == [
        "Move returned no status code. Command acceptance is unknown"
    ]
    assert timer_factory.call_count == 2
    assert timer_factory.call_args.args[0] == sdk.config.cmd_vel_timeout
    timer_factory.call_args.args[1]()
    sdk.loco_client.StopMove.assert_called_once_with()


def test_move_timeout_still_arms_stop_after_previous_stop_has_fired(sdk, timer_factory):
    sdk.start()
    # A reply timeout does not prove that the robot rejected the command.
    sdk.loco_client.Move.side_effect = [0, 3104]
    assert sdk.move(Twist(linear=Vector3(0.3, 0, 0))) is True
    timer_factory.call_args.args[1]()
    sdk.loco_client.StopMove.assert_called_once_with()

    assert sdk.move(Twist(linear=Vector3(0.4, 0, 0))) is False

    assert timer_factory.call_count == 2
    timer_factory.call_args.args[1]()
    assert sdk.loco_client.StopMove.call_count == 2
