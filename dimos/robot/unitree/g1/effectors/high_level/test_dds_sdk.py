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


import importlib.util
import sys
import threading
from unittest.mock import call

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist


@pytest.fixture
def sdk(monkeypatch, mocker):
    # Import the real module with the optional hardware SDK replaced on all platforms.
    for name in (
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client",
        "unitree_sdk2py.core.channel",
        "unitree_sdk2py.g1.loco.g1_loco_api",
        "unitree_sdk2py.g1.loco.g1_loco_client",
    ):
        monkeypatch.setitem(sys.modules, name, mocker.Mock())
    name = "dimos.robot.unitree.g1.effectors.high_level.dds_sdk"
    spec = importlib.util.find_spec(name)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    robot = module.G1HighLevelDdsSdk()
    robot.loco_client = mocker.Mock()
    robot.loco_client.SetVelocity.return_value = 0
    try:
        yield robot
    finally:
        robot.stop()


@pytest.mark.parametrize("replacement", ["stream", "timed", "request", "shutdown"])
def test_expired_watchdog_cannot_stop_the_next_command(sdk, mocker, replacement):
    timers = mocker.patch.object(threading, "Timer", side_effect=[mocker.Mock(), mocker.Mock()])
    assert sdk.move(Twist(linear=[0.25, 0, 0]))
    expired = timers.call_args.args[1]
    if replacement == "stream":
        assert sdk.move(Twist(linear=[0, 0.5, 0]))
    elif replacement == "timed":
        assert sdk.move(Twist(linear=[0, 0.5, 0]), duration=2.0)
    elif replacement == "request":
        result = sdk.publish_request(
            "rt/api/loco/request",
            {"api_id": 7105, "parameter": {"velocity": [0, 0.5, 0], "duration": 2.0}},
        )
        assert result == {"code": 0}
    else:
        sdk.stop()
    calls = sdk.loco_client.method_calls.copy()

    # Exercise an expiry already dispatched when its successor acquired the command lock.
    expired()
    assert sdk.loco_client.method_calls == calls
    if replacement == "stream":
        timers.call_args.args[1]()
        assert sdk.loco_client.method_calls == [*calls, call.StopMove()]


@pytest.mark.parametrize("rejected", ["timed", "request"])
def test_rejected_velocity_keeps_the_previous_watchdog(sdk, mocker, rejected):
    timers = mocker.patch.object(threading, "Timer")
    assert sdk.move(Twist(linear=[0.25, 0, 0]))
    expired = timers.call_args.args[1]
    sdk.loco_client.SetVelocity.return_value = 7
    if rejected == "timed":
        assert not sdk.move(Twist(linear=[0, 0.5, 0]), duration=2.0)
    else:
        assert sdk.publish_request("rt/api/loco/request", {"api_id": 7105}) == {"code": 7}
    timers.return_value.cancel.assert_not_called()
    expired()
    sdk.loco_client.StopMove.assert_called_once_with()
