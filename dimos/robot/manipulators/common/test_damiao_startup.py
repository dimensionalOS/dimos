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

"""Readiness covers the complete startup transaction, including stream setup."""

import pytest

from dimos.robot.manipulators.common import damiao_connection as common
from dimos.robot.manipulators.common._damiao_native_test_helpers import LocalTransport
from dimos.utils.testing.waiting import wait_until


@pytest.mark.parametrize("phase", ["subscription", "reader"])
def test_failed_stream_setup_never_reports_ready(connection_class, boundary, mocker, phase):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class()
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    observed = []

    def refuse(*args, **kwargs):
        observed.append(module.status().connected)
        raise RuntimeError("stream setup refused")

    if phase == "subscription":
        mocker.patch.object(commands, "subscribe", side_effect=refuse)
    else:
        mocker.patch.object(common.threading.Thread, "start", side_effect=refuse)
    try:
        with pytest.raises(RuntimeError, match="stream setup refused"):
            module.start()
        assert observed == [False]
        assert readings.sent == []
        assert not module.status().connected
        assert module._robot is None
        assert "disable" in boundary[0].events
    finally:
        module.stop()


def test_first_state_follows_stream_setup_and_ready_log(connection_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class()
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    starting = []
    publishing = []
    start_thread = common.threading.Thread.start
    info = mocker.spy(common.logger, "info")

    def start_reader(thread):
        starting.append(module.status().connected)
        start_thread(thread)

    def receive(message):
        publishing.append(module.status().connected)

    mocker.patch.object(common.threading.Thread, "start", autospec=True, side_effect=start_reader)
    readings.subscribe(receive)
    try:
        module.start()
        wait_until(lambda: bool(publishing), timeout=2)
        assert starting == [False]
        assert all(publishing)
        assert readings.sent[0].name == [
            "arm/joint1",
            "arm/joint2",
            "arm/joint3",
            "arm/left_gripper",
            "arm/right_gripper",
        ]
        info.assert_any_call("Robot connection ready", connection="Connection", source="arm")
    finally:
        module.stop()
