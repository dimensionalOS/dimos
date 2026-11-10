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

"""Readiness covers the complete hosted startup transaction."""

import pytest

from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
from dimos.utils.testing.waiting import wait_until


def test_failed_telemetry_setup_never_reports_ready(connection_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class()
    readings = LocalTransport()
    module.set_transport("joint_state", readings)
    observed = []

    def refuse():
        observed.append(module.status().connected)
        raise RuntimeError("telemetry setup refused")

    mocker.patch.object(module, "_start_telemetry", side_effect=refuse)
    try:
        with pytest.raises(RuntimeError, match="telemetry setup refused"):
            module.start()
        assert observed == [False]
        assert readings.sent == []
        assert not module.status().connected
        assert module._arm is None
        assert "disable" in boundary[0].events
    finally:
        module.stop()


def test_first_state_follows_telemetry_setup_and_ready_log(connection_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class()
    readings = LocalTransport()
    module.set_transport("joint_state", readings)
    info = mocker.spy(__import__("dimos.robot.manipulators.damiao.connection", fromlist=["logger"]).logger, "info")
    try:
        module.start()
        reading = module.read_measured()
        module.publish_measured(reading)
        wait_until(lambda: bool(readings.sent), timeout=2)
        assert readings.sent[0].name == [
            "arm/joint1",
            "arm/joint2",
            "arm/joint3",
            "arm/left_finger",
            "arm/right_finger",
        ]
        info.assert_any_call("Robot connection ready", connection="Connection", source="arm")
    finally:
        module.stop()
