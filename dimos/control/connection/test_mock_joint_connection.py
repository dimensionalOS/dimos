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

"""Position stream behavior of the standalone mock, without a device SDK."""

import pytest

from dimos.control._control_test_helpers import fake_arm_model
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common._damiao_native_test_helpers import LocalTransport
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def rig(mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = MockJointConnection(
        model=fake_arm_model(("joint1", "joint2")),
        initial_positions={"joint1": 0.2, "joint2": 0.4},
        deadman_timeout_s=10,
    )
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    try:
        module.start()
        wait_until(lambda: bool(readings.sent), timeout=2)
        yield module, commands, readings
    finally:
        module.stop()


def test_partial_commands_report_all_joints_and_deadman_holds_the_pose(rig):
    module, commands, readings = rig
    assert readings.sent[-1].name == ["arm/joint1", "arm/joint2"]
    assert readings.sent[-1].position == [0.2, 0.4]
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    wait_until(lambda: readings.sent[-1].position == [0.7, 0.4], timeout=2)
    module._on_loop(lambda: module._deadman_expired(module._command_seq))
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    assert readings.sent[-1].position == [0.7, 0.4]
    assert readings.sent[-1].velocity == readings.sent[-1].effort == [0.0, 0.0]
    module.stop()
    assert not module.status().connected
    assert not module._reader.is_alive()


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 100.0])
def test_invalid_commands_do_not_change_mock_feedback(rig, bad):
    module, commands, readings = rig
    commands.publish(JointState(name=["arm/joint1"], position=[bad]))
    assert module.status().last_rejection
    assert readings.sent[-1].position == [0.2, 0.4]
