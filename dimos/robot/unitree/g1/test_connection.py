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

"""The single G1 module selects I/O without loading a locomotion policy."""

import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
import dimos.robot.unitree.g1.connection as connection_module
from dimos.robot.unitree.g1.connection import G1Connection


@pytest.fixture
def connection_factory(mocker):
    mocker.patch.object(Module, "start")
    made = []

    def build(**kwargs):
        connection = G1Connection(**kwargs)
        mocker.patch.object(connection.cmd_vel, "subscribe", return_value=lambda: None)
        made.append(connection)
        return connection

    yield build
    for connection in made:
        connection.stop()


def test_stock_g1_keeps_velocity_and_vendor_rpc_path(connection_factory, mocker):
    device = mocker.patch.object(connection_module, "UnitreeWebRTCConnection").return_value
    connection = connection_factory(connection_type="webrtc", ip="unit-test.invalid")
    connection.start()
    command = Twist(linear=Vector3(0.2, 0, 0))
    connection.move(command, duration=2.0)
    device.move.assert_called_once_with(command, 2.0)
    connection.publish_request("test", {"data": 1})
    device.publish_request.assert_called_once_with("test", {"data": 1})
    assert connection.control_status()["controller"] == "Unitree firmware"


def test_simulation_needs_no_policy_or_velocity_controller_to_connect(connection_factory, mocker):
    hardware = mocker.patch.object(connection_module, "UnitreeWebRTCConnection")
    connection = connection_factory(connection_type="sim2")
    connection.start()
    hardware.assert_not_called()
    with pytest.raises(ValueError, match="velocity control is not configured"):
        connection.move(Twist())
    with pytest.raises(NotImplementedError, match="stock WebRTC"):
        connection.publish_request("test", {})


def test_sim_velocity_delegates_to_configured_controller(connection_factory, mocker):
    connection = connection_factory(connection_type="sim2")
    controller = mocker.patch.object(connection, "_velocity_controller")
    connection.start()
    connection.move(Twist(linear=Vector3(2, -2, 0)), duration=1.0)
    args = controller.set_velocity_command.call_args.args
    assert (args[0].linear.x, args[0].linear.y, args[1]) == (0.5, -0.3, 1.0)
    with pytest.raises(ValueError, match="finite"):
        connection.move(Twist(linear=Vector3(float("nan"), 0, 0)))


def test_dds_selection_uses_existing_driver_and_rejects_firmware_motion(connection_factory, mocker):
    driver = mocker.patch.object(connection_module, "G1DDSDevice").return_value
    connection = connection_factory(connection_type="dds")
    connection.start()
    driver.start.assert_called_once_with()
    with pytest.raises(ValueError, match="motor control owns"):
        connection.move(Twist())
    connection.cmd_vel.subscribe.assert_not_called()
