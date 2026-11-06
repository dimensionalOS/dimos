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

"""Real connection configurations require their CAN selectors before startup."""

from pydantic import ValidationError
import pytest

from dimos.robot.manipulators.damiao import damiao_connection
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnectionConfig
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnectionConfig
from dimos.robot.manipulators.makermods_metal.connection import MetalConnectionConfig
from dimos.robot.manipulators.openarm.connection import OpenArmConnectionConfig
from dimos.robot.manipulators.openyam.connection import OpenYamConnectionConfig


@pytest.mark.parametrize(
    "config_type,selectors",
    [
        (MetalConnectionConfig, ["can_port"]),
        (OpenYamConnectionConfig, ["can_port"]),
        (DualMetalConnectionConfig, ["left_can_port", "right_can_port"]),
        (DualOpenYamConnectionConfig, ["left_can_port", "right_can_port"]),
        (OpenArmConnectionConfig, ["left_can_port", "right_can_port"]),
    ],
)
def test_missing_or_none_selectors_are_rejected(config_type, selectors):
    with pytest.raises(ValidationError) as missing:
        config_type.model_validate({})
    assert [(error["loc"], error["type"]) for error in missing.value.errors()] == [
        ((name,), "missing") for name in selectors
    ]
    with pytest.raises(ValidationError) as none:
        config_type.model_validate(dict.fromkeys(selectors))
    assert [(error["loc"], error["type"]) for error in none.value.errors()] == [
        ((name,), "string_type") for name in selectors
    ]


@pytest.mark.parametrize("platform", ["linux", "darwin"])
def test_serial_bridge_selector_keeps_main_transport_support(platform, connection_class, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    mocker.patch.object(damiao_connection, "sys", mocker.Mock(platform=platform))
    serial = mocker.patch.object(damiao_connection.can_motor_control, "DmSerialBus")
    socket = mocker.patch.object(damiao_connection.can_motor_control, "SocketCanBus", create=True)
    usb = mocker.patch.object(damiao_connection.can_motor_control, "GsUsbBus", create=True)
    connection = connection_class()
    try:
        connection._bus_devices = {"left": "/dev/ttyACM0"}
        assert connection._make_can_bus("left") is serial.return_value
        serial.assert_called_once_with("/dev/ttyACM0")
        socket.assert_not_called()
        usb.assert_not_called()
    finally:
        connection.stop()
