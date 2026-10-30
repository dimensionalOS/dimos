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

from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.g1.connection import G1Connection, G1ConnectionConfig
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1DdsDevice
from dimos.robot.unitree.g1.effectors.high_level.webrtc import G1WebRtcDevice
from dimos.robot.unitree.g1.motor_device import G1MotorDevice


@pytest.fixture
def dds(mocker):
    channel = mocker.Mock()
    motion = mocker.Mock()
    motion.SelectMode.return_value = (0, {})
    loco = mocker.Mock()
    loco.Move.return_value = 0
    loco.StopMove.return_value = 0
    loco.SetFsmId.return_value = 0
    loco.StandUp2Squat.return_value = 0
    loco.Damp.return_value = 0
    loco._Call.return_value = (0, '{"data":200}')
    motion_type = mocker.Mock(return_value=motion)
    loco_type = mocker.Mock(return_value=loco)
    definitions = {
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client": {
            "MotionSwitcherClient": motion_type
        },
        "unitree_sdk2py.core.channel": {"ChannelFactoryInitialize": channel},
        "unitree_sdk2py.g1.loco.g1_loco_api": {"ROBOT_API_ID_LOCO_GET_FSM_ID": 7001},
        "unitree_sdk2py.g1.loco.g1_loco_client": {"LocoClient": loco_type},
    }
    modules = {}
    for path, attributes in definitions.items():
        parts = path.split(".")
        for depth in range(1, len(parts) + 1):
            name = ".".join(parts[:depth])
            modules.setdefault(name, ModuleType(name))
        vars(modules[path]).update(attributes)
    mocker.patch.dict(sys.modules, modules)
    mocker.patch("dimos.robot.unitree.g1.effectors.high_level.dds_sdk.time.sleep")
    device = G1DdsDevice(G1ConnectionConfig(source="g1_a", network_interface="test-nic"))
    try:
        yield SimpleNamespace(
            device=device, channel=channel, motion=motion, loco=loco, constructor=loco_type
        )
    finally:
        device.stop()


@pytest.fixture
def webrtc(mocker):
    wire = mocker.Mock()
    wire.move.return_value = True
    constructor = mocker.patch(
        "dimos.robot.unitree.g1.effectors.high_level.webrtc.UnitreeWebRTCConnection",
        return_value=wire,
    )
    config = G1ConnectionConfig(
        g=GlobalConfig(robot_ip="192.0.2.1"), source="g1_b", aes_128_key="test-key"
    )
    device = G1WebRtcDevice(config)
    try:
        yield SimpleNamespace(device=device, wire=wire, constructor=constructor)
    finally:
        device.stop()


def test_dds_construction_has_no_hardware_side_effect(dds):
    dds.constructor.assert_not_called()
    dds.channel.assert_not_called()


def test_dds_start_selects_stock_mode_without_releasing_sport_controller(dds):
    dds.device.start()
    dds.channel.assert_called_once_with(0, "test-nic")
    dds.motion.SelectMode.assert_called_once_with("ai")
    dds.motion.ReleaseMode.assert_not_called()
    dds.loco.Move.assert_not_called()


def test_dds_mode_selection_failure_is_not_reported_as_connected(dds):
    dds.motion.SelectMode.return_value = (12, {})
    with pytest.raises(RuntimeError, match="select motion mode.*12"):
        dds.device.start()
    dds.loco.Move.assert_not_called()


def test_dds_velocity_keeps_stock_deadzone_compensation(dds):
    dds.device.start()
    dds.device.write({"g1_a/base/vx": 0.01, "g1_a/base/vy": -0.1, "g1_a/base/wz": -0.05})
    dds.loco.Move.assert_called_once_with(0.05, -0.1, -0.2, continous_move=True)
    dds.device.halt()
    dds.loco.StopMove.assert_called_once_with()


def test_dds_rejected_command_is_an_error(dds):
    dds.device.start()
    dds.loco.Move.return_value = 7
    with pytest.raises(RuntimeError, match="move.*7"):
        dds.device.write({"g1_a/base/vx": 0, "g1_a/base/vy": 0, "g1_a/base/wz": 0})


def test_dds_stock_state_and_posture_methods_use_firmware_api(dds):
    dds.device.start()
    assert dds.device.get_state() == "AI_MODE"
    assert dds.device.stand_up()
    dds.loco.SetFsmId.assert_called_once_with(706)
    assert dds.device.lie_down()
    dds.loco.StandUp2Squat.assert_called_once_with()
    dds.loco.Damp.assert_called_once_with()


def test_dds_unknown_vendor_request_is_explicitly_unsupported(dds):
    dds.device.start()
    with pytest.raises(NotImplementedError, match="vendor requests"):
        dds.device.publish_request("rt/api/arm/request", {"api_id": 7106})


def test_webrtc_lifecycle_is_owned_by_device(webrtc):
    webrtc.constructor.assert_not_called()
    webrtc.device.start()
    webrtc.constructor.assert_called_once_with("192.0.2.1", "ai", aes_128_key="test-key")
    webrtc.wire.start.assert_called_once_with()
    webrtc.device.halt()
    webrtc.wire.stop_movement.assert_called_once_with()
    webrtc.device.stop()
    webrtc.wire.stop.assert_called_once_with()


def test_webrtc_velocity_preserves_body_axes(webrtc):
    webrtc.device.start()
    webrtc.device.write({"g1_b/base/vx": 0.3, "g1_b/base/vy": -0.1, "g1_b/base/wz": 0.2})
    twist = webrtc.wire.move.call_args.args[0]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.3, -0.1, 0.2)


@pytest.mark.parametrize(
    "mode, transport, expected",
    [
        ("stock", "webrtc", G1WebRtcDevice),
        ("stock", "dds", G1DdsDevice),
        ("motors", "webrtc", G1MotorDevice),
    ],
)
def test_same_connection_selects_only_the_requested_hardware_device(mode, transport, expected):
    module = G1Connection(
        g=GlobalConfig(simulation=""), control_mode=mode, stock_transport=transport
    )
    try:
        assert isinstance(module.create_hardware_device(), expected)
    finally:
        module.stop()
