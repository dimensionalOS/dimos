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

from collections.abc import Iterator

from dimos_cdrgen.registry import encode
from fastapi.testclient import TestClient
import pytest

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos.messages.std_msgs.msg import Bool, Header
from dimos.teleop.phone.phone_teleop_module import PhoneTeleopModule
from dimos.web.relay_bridge.protocol import FrameHeader, encode_data_frame


@pytest.fixture
def module() -> Iterator[PhoneTeleopModule]:
    module = PhoneTeleopModule()
    try:
        yield module
    finally:
        module.stop()


def _frame(channel, message):
    return encode_data_frame(
        FrameHeader(
            ch=channel,
            seq=0,
            ts=1.0,
            delivery="latest",
            meta={"encoding": "cdr", "type": message.__msgtype__},
        ),
        encode(message),
    )


def _sensors(roll, pitch, yaw):
    return TwistStamped(
        Header(Time(17, 1), "phone"), Twist(Vector3(roll, pitch, yaw), Vector3(1.0, 2.0, 3.0))
    )


def test_phone_cdr_commands_preserve_stamp_and_tilt_wrap(module):
    assert module._dispatch_binary_message(_frame("sensors", _sensors(1.0, 2.0, 350.0)))
    assert module._dispatch_binary_message(_frame("button", Bool(True)))
    module._handle_engage()
    assert module._dispatch_binary_message(_frame("sensors", _sensors(4.0, 8.0, 10.0)))
    output = module._get_output_twist()
    assert output is not None
    assert output.header == Header(Time(17, 1), "phone")
    assert output.twist.linear.x == pytest.approx(-6.0 / 30.0)
    assert output.twist.linear.y == pytest.approx(-3.0 / 30.0)
    assert output.twist.linear.z == pytest.approx(20.0 / 30.0)
    assert output.twist.angular.z == pytest.approx(3.0 / 30.0)
    assert module._dispatch_binary_message(_frame("button", Bool(False)))
    module._handle_engage()
    assert module._is_engaged is False


def test_phone_schema_and_codec_are_served_by_the_actual_app(module):
    with TestClient(module._web_server.app) as client:
        schemas = client.get("/teleop/schema").json()
        assert schemas["sensors"]["type"] == "geometry_msgs/msg/TwistStamped"
        assert schemas["button"]["type"] == "std_msgs/msg/Bool"
        assert client.get("/command-assets/command_codec.mjs").status_code == 200
        assert "CommandEncoder" in client.get("/teleop").text


@pytest.mark.parametrize("payload", [b"legacy LCM payload", b"", b"bad!"])
def test_phone_rejects_noncanonical_frames(module, payload):
    assert module._dispatch_binary_message(payload) is False
    assert module._current_sensors is None
