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

"""Exercise schema advertisement and CDR command routing without physical sensors."""

from collections.abc import Iterator

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos_generated.std_msgs.msg import Bool, Header
from dimos_message_build.registry import (
    decode as cdr_decode,
    encode as cdr_encode,
    schema as cdr_schema,
)
from fastapi.testclient import TestClient
import pytest

from dimos.msgs.protocol import DimosMsg
from dimos.teleop.phone.phone_extensions import SimplePhoneTeleop
from dimos.teleop.phone.phone_teleop_module import PhoneTeleopModule
from dimos.web.relay_bridge.protocol import FrameHeader, encode_data_frame


def frame(channel: str, message: DimosMsg, *, type_name: str | None = None) -> bytes:
    return encode_data_frame(
        FrameHeader(
            ch=channel,
            seq=1,
            ts=0,
            delivery="latest",
            meta={"type": type_name or message.__msgtype__, "encoding": "cdr"},
        ),
        cdr_encode(message),
    )


@pytest.fixture
def module() -> Iterator[PhoneTeleopModule]:
    instance = PhoneTeleopModule()
    try:
        yield instance
    finally:
        instance.stop()


def test_websocket_and_schema_endpoint_use_generated_contract(module: PhoneTeleopModule) -> None:
    with TestClient(module._web_server.app) as client:
        schemas = client.get("/teleop/schema").json()
        assert schemas["sensors"] == {
            "type": TwistStamped.__msgtype__,
            "definition": cdr_schema(TwistStamped.__msgtype__),
        }
        assert schemas["button"] == {
            "type": Bool.__msgtype__,
            "definition": cdr_schema(Bool.__msgtype__),
        }
        with client.websocket_connect("/ws") as ws:
            ws.send_bytes(frame("button", Bool(data=True)))
            ws.send_bytes(
                frame(
                    "sensors",
                    TwistStamped(
                        header=Header(frame_id="phone", stamp=Time(sec=0, nanosec=0)),
                        twist=Twist(
                            linear=Vector3(x=0.0, y=0.0, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0),
                        ),
                    ),
                )
            )
        assert module._teleop_button is True
        assert module._current_sensors.header.frame_id == "phone"


@pytest.mark.parametrize(
    "wire",
    [
        b"",
        cdr_encode(Bool(data=True)),
        frame("unknown", Bool(data=True)),
        frame("button", Bool(data=True), type_name="std_msgs/msg/Int8"),
        frame("sensors", Bool(data=True)),
        frame("button", Bool(data=True)) + b"trailing",
    ],
)
def test_wrong_identity_or_legacy_unframed_bytes_are_rejected(
    module: PhoneTeleopModule, wire: bytes
) -> None:
    assert module._dispatch_binary_message(wire) is False
    assert module._teleop_button is False
    assert module._current_sensors is None


def test_generated_control_math_preserves_exact_stamp_and_wraps_yaw(
    module: PhoneTeleopModule,
) -> None:
    initial = TwistStamped(
        twist=Twist(linear=Vector3(z=350, x=0.0, y=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    current = TwistStamped(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id=""),
        twist=Twist(linear=Vector3(x=15, y=-30, z=10), angular=Vector3(z=60, x=0.0, y=0.0)),
    )
    module._on_sensors_bytes(cdr_encode(initial))
    assert module._engage()
    module._on_sensors_bytes(cdr_encode(current))
    output = module._get_output_twist()
    decoded = cdr_decode(cdr_encode(output), TwistStamped)
    assert decoded.header.stamp.nanosec == 123456789
    assert decoded.header.frame_id == "phone"
    assert decoded.twist.linear.x == 1.0
    assert decoded.twist.linear.y == -0.5
    assert decoded.twist.linear.z == pytest.approx(20 / 30)
    assert decoded.twist.angular.z == 2.0


def test_ground_robot_extension_publishes_generated_twist() -> None:
    module = SimplePhoneTeleop()
    outputs: list[Twist] = []
    unsubscribe = module.cmd_vel.subscribe(outputs.append)
    try:
        module._publish_msg(
            TwistStamped(
                twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
        result = cdr_decode(cdr_encode(outputs[0]), Twist)
        assert result.linear.x == 1
        assert result.linear.y == 2
        assert result.linear.z == 0
        assert result.angular.z == 3
    finally:
        unsubscribe()
        module.stop()
