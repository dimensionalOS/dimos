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
import typer

from dimos.cli.topic import (
    _build_eval_context,
    _decode_typed_lcm_message,
    _split_topic,
    topic_echo,
    topic_send,
)
from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.utils.testing.collector import CallbackCollector


def test_decode_typed_lcm_message_resolves_message_submodule() -> None:
    msg = CameraInfo(
        width=1920,
        height=1080,
        distortion_model="plumb_bob",
        frame_id="camera_optical",
    )

    decoded = _decode_typed_lcm_message(
        "/camera_info#sensor_msgs.CameraInfo",
        msg.lcm_encode(),
    )

    assert isinstance(decoded, CameraInfo)
    assert decoded.width == 1920
    assert decoded.height == 1080
    assert decoded.frame_id == "camera_optical"
    assert decoded.distortion_model == "plumb_bob"


def test_build_eval_context_maps_message_names_to_classes() -> None:
    context = _build_eval_context()

    for name in ("Twist", "Vector3", "PoseStamped"):
        cls = context[name]
        assert isinstance(cls, type)
        assert cls.__name__ == name


def test_topic_send_delivers_over_lcm(monkeypatch: pytest.MonkeyPatch, lcm_url: str) -> None:
    monkeypatch.setattr(global_config, "transport", "lcm")

    # topic_send never stops the transport it creates; capture it so its LCM
    # threads can be stopped (the thread-leak check fails the test otherwise).
    transports: list[PubSubTransport[object]] = []

    def capturing_make_transport(name: str, msg_type: type) -> PubSubTransport[object]:
        transport: PubSubTransport[object] = make_transport(name, msg_type)
        transports.append(transport)
        return transport

    monkeypatch.setattr("dimos.cli.topic.make_transport", capturing_make_transport)

    bus = LCM(url=lcm_url)
    bus.start()
    collector = CallbackCollector(1)
    bus.subscribe(Topic(topic="/test_topic_send", lcm_type=Twist), collector)

    try:
        topic_send("/test_topic_send", "Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))")
        collector.wait()
    finally:
        for transport in transports:
            transport.stop()
        bus.stop()

    assert collector.results[0][0] == Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))


@pytest.mark.parametrize(
    ("transport", "topic", "expected"),
    [
        ("zenoh", "cmd_vel", ("cmd_vel", None)),
        ("zenoh", "/cmd_vel", ("/cmd_vel", None)),
        ("zenoh", "/cmd_vel#geometry_msgs.Twist", ("/cmd_vel", Twist)),
        ("zenoh", "dimos/cmd_vel", ("cmd_vel", None)),
        ("zenoh", "dimos/cmd_vel/geometry_msgs.Twist", ("cmd_vel", Twist)),
        ("zenoh", "dimos/robot0/cmd_vel/geometry_msgs.Twist", ("robot0/cmd_vel", Twist)),
        ("lcm", "/cmd_vel", ("/cmd_vel", None)),
        ("lcm", "/cmd_vel#geometry_msgs.Twist", ("/cmd_vel", Twist)),
    ],
)
def test_split_topic(
    monkeypatch: pytest.MonkeyPatch, transport: str, topic: str, expected: tuple[str, type | None]
) -> None:
    monkeypatch.setattr(global_config, "transport", transport)

    assert _split_topic(topic) == expected


def test_split_topic_rejects_unknown_type(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(typer.Exit):
        _split_topic("/cmd_vel#geometry_msgs.Nope")

    assert "geometry_msgs.Nope" in capsys.readouterr().err


def test_topic_echo_reports_invalid_zenoh_key(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")

    with pytest.raises(typer.Exit):
        topic_echo("cmd?vel", None)

    assert "Invalid topic 'cmd?vel'" in capsys.readouterr().err
