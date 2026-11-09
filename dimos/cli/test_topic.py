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

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.cli.topic import (
    _build_eval_context,
    _decode_typed_lcm_message,
    _resolve_type,
    topic_echo,
    topic_send,
)
from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.message_utils import camera_info
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Twist, Vector3
from dimos.messages.sensor_msgs.msg import CameraInfo
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.pubsub.codec import describe_message
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.utils.testing.collector import CallbackCollector


def test_decode_typed_lcm_message_resolves_message_submodule() -> None:
    msg = camera_info.from_intrinsics(
        500.0, 510.0, 320.0, 240.0, 1920, 1080, header=Header(Time(4, 125), "camera_optical")
    )

    decoded = _decode_typed_lcm_message(
        "/camera_info#sensor_msgs/msg/CameraInfo.cdr.v1",
        describe_message(CameraInfo).codec.encode(msg),
    )

    assert isinstance(decoded, CameraInfo)
    assert decoded.width == 1920
    assert decoded.height == 1080
    assert decoded.header.frame_id == "camera_optical"
    assert decoded.distortion_model == "plumb_bob"


def test_build_eval_context_maps_message_names_to_classes() -> None:
    context = _build_eval_context()

    for name in ("Twist", "Vector3", "PoseStamped"):
        cls = context[name]
        assert isinstance(cls, type)
        assert cls.__msgtype__.rsplit("/", 1)[-1] == name


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


def test_resolve_type_accepts_registered_external_messages(monkeypatch) -> None:
    class CustomState:
        pass

    monkeypatch.setattr(
        "dimos.cli.topic.message_types", lambda: {"custom_msgs/msg/CustomState": CustomState}
    )
    assert _resolve_type("CustomState") is CustomState
    assert _build_eval_context()["CustomState"] is CustomState
    assert _build_eval_context()["custom_msgs"].CustomState is CustomState


def test_topic_echo_uses_running_graph_for_raw_cdr_channel(monkeypatch, capsys) -> None:
    source = MagicMock()
    source.list_stream_descriptors.return_value = [
        SimpleNamespace(
            channel="dimos/odom", type_name="generated.Vector3", module="PoseSource", name="odom"
        )
    ]
    stream = source.get_module.return_value.get_stream.return_value
    unsubscribe = MagicMock()

    def subscribe(callback):
        callback(Vector3(1.0, 2.0, 3.0))
        return unsubscribe

    stream.subscribe.side_effect = subscribe
    monkeypatch.setattr("dimos.cli.topic.RemoteModuleSource", lambda **kwargs: source)
    monkeypatch.setattr("dimos.cli.topic._listen_forever", lambda *args: None)
    topic_echo("/odom", None)
    source.get_module.assert_called_once_with("PoseSource")
    source.get_module.return_value.get_stream.assert_called_once_with("odom")
    unsubscribe.assert_called_once()
    source.close.assert_called_once()
    assert "x=1.0, y=2.0, z=3.0" in capsys.readouterr().out
