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

from dimos.core.global_config import global_config
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.topic import Topic
from dimos.robot.cli.topic import _build_eval_context, _decode_typed_lcm_message, topic_send
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

    monkeypatch.setattr("dimos.robot.cli.topic.make_transport", capturing_make_transport)

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
