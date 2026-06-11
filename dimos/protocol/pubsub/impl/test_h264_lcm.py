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

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pytest

from dimos.msgs.protocol import DimosMsg
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.VideoPacket import VideoPacket
from dimos.protocol.pubsub.encoders import DecodingError, LCMTopicProto
from dimos.protocol.pubsub.impl.h264_lcm import H264LCM, H264EncoderMixin
from dimos.protocol.video.h264 import VideoDecodeGapError


@dataclass
class StubTopic:
    topic: str
    lcm_type: type[DimosMsg] | None = None


class FakeEncoder:
    def encode(self, image: Image) -> VideoPacket:
        return VideoPacket(
            seq=0,
            ts=image.ts,
            frame_id=image.frame_id,
            width=image.width,
            height=image.height,
            format=image.format.value,
            codec="h264",
            bitstream="annex_b",
            is_keyframe=True,
            keyframe_seq=0,
            pts=90,
            data=b"\x00\x00\x00\x01\x65",
        )


class FakeDecoder:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail

    def decode(self, packet: VideoPacket) -> Image:
        if self.fail:
            raise VideoDecodeGapError("waiting for keyframe")
        return Image(
            data=np.zeros((packet.height, packet.width, 3), dtype=np.uint8),
            format=ImageFormat(packet.format),
            frame_id=packet.frame_id,
            ts=packet.ts,
        )


class InMemoryPubSubBase:
    def __init__(self, **_: object) -> None:
        self._subscribers: list[tuple[LCMTopicProto, Callable[[bytes, LCMTopicProto], None]]] = []

    def publish(self, topic: LCMTopicProto, message: bytes) -> None:
        for subscribed_topic, callback in self._subscribers:
            if subscribed_topic.topic == topic.topic:
                callback(message, topic)

    def subscribe(
        self, topic: LCMTopicProto, callback: Callable[[bytes, LCMTopicProto], None]
    ) -> Callable[[], None]:
        item = (topic, callback)
        self._subscribers.append(item)

        def unsubscribe() -> None:
            self._subscribers.remove(item)

        return unsubscribe


class InMemoryH264PubSub(H264EncoderMixin, InMemoryPubSubBase):  # type: ignore[misc]
    pass


def test_h264_lcm_encodes_image_as_video_packet_bytes() -> None:
    transport = H264LCM()
    transport._encoder = FakeEncoder()  # type: ignore[assignment]
    image = Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")

    payload = transport.encode(image, StubTopic("/color", Image))
    packet = VideoPacket.lcm_decode(payload)

    assert packet.codec == "h264"
    assert packet.bitstream == "annex_b"
    assert packet.width == 3
    assert packet.height == 2
    assert packet.is_keyframe is True


def test_h264_lcm_decodes_video_packet_bytes_to_image() -> None:
    transport = H264LCM()
    transport._decoder = FakeDecoder()  # type: ignore[assignment]
    packet = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    image = transport.decode(packet.lcm_encode(), StubTopic("/color", Image))

    assert image.frame_id == "cam"
    assert image.shape == (2, 3, 3)


def test_h264_lcm_suppresses_decode_gap() -> None:
    transport = H264LCM()
    transport._decoder = FakeDecoder(fail=True)  # type: ignore[assignment]
    packet = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    with pytest.raises(DecodingError, match="waiting for keyframe"):
        transport.decode(packet.lcm_encode(), StubTopic("/color", Image))


def test_h264_lcm_suppresses_non_video_packet_payload() -> None:
    transport = H264LCM()

    with pytest.raises(DecodingError, match="Invalid VideoPacket payload"):
        transport.decode(b"not-a-video-packet", StubTopic("/color", Image))


def test_h264_lcm_publish_subscribe_delivers_decoded_image() -> None:
    transport = InMemoryH264PubSub()
    transport._encoder = FakeEncoder()  # type: ignore[assignment]
    transport._decoder = FakeDecoder()  # type: ignore[assignment]
    topic = StubTopic("/color", Image)
    received: list[Image] = []

    transport.subscribe(topic, lambda image, _topic: received.append(image))
    transport.publish(
        topic,
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam"),
    )

    assert len(received) == 1
    assert received[0].frame_id == "cam"
    assert received[0].shape == (2, 3, 3)


def test_h264_lcm_late_subscriber_waits_for_keyframe() -> None:
    transport = InMemoryH264PubSub()
    topic = StubTopic("/color", Image)
    received: list[Image] = []
    decoder = FakeDecoder(fail=True)
    transport._decoder = decoder  # type: ignore[assignment]

    transport.subscribe(topic, lambda image, _topic: received.append(image))
    delta_packet = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )
    InMemoryPubSubBase.publish(transport, topic, delta_packet.lcm_encode())

    decoder.fail = False
    keyframe_packet = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )
    InMemoryPubSubBase.publish(transport, topic, keyframe_packet.lcm_encode())

    assert len(received) == 1
    assert received[0].frame_id == "cam"
