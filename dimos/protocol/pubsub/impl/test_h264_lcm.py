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
from dimos.msgs.sensor_msgs.Image import H264_IMAGE_ENCODING, Image, ImageFormat
from dimos.protocol.pubsub.encoders import DecodingError, LCMTopicProto
from dimos.protocol.pubsub.impl.h264_lcm import H264LCM, H264EncoderMixin
from dimos.protocol.video.h264 import VideoDecodeGapError


@dataclass
class StubTopic:
    topic: str
    lcm_type: type[DimosMsg] | None = None


def _encoded(image: Image, *, seq: int = 0, key: bool = True) -> Image:
    return Image.encoded(
        data=b"\x00\x00\x00\x01\x65" if key else b"\x00\x00\x00\x01\x41",
        encoding=H264_IMAGE_ENCODING,
        format=image.format,
        frame_id=image.frame_id,
        ts=image.ts,
        codec_metadata={
            "seq": seq,
            "codec": "h264",
            "bitstream": "annex_b",
            "is_keyframe": key,
            "keyframe_seq": seq if key else 0,
            "pts": seq * 90,
            "width": image.width,
            "height": image.height,
            "channels": image.channels,
            "dtype": str(image.dtype),
        },
    )


class FakeEncoder:
    def encode(self, image: Image) -> Image:
        return _encoded(image)


class FakeDecoder:
    def __init__(self, *, fail: bool = False, invalid: bool = False) -> None:
        self.fail = fail
        self.invalid = invalid

    def decode(self, image: Image) -> Image:
        if self.fail:
            raise VideoDecodeGapError("waiting for keyframe")
        if self.invalid:
            raise ValueError("Expected H.264 encoded Image")
        return Image(
            data=np.zeros((image.height, image.width, 3), dtype=np.uint8),
            format=image.format,
            frame_id=image.frame_id,
            ts=image.ts,
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


def test_h264_lcm_encodes_image_as_h264_encoded_image_bytes() -> None:
    transport = H264LCM()
    transport._encoder = FakeEncoder()  # type: ignore[assignment]
    image = Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")

    payload = transport.encode(image, StubTopic("/color", Image))
    encoded = Image.lcm_decode(payload)

    assert encoded.encoding == H264_IMAGE_ENCODING
    assert encoded.codec_metadata["codec"] == "h264"
    assert encoded.codec_metadata["bitstream"] == "annex_b"
    assert encoded.width == 3
    assert encoded.height == 2
    assert encoded.codec_metadata["is_keyframe"] is True


def test_h264_lcm_decodes_h264_image_bytes_to_raw_image() -> None:
    transport = H264LCM()
    transport._decoder = FakeDecoder()  # type: ignore[assignment]
    encoded = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    image = transport.decode(encoded.lcm_encode(), StubTopic("/color", Image))

    assert image.encoding == "raw"
    assert image.frame_id == "cam"
    assert image.shape == (2, 3, 3)


def test_h264_lcm_decode_false_returns_encoded_image() -> None:
    transport = H264LCM(decode_images=False)
    encoded = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    image = transport.decode(encoded.lcm_encode(), StubTopic("/color", Image))

    assert image.encoding == H264_IMAGE_ENCODING
    assert image.frame_id == "cam"


def test_h264_lcm_suppresses_decode_gap() -> None:
    transport = H264LCM()
    transport._decoder = FakeDecoder(fail=True)  # type: ignore[assignment]
    encoded = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    with pytest.raises(DecodingError, match="waiting for keyframe"):
        transport.decode(encoded.lcm_encode(), StubTopic("/color", Image))


def test_h264_lcm_suppresses_invalid_h264_image() -> None:
    transport = H264LCM()
    transport._decoder = FakeDecoder(invalid=True)  # type: ignore[assignment]
    encoded = FakeEncoder().encode(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam")
    )

    with pytest.raises(DecodingError, match="Expected H.264 encoded Image"):
        transport.decode(encoded.lcm_encode(), StubTopic("/color", Image))


def test_h264_lcm_suppresses_non_image_payload() -> None:
    transport = H264LCM()

    with pytest.raises(DecodingError):
        transport.decode(b"not-an-image", StubTopic("/color", Image))


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
    delta = _encoded(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam"),
        seq=1,
        key=False,
    )
    InMemoryPubSubBase.publish(transport, topic, delta.lcm_encode())

    decoder.fail = False
    keyframe = _encoded(
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="cam"),
        seq=2,
        key=True,
    )
    InMemoryPubSubBase.publish(transport, topic, keyframe.lcm_encode())

    assert len(received) == 1
    assert received[0].frame_id == "cam"
