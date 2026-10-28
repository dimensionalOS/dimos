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

"""JPEG image transport with a standalone CDR CompressedImage envelope."""

from collections.abc import Callable
from typing import Any

from dimos.message_utils import image
from dimos.messages.sensor_msgs.msg import CompressedImage, Image
from dimos.protocol.pubsub.codec import CDRCodec
from dimos.protocol.pubsub.encoders import DecodingError, PubSubEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic


class JpegEncoderMixin(PubSubEncoderMixin[Topic, Image, bytes]):
    def encode(self, msg: Image, _: Topic) -> bytes:
        return CDRCodec(CompressedImage).encode(image.to_compressed(msg, format="jpeg"))

    def decode(self, msg: bytes, topic: Topic) -> Image:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.lcm_type is not Image:
            raise DecodingError("JPEG transport requires generated sensor_msgs/Image")
        return image.from_compressed(CDRCodec(CompressedImage).decode(msg))


class JpegLCM(LCMPubSubBase):
    """Adapt generated images at the byte-oriented LCM boundary."""

    _codec = JpegEncoderMixin()

    def publish(self, topic: Topic | str, message: Any) -> None:
        if isinstance(topic, str):
            topic = Topic(topic, Image)
        if not isinstance(message, Image):
            raise TypeError("JPEG transport requires a generated Image")
        super().publish(topic, self._codec.encode(message, topic))

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        def receive(payload: bytes, received: Topic) -> None:
            try:
                message = self._codec.decode(payload, received)
            except (DecodingError, ValueError):
                return
            callback(message, received)

        return super().subscribe(topic, receive)
