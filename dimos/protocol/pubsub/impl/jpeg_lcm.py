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

"""JPEG image transport with a CDR CompressedImage envelope.

Split from lcmpubsub.py so that importing CborLCM / LCM does not
transitively pull in ``dimos.msgs.sensor_msgs.Image`` (and its heavy
cv2 / rerun dependencies).

Generated images (``dimos_messages.sensor_msgs.msg.Image``) travel as a raw CDR
``CompressedImage``. Legacy ``dimos.msgs.sensor_msgs.Image`` values keep their
existing LCM JPEG envelope (``Image.lcm_jpeg_encode``), so consumers that have
not migrated see no change.
"""

from __future__ import annotations

from typing import Any, cast

from dimos_messages.sensor_msgs.msg import CompressedImage, Image

from dimos.message_utils import image
from dimos.protocol.pubsub.codec import CDRCodec
from dimos.protocol.pubsub.encoders import DecodingError, PubSubEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic


class JpegEncoderMixin(PubSubEncoderMixin[Topic, Any, bytes]):
    """JPEG-encode either Image family at the byte-oriented LCM boundary."""

    def encode(self, msg: Any, topic: Topic | str) -> bytes:
        descriptor = getattr(topic, "message", None)
        if descriptor is not None:
            if descriptor.encoding == "cdr":
                return self._encode_generated(msg, descriptor.message_type)
            return self._encode_legacy(msg, descriptor.message_type)

        # A bare string/pattern topic carries no declaration: follow the value.
        if _uses_cdr(msg):
            return self._encode_generated(msg, Image)
        return cast("bytes", msg.lcm_jpeg_encode())

    def decode(self, msg: bytes, topic: Topic) -> Any:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        descriptor = topic.message
        if descriptor is None:
            raise DecodingError(
                f"Cannot decode JPEG: topic {topic.topic!r} has no declared message type"
            )
        if descriptor.encoding == "cdr":
            if descriptor.message_type is not Image:
                raise DecodingError("JPEG transport requires generated sensor_msgs/Image")
            return image.from_compressed(CDRCodec(CompressedImage).decode(msg))
        return cast("type[Any]", descriptor.message_type).lcm_jpeg_decode(msg)

    @staticmethod
    def _encode_generated(msg: Any, message_type: type) -> bytes:
        if message_type is not Image or type(msg) is not Image:
            raise TypeError("JPEG transport requires a generated sensor_msgs/Image")
        return CDRCodec(CompressedImage).encode(image.to_compressed(msg, format="jpeg"))

    @staticmethod
    def _encode_legacy(msg: Any, message_type: type) -> bytes:
        if type(msg) is not message_type or not hasattr(msg, "lcm_jpeg_encode"):
            raise TypeError("JPEG transport requires the declared dimos Image class")
        return cast("bytes", msg.lcm_jpeg_encode())


def _uses_cdr(msg: Any) -> bool:
    from dimos.protocol.pubsub.codec import describe_message

    try:
        return describe_message(type(msg)).encoding == "cdr"
    except TypeError:
        return False


class JpegLCM(  # type: ignore[misc]
    JpegEncoderMixin,
    LCMPubSubBase,
): ...
