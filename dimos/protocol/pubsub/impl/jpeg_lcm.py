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

"""JPEG-encoded LCM PubSub.

Split from lcmpubsub.py so that importing PickleLCM / LCM does not
transitively pull in ``dimos.msgs.sensor_msgs.Image`` (and its heavy
cv2 / rerun dependencies).
"""

from __future__ import annotations

from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode

from dimos.msgs.image import compressed_image_from_image, image_from_compressed
from dimos.protocol.pubsub.encoders import DecodingError, PubSubEncoderMixin, TypedTopicProto
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase


class JpegEncoderMixin(PubSubEncoderMixin[TypedTopicProto, Image, bytes]):
    """Transcode generated Image values using CompressedImage CDR on the wire."""

    def encode(self, msg: Image, topic: TypedTopicProto) -> bytes:
        if topic.msg_type is not CompressedImage:
            raise ValueError("JPEG wire topics must declare generated CompressedImage")
        return cdr_encode(compressed_image_from_image(msg))

    def decode(self, msg: bytes, topic: TypedTopicProto) -> Image:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.msg_type is None:
            raise DecodingError(f"Cannot decode: topic {topic.topic!r} has no msg_type")
        if topic.msg_type is not CompressedImage:
            raise DecodingError("JPEG wire topics must declare generated CompressedImage")
        return image_from_compressed(cdr_decode(msg, CompressedImage))


class JpegLCM(  # type: ignore[misc]
    JpegEncoderMixin,
    LCMPubSubBase,
): ...
