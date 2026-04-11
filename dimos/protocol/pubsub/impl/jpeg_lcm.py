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

from typing import cast

from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.encoders import DecodingError, LCMTopicProto, PubSubEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase


class JpegEncoderMixin(PubSubEncoderMixin[LCMTopicProto, Image, bytes]):
    """Encoder mixin for DimosMsg using JPEG encoding (for images)."""

    def encode(self, msg: Image, _: LCMTopicProto) -> bytes:
        return msg.lcm_jpeg_encode()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> Image:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.lcm_type is None:
            raise DecodingError(f"Cannot decode: topic {topic.topic!r} has no lcm_type")
        return cast("type[Image]", topic.lcm_type).lcm_jpeg_decode(msg)


class JpegLCM(  # type: ignore[misc]
    JpegEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...
