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

from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.encoders import DecodingError
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase


def encode(message: Image, _: object) -> bytes:
    return message.lcm_jpeg_encode()


def decode(message: bytes, topic: object) -> Image:
    lcm_type = topic.lcm_type  # type: ignore[attr-defined]
    if lcm_type is None:
        raise DecodingError
    return lcm_type.lcm_jpeg_decode(message)  # type: ignore[no-any-return]


class JpegLCM(LCMPubSubBase):
    codec = (encode, decode)
