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

from typing import Any

from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode

from dimos.msgs.image import compressed_image_from_image, image_from_compressed
from dimos.protocol.pubsub.encoders import PubSubEncoderMixin
from dimos.protocol.pubsub.impl.shmpubsub import SharedMemoryPubSubBase


class JpegSharedMemoryEncoderMixin(PubSubEncoderMixin[str, Image, bytes]):
    def __init__(self, quality: int = 75, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.quality = quality

    def encode(self, msg: Any, _topic: str) -> bytes:
        if not isinstance(msg, Image):
            raise ValueError("Can only encode images.")

        return cdr_encode(compressed_image_from_image(msg, quality=self.quality))

    def decode(self, msg: bytes, _topic: str) -> Image:
        return image_from_compressed(cdr_decode(msg, CompressedImage))


class JpegSharedMemory(JpegSharedMemoryEncoderMixin, SharedMemoryPubSubBase):  # type: ignore[misc]
    pass
