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

from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.encoders import PubSubEncoderMixin
from dimos.protocol.pubsub.impl.shmpubsub import SharedMemoryPubSubBase


class JpegSharedMemoryEncoderMixin(PubSubEncoderMixin[str, Image, bytes]):
    def __init__(self, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self.jpeg = TurboJPEG()
        self.quality = quality

    def encode(self, msg: Any, _topic: str) -> bytes:
        if not isinstance(msg, Image):
            raise ValueError("Can only encode images.")

        bgr_image = msg.to_bgr().to_opencv()
        return self.jpeg.encode(bgr_image, quality=self.quality)  # type: ignore[no-any-return]

    def decode(self, msg: bytes, _topic: str) -> Image:
        bgr_array = self.jpeg.decode(msg)
        return Image(data=bgr_array, format=ImageFormat.BGR)


class JpegSharedMemory(JpegSharedMemoryEncoderMixin, SharedMemoryPubSubBase):  # type: ignore[misc]
    pass
