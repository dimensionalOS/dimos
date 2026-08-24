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

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


class JpegCodec:
    """Default storage codec for images.

    Visual images use lossy JPEG. Float32 and uint16 depth images use lossless
    JPEG XL. Both paths preserve timestamps and frame IDs.
    """

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality

    def encode(self, value: Image) -> bytes:
        from dimos.msgs.sensor_msgs.Image import ImageFormat

        if value.format in (ImageFormat.DEPTH, ImageFormat.DEPTH16):
            return value.lcm_jpegxl_encode(effort=1)
        return value.lcm_jpeg_encode(quality=self._quality)

    def decode(self, data: bytes) -> Image:
        from dimos.msgs.sensor_msgs.Image import Image

        return Image.lcm_decode(data)
