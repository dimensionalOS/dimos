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
        from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
        from dimos.msgs.sensor_msgs.Image import ImageFormat

        if value.format in (ImageFormat.DEPTH, ImageFormat.DEPTH16):
            compressed = CompressedImage.from_image(value, format="jxl", effort=1)
            compressed.format = f"jxl;{value.format.value.lower()}"
        else:
            compressed = CompressedImage.from_image(
                value,
                format="jpeg",
                quality=self._quality,
            )
        return compressed.lcm_encode()

    def decode(self, data: bytes) -> Image:
        from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        compressed = CompressedImage.lcm_decode(data)
        image = compressed.decode()
        depth_formats = {
            "jxl;depth": ImageFormat.DEPTH,
            "jxl;depth16": ImageFormat.DEPTH16,
        }
        image_format = depth_formats.get(compressed.format)
        if image_format is None:
            return image
        return Image(
            data=image.data,
            format=image_format,
            frame_id=image.frame_id,
            ts=image.ts,
        )
