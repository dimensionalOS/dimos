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

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

DEPTH_FORMATS = (ImageFormat.DEPTH, ImageFormat.DEPTH16)


class JpegCodec:
    """Codec for Image types — JPEG-compressed inside an LCM Image envelope.

    Uses ``Image.lcm_jpeg_encode/decode`` which preserves ``ts``, ``frame_id``,
    and all LCM header fields. Pixel data is lossy-compressed via TurboJPEG.

    Depth frames are stored uncompressed instead. JPEG is 8-bit colour: a float32
    metre map cannot be encoded at all, and a uint16 one would be rescaled to 8
    bits and then lossy-compressed, quietly destroying the metric values a cloud
    is unprojected from. The LCM envelope carries its own encoding, so both kinds
    decode through the same path.
    """

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality

    @property
    def payload_type(self) -> type[Image]:
        """Message type decoded by this storage codec."""
        return Image

    def encode(self, value: Image) -> bytes:
        if value.format in DEPTH_FORMATS:
            return value.lcm_encode()
        return value.lcm_jpeg_encode(quality=self._quality)

    def decode(self, data: bytes) -> Image:
        return Image.lcm_decode(data)
