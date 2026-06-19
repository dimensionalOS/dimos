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
    """Codec for Image types — JPEG-compressed inside an LCM Image envelope.

    Uses ``Image.lcm_jpeg_encode/decode`` which preserves ``ts``, ``frame_id``,
    and all LCM header fields. Pixel data is lossy-compressed via TurboJPEG.
    """

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality

    def encode(self, value: Image) -> bytes:
        return value.lcm_jpeg_encode(quality=self._quality)

    def decode(self, data: bytes) -> Image:
        from dimos.msgs.sensor_msgs.Image import Image

        # Some recordings include a 1-byte format tag before the LCM envelope
        # (b'J' for JPEG-encoded Image). Strip it on read so old + new sessions
        # both decode cleanly.
        if data and data[0:1] == b"J":
            data = data[1:]
        return Image.lcm_jpeg_decode(data)
