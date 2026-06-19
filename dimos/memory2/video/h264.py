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

from dimos.msgs.sensor_msgs.Image import H264_IMAGE_ENCODING, Image


class H264ImageCodec:
    """memory2 codec for already-H.264 encoded Image payloads.

    This codec deliberately does not decode pixels. It persists an ``Image`` whose
    ``encoding`` is ``"h264"`` and restores the same encoded image on read. A
    separate H.264 decode session turns the encoded stream back into raw Images
    for visualization or module consumption.
    """

    CODEC_ID = "h264"

    def encode(self, value: Image) -> bytes:
        if value.encoding != H264_IMAGE_ENCODING:
            raise ValueError(
                f"H264ImageCodec stores encoded Images; got encoding={value.encoding!r}"
            )
        return value.lcm_encode()

    def decode(self, data: bytes) -> Image:
        image = Image.lcm_decode(data)
        if image.encoding != H264_IMAGE_ENCODING:
            raise ValueError(
                f"H264ImageCodec expected encoded Image; got encoding={image.encoding!r}"
            )
        return image


def is_h264_image(image: Image) -> bool:
    return image.encoding == H264_IMAGE_ENCODING


__all__ = ["H264ImageCodec", "is_h264_image"]
