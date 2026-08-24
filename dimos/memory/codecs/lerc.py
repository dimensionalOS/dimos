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

import imagecodecs
import numpy as np

from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

if TYPE_CHECKING:
    from numpy.typing import NDArray


MAX_ERROR_METERS = 0.005
_MILLIMETERS_PER_METER = 1000.0
_WIRE_FORMAT = "lerc"


class LercCodec:
    """Bounded-error storage codec for depth images.

    Float depth is measured in meters and uint16 depth in millimeters. Every
    decoded valid sample differs from its input by at most 5 mm. Invalid float
    samples decode as NaN; invalid uint16 samples decode as zero.
    """

    def encode(self, value: Image) -> bytes:
        level, valid = _encoding_settings(value)
        pixels = np.ascontiguousarray(np.where(valid, value.data, 0))
        blob = bytes(imagecodecs.lerc_encode(pixels, level=level, masks=valid))
        return CompressedImage(
            data=blob,
            format=_WIRE_FORMAT,
            frame_id=value.frame_id,
            ts=value.ts,
        ).lcm_encode()

    def decode(self, data: bytes) -> Image:
        compressed = CompressedImage.lcm_decode(data)
        if compressed.format != _WIRE_FORMAT:
            raise ValueError(
                f"LercCodec expected {_WIRE_FORMAT!r} payload, got {compressed.format!r}"
            )

        pixels, valid = imagecodecs.lerc_decode(compressed.data, masks=True)
        if valid is None:
            valid = np.ones(pixels.shape, dtype=np.bool_)
        if pixels.dtype == np.float32:
            pixels = np.asarray(pixels)
            pixels[~valid] = np.nan
            pixels[valid & (pixels <= 0)] = np.nextafter(np.float32(0), np.float32(1))
            image_format = ImageFormat.DEPTH
        elif pixels.dtype == np.uint16:
            pixels = np.asarray(pixels)
            pixels[~valid] = 0
            pixels[valid & (pixels == 0)] = 1
            image_format = ImageFormat.DEPTH16
        else:
            raise ValueError(f"LercCodec decoded unsupported dtype {pixels.dtype}")

        return Image(
            data=pixels,
            format=image_format,
            frame_id=compressed.frame_id,
            ts=compressed.ts,
        )


def _encoding_settings(value: Image) -> tuple[float, NDArray[np.bool_]]:
    pixels = value.data
    if value.format is ImageFormat.DEPTH and pixels.dtype == np.float32:
        return MAX_ERROR_METERS, np.isfinite(pixels) & (pixels > 0)
    if value.format is ImageFormat.DEPTH16 and pixels.dtype == np.uint16:
        return MAX_ERROR_METERS * _MILLIMETERS_PER_METER, pixels > 0
    raise ValueError(
        "LercCodec supports only DEPTH/float32 meters and DEPTH16/uint16 millimeters; "
        f"got {value.format.value}/{pixels.dtype}"
    )
