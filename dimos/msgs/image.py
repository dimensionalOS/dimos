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

"""Array views and compression for generated ROS image messages."""

from typing import Any

import cv2
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray

_FORMATS = {
    "rgb8": ("u1", 3),
    "bgr8": ("u1", 3),
    "rgba8": ("u1", 4),
    "bgra8": ("u1", 4),
    "mono8": ("u1", 1),
    "mono16": ("u2", 1),
    "16UC1": ("u2", 1),
    "32FC1": ("f4", 1),
}


def image_from_array(pixels: NDArray[Any], *, encoding: str, header: Header | None = None) -> Image:
    """Copy an array into a generated image with an explicit pixel encoding."""
    if encoding not in _FORMATS:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    kind, channels = _FORMATS[encoding]
    expected = np.dtype(kind)
    if pixels.dtype.kind != expected.kind or pixels.dtype.itemsize != expected.itemsize:
        raise ValueError(f"array dtype does not match {encoding}")
    if (channels == 1 and pixels.ndim != 2) or (
        channels != 1 and (pixels.ndim != 3 or pixels.shape[2] != channels)
    ):
        raise ValueError(f"array shape does not match {encoding}")
    contiguous = np.ascontiguousarray(pixels)
    return Image(
        header=header if header is not None else Header(),
        width=pixels.shape[1],
        height=pixels.shape[0],
        encoding=encoding,
        is_bigendian=int(not pixels.dtype.isnative if np.little_endian else pixels.dtype.isnative),
        step=pixels.shape[1] * channels * pixels.dtype.itemsize,
        data=contiguous.view(np.uint8).ravel(),
    )


def image_view(msg: Image) -> NDArray[Any]:
    """Borrow a read-only array, retaining the message and respecting row padding/endian."""
    if msg.encoding not in _FORMATS:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    kind, channels = _FORMATS[msg.encoding]
    dtype = np.dtype((">" if msg.is_bigendian else "<") + kind)
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) != msg.height * msg.step:
        raise ValueError("image data length/step does not match its dimensions")
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (
        (msg.step, dtype.itemsize)
        if channels == 1
        else (msg.step, channels * dtype.itemsize, dtype.itemsize)
    )
    return np.ndarray(shape, dtype=dtype, buffer=msg.data.view(), strides=strides)


def image_to_jpeg(msg: Image, quality: int = 75) -> bytes:
    """Encode an 8-bit gray or color image using OpenCV's JPEG encoder."""
    if isinstance(quality, bool) or not 0 <= quality <= 100:
        raise ValueError("JPEG quality must be in 0..100")
    pixels = image_view(msg)
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
    }
    if msg.encoding in conversions:
        pixels = cv2.cvtColor(pixels, conversions[msg.encoding])
    elif msg.encoding not in ("bgr8", "mono8"):
        raise ValueError(f"JPEG requires 8-bit color or gray, got {msg.encoding!r}")
    ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return bytes(encoded)
