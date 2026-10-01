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

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
from PIL import Image as PILImage

_FORMATS = {
    "rgb8": ("u1", 3),
    "bgr8": ("u1", 3),
    "rgba8": ("u1", 4),
    "bgra8": ("u1", 4),
    "mono8": ("u1", 1),
    "mono16": ("u2", 1),
    "16UC1": ("u2", 1),
    "32FC1": ("f4", 1),
    "64FC1": ("f8", 1),
    "16SC1": ("i2", 1),
    "32FC3": ("f4", 3),
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
    import cv2

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


def compressed_image_from_image(
    message: Image,
    quality: int = 75,
    *,
    format: str = "jpeg",
    max_width: int | None = None,
    effort: int | None = None,
) -> CompressedImage:
    """Compress generated pixels, retaining the exact header.

    PNG preserves uint16 depth. JXL preserves uint16/float32 depth losslessly;
    it requires imagecodecs and is an explicit codec extension for consumers.
    """
    if not isinstance(message, Image):
        raise TypeError("compression expects a generated Image")
    if max_width is not None:
        message, _ = image_resize_to_fit(message, max_width, max_width)
    if format == "jpeg":
        output_encoding = "mono8" if message.encoding == "mono8" else "bgr8"
        data = image_to_jpeg(message, quality=quality)
    elif format == "png":
        import cv2

        pixels = image_view(message)
        if pixels.dtype.kind != "u" or pixels.dtype.itemsize not in (1, 2):
            raise ValueError("PNG cannot encode floating-point depth")
        if message.encoding in ("rgb8", "rgba8"):
            code = cv2.COLOR_RGB2BGR if message.encoding == "rgb8" else cv2.COLOR_RGBA2BGRA
            pixels = cv2.cvtColor(pixels, code)
        pixels = np.ascontiguousarray(pixels, dtype=pixels.dtype.newbyteorder("="))
        ok, encoded = cv2.imencode(".png", pixels)
        if not ok:
            raise ValueError("PNG encoding failed")
        data = bytes(encoded)
        output_encoding = {"rgb8": "bgr8", "rgba8": "bgra8"}.get(message.encoding, message.encoding)
    elif format == "jxl":
        import imagecodecs

        pixels = image_view(message)
        if pixels.dtype.kind not in ("u", "f") or (
            pixels.dtype.kind,
            pixels.dtype.itemsize,
        ) not in (("u", 1), ("u", 2), ("f", 4)):
            raise ValueError(f"JXL cannot encode dtype {pixels.dtype}")
        if message.encoding == "bgr8":
            pixels = pixels[..., ::-1]
        elif message.encoding == "bgra8":
            pixels = pixels[..., [2, 1, 0, 3]]
        pixels = np.ascontiguousarray(pixels, dtype=pixels.dtype.newbyteorder("="))
        if pixels.dtype == np.uint8:
            data = bytes(imagecodecs.jpegxl_encode(pixels, level=quality, effort=effort or 3))
        else:
            data = bytes(imagecodecs.jpegxl_encode(pixels, lossless=True, effort=effort or 1))
        output_encoding = {"bgr8": "rgb8", "bgra8": "rgba8"}.get(message.encoding, message.encoding)
    else:
        raise ValueError(f"unsupported compression format {format!r}")
    return CompressedImage(
        header=message.header,
        format=f"{message.encoding}; {format} compressed {output_encoding}",
        data=data,
    )


def image_from_compressed(message: CompressedImage) -> Image:
    """Decode JPEG/PNG or the JXL extension into explicitly encoded generated pixels."""
    import cv2

    format_name = message.format.lower()
    if "jxl" in format_name:
        import imagecodecs

        jxl_pixels = imagecodecs.jpegxl_decode(bytes(message.data))
        if jxl_pixels.ndim == 2:
            encoding = {
                np.dtype("float32"): "32FC1",
                np.dtype("uint16"): "mono16",
                np.dtype("uint8"): "mono8",
            }.get(jxl_pixels.dtype)
        elif jxl_pixels.ndim == 3 and jxl_pixels.shape[2] in (3, 4):
            encoding = "rgb8" if jxl_pixels.shape[2] == 3 else "rgba8"
        else:
            encoding = None
        if encoding is None:
            raise ValueError("unsupported decoded JXL layout")
        if encoding == "mono16" and format_name.startswith("16uc1;"):
            encoding = "16UC1"
        return image_from_array(jxl_pixels, encoding=encoding, header=message.header)
    if not any(name in format_name for name in ("jpeg", "jpg", "png")):
        raise ValueError(f"unsupported compressed image format {message.format!r}")
    pixels = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError("invalid compressed image data")
    if pixels.ndim == 2:
        encoding = "mono16" if pixels.dtype.itemsize == 2 else "mono8"
    elif pixels.ndim == 3 and pixels.shape[2] == 3:
        encoding = "bgr8"
    elif pixels.ndim == 3 and pixels.shape[2] == 4:
        encoding = "bgra8"
    else:
        raise ValueError("unsupported decoded image layout")
    if encoding == "mono16" and format_name.startswith("16uc1;"):
        encoding = "16UC1"
    return image_from_array(pixels, encoding=encoding, header=message.header)


def image_brightness(message: Image) -> float:
    """Sample mean pixel intensity in [0, 1] for 8/16-bit image encodings.

    Respect row padding and endian. Floating-point depth has no normalized
    intensity scale and is rejected rather than treated as a color image.
    """
    pixels = image_view(message)
    if pixels.dtype.kind != "u":
        raise ValueError("brightness requires unsigned integer pixels")
    stride = max(1, max(message.height, message.width) // 256)
    return float(pixels[::stride, ::stride].mean() / np.iinfo(pixels.dtype).max)


def image_sharpness(message: Image) -> float:
    """Laplacian variance of an 8-bit visual image, downsampled to 160 pixels wide."""
    import cv2

    pixels = image_view(message)
    codes = {
        "rgb8": cv2.COLOR_RGB2GRAY,
        "bgr8": cv2.COLOR_BGR2GRAY,
        "rgba8": cv2.COLOR_RGBA2GRAY,
        "bgra8": cv2.COLOR_BGRA2GRAY,
    }
    if message.encoding == "mono8":
        gray = pixels
    elif message.encoding in codes:
        gray = cv2.cvtColor(pixels, codes[message.encoding])
    else:
        raise ValueError("Sharpness requires an 8-bit visual image encoding")
    if gray.size == 0:
        raise ValueError("Sharpness requires a nonempty image")
    if message.width > 160:
        gray = cv2.resize(
            gray,
            (160, max(1, round(message.height * 160 / message.width))),
            interpolation=cv2.INTER_AREA,
        )
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def image_to_bgr(message: Image) -> NDArray[np.uint8]:
    """Return an independent BGR8 array for drawing or OpenCV color operations."""
    import cv2

    pixels = image_view(message)
    if message.encoding == "bgr8":
        return pixels.copy()
    codes = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
    }
    if message.encoding not in codes:
        raise ValueError(f"Cannot convert {message.encoding!r} to BGR8")
    return np.asarray(cv2.cvtColor(pixels, codes[message.encoding]), dtype=np.uint8)


def image_to_rgb(message: Image) -> NDArray[np.uint8]:
    """Copy a display image as RGB8; 16-bit grayscale uses its high byte.

    Alpha is discarded. Floating-point depth needs an explicit visualization
    scale and is rejected rather than assigning pixel values to meters.
    """
    import cv2

    pixels = image_view(message)
    if message.encoding == "rgb8":
        return pixels.copy()
    if message.encoding in ("mono16", "16UC1"):
        return np.asarray(
            cv2.cvtColor((pixels / 256).astype(np.uint8), cv2.COLOR_GRAY2RGB), dtype=np.uint8
        )
    codes = {
        "bgr8": cv2.COLOR_BGR2RGB,
        "rgba8": cv2.COLOR_RGBA2RGB,
        "bgra8": cv2.COLOR_BGRA2RGB,
        "mono8": cv2.COLOR_GRAY2RGB,
    }
    if message.encoding not in codes:
        raise ValueError(f"Cannot convert {message.encoding!r} to RGB8")
    return np.asarray(cv2.cvtColor(pixels, codes[message.encoding]), dtype=np.uint8)


def image_resize_to_fit(message: Image, max_width: int, max_height: int) -> tuple[Image, float]:
    """Downscale a generated image while preserving encoding, aspect and exact header."""
    import cv2

    if min(max_width, max_height, message.width, message.height) <= 0:
        raise ValueError("Image and target dimensions must be positive")
    if message.width <= max_width and message.height <= max_height:
        return message, 1.0
    scale = min(max_width / message.width, max_height / message.height)
    pixels = cv2.resize(
        image_view(message),
        (max(1, int(message.width * scale)), max(1, int(message.height * scale))),
        interpolation=cv2.INTER_LINEAR,
    )
    return image_from_array(pixels, encoding=message.encoding, header=message.header), scale


def image_from_file(path: str | Path, *, header: Header | None = None) -> Image:
    """Decode an image file as generated RGB8 with an explicit optional source header."""
    with PILImage.open(path) as image:
        return image_from_array(np.asarray(image.convert("RGB")), encoding="rgb8", header=header)
