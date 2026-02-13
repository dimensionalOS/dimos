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

from __future__ import annotations

import base64
from dataclasses import dataclass, field
from enum import Enum
import time
from typing import TYPE_CHECKING, Any, Literal, TypedDict

import cv2
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.std_msgs.Header import Header
import numpy as np
import reactivex as rx
from reactivex import operators as ops
import rerun as rr
from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

from dimos.types.timestamped import Timestamped, TimestampedBufferCollection, to_human_readable
from dimos.utils.reactive import quality_barrier

if TYPE_CHECKING:
    from collections.abc import Callable
    import os

    from reactivex.observable import Observable


class ImageFormat(Enum):
    BGR = "BGR"
    RGB = "RGB"
    RGBA = "RGBA"
    BGRA = "BGRA"
    GRAY = "GRAY"
    GRAY16 = "GRAY16"
    DEPTH = "DEPTH"
    DEPTH16 = "DEPTH16"


def _format_to_rerun(data: np.ndarray, fmt: ImageFormat) -> Any:  # type: ignore[type-arg]
    """Convert image data to Rerun archetype based on format."""
    match fmt:
        case ImageFormat.RGB:
            return rr.Image(data, color_model="RGB")
        case ImageFormat.RGBA:
            return rr.Image(data, color_model="RGBA")
        case ImageFormat.BGR:
            return rr.Image(data, color_model="BGR")
        case ImageFormat.BGRA:
            return rr.Image(data, color_model="BGRA")
        case ImageFormat.GRAY:
            return rr.Image(data, color_model="L")
        case ImageFormat.GRAY16:
            return rr.Image(data, color_model="L")
        case ImageFormat.DEPTH:
            return rr.DepthImage(data)
        case ImageFormat.DEPTH16:
            return rr.DepthImage(data)
        case _:
            raise ValueError(f"Unsupported format for Rerun: {fmt}")


class AgentImageMessage(TypedDict):
    """Type definition for agent-compatible image representation."""

    type: Literal["image"]
    source_type: Literal["base64"]
    mime_type: Literal["image/jpeg", "image/png"]
    data: str  # Base64 encoded image data


@dataclass
class Image(Timestamped):
    """Simple NumPy-based image container."""

    msg_name = "sensor_msgs.Image"

    data: np.ndarray[Any, np.dtype[Any]] = field(
        default_factory=lambda: np.zeros((1, 1, 3), dtype=np.uint8)
    )
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        if not isinstance(self.data, np.ndarray):
            self.data = np.asarray(self.data)
        if self.data.ndim < 2:
            raise ValueError("Image requires a 2D/3D NumPy array")

    def __str__(self) -> str:
        return (
            f"Image(shape={self.shape}, format={self.format.value}, dtype={self.dtype}, "
            f"ts={to_human_readable(self.ts)})"
        )

    def __repr__(self) -> str:
        return f"Image(shape={self.shape}, format={self.format.value}, dtype={self.dtype}, frame_id='{self.frame_id}', ts={self.ts})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Image):
            return False
        return (
            np.array_equal(self.data, other.data)
            and self.format == other.format
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __len__(self) -> int:
        return int(self.height * self.width)

    def __getstate__(self) -> dict[str, Any]:
        return {"data": self.data, "format": self.format, "frame_id": self.frame_id, "ts": self.ts}

    def __setstate__(self, state: dict[str, Any]) -> None:
        self.data = state.get("data", np.zeros((1, 1, 3), dtype=np.uint8))
        self.format = state.get("format", ImageFormat.BGR)
        self.frame_id = state.get("frame_id", "")
        self.ts = state.get("ts", time.time())

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def channels(self) -> int:
        if self.data.ndim == 2:
            return 1
        if self.data.ndim == 3:
            return int(self.data.shape[2])
        raise ValueError("Invalid image dimensions")

    @property
    def shape(self) -> tuple[int, ...]:
        return tuple(self.data.shape)

    @property
    def dtype(self) -> np.dtype[Any]:
        return self.data.dtype

    def copy(self) -> Image:
        return Image(data=self.data.copy(), format=self.format, frame_id=self.frame_id, ts=self.ts)

    @classmethod
    def from_numpy(
        cls,
        np_image: np.ndarray,  # type: ignore[type-arg]
        format: ImageFormat = ImageFormat.BGR,
        frame_id: str = "",
        ts: float | None = None,
    ) -> Image:
        return cls(
            data=np.asarray(np_image),
            format=format,
            frame_id=frame_id,
            ts=ts if ts is not None else time.time(),
        )

    @classmethod
    def from_file(
        cls,
        filepath: str | os.PathLike[str],
        format: ImageFormat = ImageFormat.RGB,
    ) -> Image:
        arr = cv2.imread(str(filepath), cv2.IMREAD_UNCHANGED)
        if arr is None:
            raise ValueError(f"Could not load image from {filepath}")
        if arr.ndim == 2:
            detected = ImageFormat.GRAY16 if arr.dtype == np.uint16 else ImageFormat.GRAY
        elif arr.shape[2] == 3:
            detected = ImageFormat.BGR  # OpenCV default
        elif arr.shape[2] == 4:
            detected = ImageFormat.BGRA  # OpenCV default
        else:
            detected = format
        return cls(data=arr, format=detected)

    @classmethod
    def from_opencv(
        cls,
        cv_image: np.ndarray,  # type: ignore[type-arg]
        format: ImageFormat = ImageFormat.BGR,
        frame_id: str = "",
        ts: float | None = None,
    ) -> Image:
        """Construct from an OpenCV image (NumPy array)."""
        return cls(
            data=cv_image,
            format=format,
            frame_id=frame_id,
            ts=ts if ts is not None else time.time(),
        )

    def to_opencv(self) -> np.ndarray:  # type: ignore[type-arg]
        """Convert to OpenCV BGR format."""
        arr = self.data
        if self.format == ImageFormat.BGR:
            return arr
        if self.format == ImageFormat.RGB:
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if self.format == ImageFormat.RGBA:
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if self.format == ImageFormat.BGRA:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if self.format in (
            ImageFormat.GRAY,
            ImageFormat.GRAY16,
            ImageFormat.DEPTH,
            ImageFormat.DEPTH16,
        ):
            return arr
        raise ValueError(f"Unsupported format: {self.format}")

    def as_numpy(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get image data as numpy array."""
        return self.data

    def to_rgb(self) -> Image:
        if self.format == ImageFormat.RGB:
            return self.copy()
        arr = self.data
        if self.format == ImageFormat.BGR:
            return Image(
                data=cv2.cvtColor(arr, cv2.COLOR_BGR2RGB),
                format=ImageFormat.RGB,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format == ImageFormat.RGBA:
            return self.copy()  # RGBA contains RGB + alpha
        if self.format == ImageFormat.BGRA:
            rgba = cv2.cvtColor(arr, cv2.COLOR_BGRA2RGBA)
            return Image(data=rgba, format=ImageFormat.RGBA, frame_id=self.frame_id, ts=self.ts)
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (arr / 256).astype(np.uint8) if self.format != ImageFormat.GRAY else arr
            rgb = cv2.cvtColor(gray8, cv2.COLOR_GRAY2RGB)
            return Image(data=rgb, format=ImageFormat.RGB, frame_id=self.frame_id, ts=self.ts)
        return self.copy()

    def to_bgr(self) -> Image:
        if self.format == ImageFormat.BGR:
            return self.copy()
        arr = self.data
        if self.format == ImageFormat.RGB:
            return Image(
                data=cv2.cvtColor(arr, cv2.COLOR_RGB2BGR),
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format == ImageFormat.RGBA:
            return Image(
                data=cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR),
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format == ImageFormat.BGRA:
            return Image(
                data=cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR),
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH16):
            gray8 = (arr / 256).astype(np.uint8) if self.format != ImageFormat.GRAY else arr
            return Image(
                data=cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR),
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        return self.copy()

    def to_grayscale(self) -> Image:
        if self.format in (ImageFormat.GRAY, ImageFormat.GRAY16, ImageFormat.DEPTH):
            return self.copy()
        if self.format == ImageFormat.BGR:
            return Image(
                data=cv2.cvtColor(self.data, cv2.COLOR_BGR2GRAY),
                format=ImageFormat.GRAY,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format == ImageFormat.RGB:
            return Image(
                data=cv2.cvtColor(self.data, cv2.COLOR_RGB2GRAY),
                format=ImageFormat.GRAY,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        if self.format in (ImageFormat.RGBA, ImageFormat.BGRA):
            code = cv2.COLOR_RGBA2GRAY if self.format == ImageFormat.RGBA else cv2.COLOR_BGRA2GRAY
            return Image(
                data=cv2.cvtColor(self.data, code),
                format=ImageFormat.GRAY,
                frame_id=self.frame_id,
                ts=self.ts,
            )
        raise ValueError(f"Unsupported format: {self.format}")

    def to_rerun(self) -> Any:
        """Convert to rerun Image format."""
        return _format_to_rerun(self.data, self.format)

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> Image:
        return Image(
            data=cv2.resize(self.data, (width, height), interpolation=interpolation),
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def resize_to_fit(
        self, max_width: int, max_height: int, interpolation: int = cv2.INTER_LINEAR
    ) -> tuple[Image, float]:
        """Resize image to fit within max dimensions while preserving aspect ratio.

        Only scales down if image exceeds max dimensions. Returns self if already fits.

        Returns:
            Tuple of (resized_image, scale_factor). Scale factor is 1.0 if no resize needed.
        """
        if self.width <= max_width and self.height <= max_height:
            return self, 1.0

        scale = min(max_width / self.width, max_height / self.height)
        new_width = int(self.width * scale)
        new_height = int(self.height * scale)
        return self.resize(new_width, new_height, interpolation), scale

    def crop(self, x: int, y: int, width: int, height: int) -> Image:
        """Crop the image to the specified region.

        Args:
            x: Starting x coordinate (left edge)
            y: Starting y coordinate (top edge)
            width: Width of the cropped region
            height: Height of the cropped region

        Returns:
            A new Image containing the cropped region
        """
        img_height, img_width = self.data.shape[:2]

        # Clamp the crop region to image bounds
        x = max(0, min(x, img_width))
        y = max(0, min(y, img_height))
        x_end = min(x + width, img_width)
        y_end = min(y + height, img_height)

        # Perform the crop using array slicing
        if self.data.ndim == 2:
            cropped_data = self.data[y:y_end, x:x_end]
        else:
            cropped_data = self.data[y:y_end, x:x_end, :]

        return Image(data=cropped_data, format=self.format, frame_id=self.frame_id, ts=self.ts)

    @property
    def sharpness(self) -> float:
        """Return sharpness score."""
        gray = self.to_grayscale()
        sx = cv2.Sobel(gray.data, cv2.CV_32F, 1, 0, ksize=5)
        sy = cv2.Sobel(gray.data, cv2.CV_32F, 0, 1, ksize=5)
        magnitude = cv2.magnitude(sx, sy)
        mean_mag = float(magnitude.mean())
        if mean_mag <= 0:
            return 0.0
        return float(np.clip((np.log10(mean_mag + 1) - 1.7) / 2.0, 0.0, 1.0))

    def save(self, filepath: str) -> bool:
        arr = self.to_opencv()
        return cv2.imwrite(filepath, arr)

    def to_base64(
        self,
        quality: int = 80,
        *,
        max_width: int | None = None,
        max_height: int | None = None,
    ) -> str:
        """Encode the image as a base64 JPEG string.

        Args:
            quality: JPEG quality (0-100).
            max_width: Optional maximum width to constrain the encoded image.
            max_height: Optional maximum height to constrain the encoded image.

        Returns:
            Base64-encoded JPEG representation of the image.
        """
        bgr_image = self.to_bgr().to_opencv()
        height, width = bgr_image.shape[:2]

        scale = 1.0
        if max_width is not None and width > max_width:
            scale = min(scale, max_width / width)
        if max_height is not None and height > max_height:
            scale = min(scale, max_height / height)

        if scale < 1.0:
            new_width = max(1, round(width * scale))
            new_height = max(1, round(height * scale))
            bgr_image = cv2.resize(bgr_image, (new_width, new_height), interpolation=cv2.INTER_AREA)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(quality, 0, 100))]
        success, buffer = cv2.imencode(".jpg", bgr_image, encode_param)
        if not success:
            raise ValueError("Failed to encode image as JPEG")

        return base64.b64encode(buffer.tobytes()).decode("utf-8")

    def agent_encode(self) -> AgentImageMessage:
        return [  # type: ignore[return-value]
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{self.to_base64()}"},
            }
        ]

    # LCM encode/decode
    def lcm_encode(self, frame_id: str | None = None) -> bytes:
        """Convert to LCM Image message."""
        msg = LCMImage()

        # Header
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id

        # Set timestamp
        if self.ts is not None:
            msg.header.stamp.sec = int(self.ts)
            msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        else:
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nsec = int((now - int(now)) * 1e9)

        # Image properties
        msg.height = self.height
        msg.width = self.width
        msg.encoding = _get_lcm_encoding(self.format, self.dtype)
        msg.is_bigendian = False

        # Calculate step (bytes per row)
        channels = 1 if self.data.ndim == 2 else self.data.shape[2]
        msg.step = self.width * self.dtype.itemsize * channels

        view = memoryview(np.ascontiguousarray(self.data)).cast("B")  # type: ignore[arg-type]
        msg.data_length = len(view)
        msg.data = view

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> Image:
        msg = LCMImage.lcm_decode(data)
        fmt, dtype, channels = _parse_lcm_encoding(msg.encoding)
        arr: np.ndarray[Any, Any] = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            arr = arr.reshape((msg.height, msg.width))
        else:
            arr = arr.reshape((msg.height, msg.width, channels))
        return cls(
            data=arr,
            format=fmt,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=(
                msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header")
                and hasattr(msg.header, "stamp")
                and msg.header.stamp.sec > 0
                else time.time()
            ),
        )

    def lcm_jpeg_encode(self, quality: int = 75, frame_id: str | None = None) -> bytes:
        """Convert to LCM Image message with JPEG-compressed data.

        Args:
            quality: JPEG compression quality (0-100, default 75)
            frame_id: Optional frame ID override

        Returns:
            LCM-encoded bytes with JPEG-compressed image data
        """
        jpeg = TurboJPEG()
        msg = LCMImage()

        # Header
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id

        # Set timestamp
        if self.ts is not None:
            msg.header.stamp.sec = int(self.ts)
            msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        else:
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nsec = int((now - int(now)) * 1e9)

        # Get image in BGR format for JPEG encoding
        bgr_image = self.to_bgr().to_opencv()

        # Encode as JPEG
        jpeg_data = jpeg.encode(bgr_image, quality=quality)

        # Store JPEG data and metadata
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "jpeg"
        msg.is_bigendian = False
        msg.step = 0  # Not applicable for compressed format

        msg.data_length = len(jpeg_data)
        msg.data = jpeg_data

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_jpeg_decode(cls, data: bytes, **kwargs: Any) -> Image:
        """Decode an LCM Image message with JPEG-compressed data.

        Args:
            data: LCM-encoded bytes containing JPEG-compressed image

        Returns:
            Image instance
        """
        jpeg = TurboJPEG()
        msg = LCMImage.lcm_decode(data)

        if msg.encoding != "jpeg":
            raise ValueError(f"Expected JPEG encoding, got {msg.encoding}")

        # Decode JPEG data
        bgr_array = jpeg.decode(msg.data)

        return cls(
            data=bgr_array,
            format=ImageFormat.BGR,
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=(
                msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header")
                and hasattr(msg.header, "stamp")
                and msg.header.stamp.sec > 0
                else time.time()
            ),
        )


__all__ = [
    "Image",
    "ImageFormat",
    "sharpness_barrier",
    "sharpness_window",
]


def sharpness_window(target_frequency: float, source: Observable[Image]) -> Observable[Image]:
    """Emit the sharpest Image seen within each sliding time window."""
    from reactivex.scheduler import ThreadPoolScheduler

    if target_frequency <= 0:
        raise ValueError("target_frequency must be positive")

    window = TimestampedBufferCollection(1.0 / target_frequency)  # type: ignore[var-annotated]
    source.subscribe(window.add)

    thread_scheduler = ThreadPoolScheduler(max_workers=1)

    def find_best(*_args: Any) -> Image | None:
        if len(window) == 0:
            return None
        return max(window, key=lambda img: img.sharpness)  # type: ignore[no-any-return]

    return rx.interval(1.0 / target_frequency).pipe(  # type: ignore[misc]
        ops.observe_on(thread_scheduler),
        ops.map(find_best),
        ops.filter(lambda img: img is not None),
    )


def sharpness_barrier(target_frequency: float) -> Callable[[Observable[Image]], Observable[Image]]:
    """Select the sharpest Image within each time window."""
    if target_frequency <= 0:
        raise ValueError("target_frequency must be positive")
    return quality_barrier(lambda image: image.sharpness, target_frequency)  # type: ignore[attr-defined]


def _get_lcm_encoding(fmt: ImageFormat, dtype: np.dtype) -> str:  # type: ignore[type-arg]
    if fmt == ImageFormat.GRAY:
        if dtype == np.uint8:
            return "mono8"
        if dtype == np.uint16:
            return "mono16"
    if fmt == ImageFormat.GRAY16:
        return "mono16"
    if fmt == ImageFormat.RGB:
        return "rgb8"
    if fmt == ImageFormat.RGBA:
        return "rgba8"
    if fmt == ImageFormat.BGR:
        return "bgr8"
    if fmt == ImageFormat.BGRA:
        return "bgra8"
    if fmt == ImageFormat.DEPTH:
        if dtype == np.float32:
            return "32FC1"
        if dtype == np.float64:
            return "64FC1"
    if fmt == ImageFormat.DEPTH16:
        if dtype == np.uint16:
            return "16UC1"
        if dtype == np.int16:
            return "16SC1"
    raise ValueError(f"Unsupported LCM encoding for fmt={fmt}, dtype={dtype}")


def _parse_lcm_encoding(enc: str) -> tuple[ImageFormat, type, int]:
    m = {
        "mono8": (ImageFormat.GRAY, np.uint8, 1),
        "mono16": (ImageFormat.GRAY16, np.uint16, 1),
        "rgb8": (ImageFormat.RGB, np.uint8, 3),
        "rgba8": (ImageFormat.RGBA, np.uint8, 4),
        "bgr8": (ImageFormat.BGR, np.uint8, 3),
        "bgra8": (ImageFormat.BGRA, np.uint8, 4),
        "32FC1": (ImageFormat.DEPTH, np.float32, 1),
        "32FC3": (ImageFormat.RGB, np.float32, 3),
        "64FC1": (ImageFormat.DEPTH, np.float64, 1),
        "16UC1": (ImageFormat.DEPTH16, np.uint16, 1),
        "16SC1": (ImageFormat.DEPTH16, np.int16, 1),
    }
    if enc not in m:
        raise ValueError(f"Unsupported encoding: {enc}")
    return m[enc]
