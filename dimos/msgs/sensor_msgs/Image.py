# Copyright 2025 Dimensional Inc.
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
import time
from typing import TYPE_CHECKING, Literal, TypedDict

import cv2
from dimos_lcm.sensor_msgs.Image import Image as LCMImage  # type: ignore[import-untyped]
from dimos_lcm.std_msgs.Header import Header  # type: ignore[import-untyped]
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    HAS_CUDA,
    HAS_NVIMGCODEC,
    NVIMGCODEC_LAST_USED,
    ImageFormat,
)
from dimos.msgs.sensor_msgs.image_impls.CudaImage import CudaImage
from dimos.msgs.sensor_msgs.image_impls.NumpyImage import NumpyImage
from dimos.types.timestamped import Timestamped, TimestampedBufferCollection, to_human_readable
from dimos.utils.reactive import quality_barrier

if TYPE_CHECKING:
    import os

    from reactivex.observable import Observable

    from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
        AbstractImage,
    )

try:
    import cupy as cp  # type: ignore
except Exception:
    cp = None

try:
    from sensor_msgs.msg import Image as ROSImage  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSImage = None  # type: ignore[assignment, misc]


class AgentImageMessage(TypedDict):
    """Type definition for agent-compatible image representation."""

    type: Literal["image"]
    source_type: Literal["base64"]
    mime_type: Literal["image/jpeg", "image/png"]
    data: str  # Base64 encoded image data


class Image(Timestamped):
    msg_name = "sensor_msgs.Image"

    def __init__(  # type: ignore[no-untyped-def]
        self,
        impl: AbstractImage | None = None,
        *,
        data=None,
        format: ImageFormat | None = None,
        frame_id: str | None = None,
        ts: float | None = None,
    ) -> None:
        """Construct an Image facade.

        Usage:
        - Image(impl=<AbstractImage>)
        - Image(data=<ndarray | cupy.ndarray>, format=ImageFormat.RGB, frame_id=str, ts=float)

        Notes:
        - When constructed from `data`, uses CudaImage if `data` is a CuPy array and CUDA is available; otherwise NumpyImage.
        - `format` defaults to ImageFormat.RGB; `frame_id` defaults to ""; `ts` defaults to `time.time()`.
        """
        # Disallow mixing impl with raw kwargs
        if impl is not None and any(x is not None for x in (data, format, frame_id, ts)):
            raise TypeError(
                "Provide either 'impl' or ('data', 'format', 'frame_id', 'ts'), not both"
            )

        if impl is not None:
            self._impl = impl
            return

        # Raw constructor path
        if data is None:
            raise TypeError("'data' is required when constructing Image without 'impl'")
        fmt = format if format is not None else ImageFormat.BGR
        fid = frame_id if frame_id is not None else ""
        tstamp = ts if ts is not None else time.time()

        # Detect CuPy array without a hard dependency
        is_cu = False
        try:
            import cupy as _cp

            is_cu = isinstance(data, _cp.ndarray)
        except Exception:
            is_cu = False

        if is_cu and HAS_CUDA:
            self._impl = CudaImage(data, fmt, fid, tstamp)
        else:
            self._impl = NumpyImage(np.asarray(data), fmt, fid, tstamp)

    def __str__(self) -> str:
        dev = "cuda" if self.is_cuda else "cpu"
        return (
            f"Image(shape={self.shape}, format={self.format.value}, dtype={self.dtype}, "
            f"dev={dev}, ts={to_human_readable(self.ts)})"
        )

    @classmethod
    def from_impl(cls, impl: AbstractImage) -> Image:
        return cls(impl)

    @classmethod
    def from_numpy(  # type: ignore[no-untyped-def]
        cls,
        np_image: np.ndarray,  # type: ignore[type-arg]
        format: ImageFormat = ImageFormat.BGR,
        to_cuda: bool = False,
        **kwargs,
    ) -> Image:
        if kwargs.pop("to_gpu", False):
            to_cuda = True
        if to_cuda and HAS_CUDA:
            return cls(
                CudaImage(
                    np_image if hasattr(np_image, "shape") else np.asarray(np_image),
                    format,
                    kwargs.get("frame_id", ""),
                    kwargs.get("ts", time.time()),
                )
            )
        return cls(
            NumpyImage(
                np.asarray(np_image),
                format,
                kwargs.get("frame_id", ""),
                kwargs.get("ts", time.time()),
            )
        )

    @classmethod
    def from_file(  # type: ignore[no-untyped-def]
        cls,
        filepath: str | os.PathLike[str],
        format: ImageFormat = ImageFormat.RGB,
        to_cuda: bool = False,
        **kwargs,
    ) -> Image:
        if kwargs.pop("to_gpu", False):
            to_cuda = True
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
        return cls(CudaImage(arr, detected) if to_cuda and HAS_CUDA else NumpyImage(arr, detected))

    @classmethod
    def from_opencv(  # type: ignore[no-untyped-def]
        cls,
        cv_image: np.ndarray,  # type: ignore[type-arg]
        format: ImageFormat = ImageFormat.BGR,
        **kwargs,
    ) -> Image:
        """Construct from an OpenCV image (NumPy array)."""
        return cls(
            NumpyImage(cv_image, format, kwargs.get("frame_id", ""), kwargs.get("ts", time.time()))
        )

    @classmethod
    def from_depth(  # type: ignore[no-untyped-def]
        cls, depth_data, frame_id: str = "", ts: float | None = None, to_cuda: bool = False
    ) -> Image:
        arr = np.asarray(depth_data)
        if arr.dtype != np.float32:
            arr = arr.astype(np.float32)
        impl = (
            CudaImage(arr, ImageFormat.DEPTH, frame_id, time.time() if ts is None else ts)
            if to_cuda and HAS_CUDA
            else NumpyImage(arr, ImageFormat.DEPTH, frame_id, time.time() if ts is None else ts)
        )
        return cls(impl)

    # Delegation
    @property
    def is_cuda(self) -> bool:
        return self._impl.is_cuda

    @property
    def data(self):  # type: ignore[no-untyped-def]
        return self._impl.data

    @data.setter
    def data(self, value) -> None:  # type: ignore[no-untyped-def]
        # Preserve backend semantics: ensure array type matches implementation
        if isinstance(self._impl, NumpyImage):
            self._impl.data = np.asarray(value)
        elif isinstance(self._impl, CudaImage):
            if cp is None:
                raise RuntimeError("CuPy not available to set CUDA image data")
            self._impl.data = cp.asarray(value)
        else:
            self._impl.data = value

    @property
    def format(self) -> ImageFormat:
        return self._impl.format

    @format.setter
    def format(self, value) -> None:  # type: ignore[no-untyped-def]
        if isinstance(value, ImageFormat):
            self._impl.format = value
        elif isinstance(value, str):
            try:
                self._impl.format = ImageFormat[value]
            except KeyError as e:
                raise ValueError(f"Invalid ImageFormat: {value}") from e
        else:
            raise TypeError("format must be ImageFormat or str name")

    @property
    def frame_id(self) -> str:
        return self._impl.frame_id

    @frame_id.setter
    def frame_id(self, value: str) -> None:
        self._impl.frame_id = str(value)

    @property
    def ts(self) -> float:
        return self._impl.ts

    @ts.setter
    def ts(self, value: float) -> None:
        self._impl.ts = float(value)

    @property
    def height(self) -> int:
        return self._impl.height

    @property
    def width(self) -> int:
        return self._impl.width

    @property
    def channels(self) -> int:
        return self._impl.channels

    @property
    def shape(self):  # type: ignore[no-untyped-def]
        return self._impl.shape

    @property
    def dtype(self):  # type: ignore[no-untyped-def]
        return self._impl.dtype

    def copy(self) -> Image:
        return Image(self._impl.copy())

    def to_cpu(self) -> Image:
        if isinstance(self._impl, NumpyImage):
            return self.copy()

        data = self._impl.data.get()  # CuPy array to NumPy

        return Image(
            NumpyImage(
                data,
                self._impl.format,
                self._impl.frame_id,
                self._impl.ts,
            )
        )

    def to_cupy(self) -> Image:
        if isinstance(self._impl, CudaImage):
            return self.copy()
        return Image(
            CudaImage(
                np.asarray(self._impl.data), self._impl.format, self._impl.frame_id, self._impl.ts
            )
        )

    def to_opencv(self) -> np.ndarray:  # type: ignore[type-arg]
        return self._impl.to_opencv()

    def to_rgb(self) -> Image:
        return Image(self._impl.to_rgb())

    def to_bgr(self) -> Image:
        return Image(self._impl.to_bgr())

    def to_grayscale(self) -> Image:
        return Image(self._impl.to_grayscale())

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> Image:
        return Image(self._impl.resize(width, height, interpolation))

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
        return Image(self._impl.crop(x, y, width, height))  # type: ignore[attr-defined]

    @property
    def sharpness(self) -> float:
        """Return sharpness score."""
        return self._impl.sharpness()

    def save(self, filepath: str) -> bool:
        return self._impl.save(filepath)

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

        # Image data - use raw data to preserve format
        image_bytes = self.data.tobytes()
        msg.data_length = len(image_bytes)
        msg.data = image_bytes

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs) -> Image:  # type: ignore[no-untyped-def]
        msg = LCMImage.lcm_decode(data)
        fmt, dtype, channels = _parse_lcm_encoding(msg.encoding)
        arr = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            arr = arr.reshape((msg.height, msg.width))
        else:
            arr = arr.reshape((msg.height, msg.width, channels))
        return cls(
            NumpyImage(
                arr,
                fmt,
                msg.header.frame_id if hasattr(msg, "header") else "",
                (
                    msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                    if hasattr(msg, "header")
                    and hasattr(msg.header, "stamp")
                    and msg.header.stamp.sec > 0
                    else time.time()
                ),
            )
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
    def lcm_jpeg_decode(cls, data: bytes, **kwargs) -> Image:  # type: ignore[no-untyped-def]
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
            NumpyImage(
                bgr_array,
                ImageFormat.BGR,
                msg.header.frame_id if hasattr(msg, "header") else "",
                (
                    msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                    if hasattr(msg, "header")
                    and hasattr(msg.header, "stamp")
                    and msg.header.stamp.sec > 0
                    else time.time()
                ),
            )
        )

    # PnP wrappers
    def solve_pnp(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return self._impl.solve_pnp(*args, **kwargs)  # type: ignore

    def solve_pnp_ransac(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return self._impl.solve_pnp_ransac(*args, **kwargs)  # type: ignore

    def solve_pnp_batch(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return self._impl.solve_pnp_batch(*args, **kwargs)  # type: ignore

    def create_csrt_tracker(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return self._impl.create_csrt_tracker(*args, **kwargs)  # type: ignore

    def csrt_update(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        return self._impl.csrt_update(*args, **kwargs)  # type: ignore

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSImage) -> Image:
        """Create an Image from a ROS sensor_msgs/Image message.

        Args:
            ros_msg: ROS Image message

        Returns:
            Image instance
        """
        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Parse encoding to determine format and data type
        format_info = cls._parse_encoding(ros_msg.encoding)

        # Convert data from ROS message (array.array) to numpy array
        data_array = np.frombuffer(ros_msg.data, dtype=format_info["dtype"])

        # Reshape to image dimensions
        if format_info["channels"] == 1:
            data_array = data_array.reshape((ros_msg.height, ros_msg.width))
        else:
            data_array = data_array.reshape(
                (ros_msg.height, ros_msg.width, format_info["channels"])
            )

        # Crop to center 1/3 of the image (simulate 120-degree FOV from 360-degree)
        original_width = data_array.shape[1]
        crop_width = original_width // 3
        start_x = (original_width - crop_width) // 2
        end_x = start_x + crop_width

        # Crop the image horizontally to center 1/3
        if len(data_array.shape) == 2:
            # Grayscale image
            data_array = data_array[:, start_x:end_x]
        else:
            # Color image
            data_array = data_array[:, start_x:end_x, :]

        # Fix color channel order: if ROS sends RGB but we expect BGR, swap channels
        # ROS typically uses rgb8 encoding, but OpenCV/our system expects BGR
        if format_info["format"] == ImageFormat.RGB:
            # Convert RGB to BGR by swapping channels
            if len(data_array.shape) == 3 and data_array.shape[2] == 3:
                data_array = data_array[:, :, [2, 1, 0]]  # RGB -> BGR
                format_info["format"] = ImageFormat.BGR
        elif format_info["format"] == ImageFormat.RGBA:
            # Convert RGBA to BGRA by swapping channels
            if len(data_array.shape) == 3 and data_array.shape[2] == 4:
                data_array = data_array[:, :, [2, 1, 0, 3]]  # RGBA -> BGRA
                format_info["format"] = ImageFormat.BGRA

        return cls(
            data=data_array,
            format=format_info["format"],
            frame_id=ros_msg.header.frame_id,
            ts=ts,
        )

    @staticmethod
    def _parse_encoding(encoding: str) -> dict:  # type: ignore[type-arg]
        """Translate ROS encoding strings into format metadata."""
        encoding_map = {
            "mono8": {"format": ImageFormat.GRAY, "dtype": np.uint8, "channels": 1},
            "mono16": {"format": ImageFormat.GRAY16, "dtype": np.uint16, "channels": 1},
            "rgb8": {"format": ImageFormat.RGB, "dtype": np.uint8, "channels": 3},
            "rgba8": {"format": ImageFormat.RGBA, "dtype": np.uint8, "channels": 4},
            "bgr8": {"format": ImageFormat.BGR, "dtype": np.uint8, "channels": 3},
            "bgra8": {"format": ImageFormat.BGRA, "dtype": np.uint8, "channels": 4},
            "32FC1": {"format": ImageFormat.DEPTH, "dtype": np.float32, "channels": 1},
            "32FC3": {"format": ImageFormat.RGB, "dtype": np.float32, "channels": 3},
            "64FC1": {"format": ImageFormat.DEPTH, "dtype": np.float64, "channels": 1},
            "16UC1": {"format": ImageFormat.DEPTH16, "dtype": np.uint16, "channels": 1},
            "16SC1": {"format": ImageFormat.DEPTH16, "dtype": np.int16, "channels": 1},
        }

        key = encoding.strip()
        for candidate in (key, key.lower(), key.upper()):
            if candidate in encoding_map:
                return dict(encoding_map[candidate])

        raise ValueError(f"Unsupported encoding: {encoding}")

    def __repr__(self) -> str:
        dev = "cuda" if self.is_cuda else "cpu"
        return f"Image(shape={self.shape}, format={self.format.value}, dtype={self.dtype}, dev={dev}, frame_id='{self.frame_id}', ts={self.ts})"

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
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

    def __getstate__(self):  # type: ignore[no-untyped-def]
        return {"data": self.data, "format": self.format, "frame_id": self.frame_id, "ts": self.ts}

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self.__init__(  # type: ignore[misc]
            data=state.get("data"),
            format=state.get("format"),
            frame_id=state.get("frame_id"),
            ts=state.get("ts"),
        )


# Re-exports for tests
HAS_CUDA = HAS_CUDA
ImageFormat = ImageFormat
NVIMGCODEC_LAST_USED = NVIMGCODEC_LAST_USED
HAS_NVIMGCODEC = HAS_NVIMGCODEC
__all__ = [
    "HAS_CUDA",
    "HAS_NVIMGCODEC",
    "NVIMGCODEC_LAST_USED",
    "ImageFormat",
    "sharpness_barrier",
    "sharpness_window",
]


def sharpness_window(target_frequency: float, source: Observable[Image]) -> Observable[Image]:
    """Emit the sharpest Image seen within each sliding time window."""
    if target_frequency <= 0:
        raise ValueError("target_frequency must be positive")

    window = TimestampedBufferCollection(1.0 / target_frequency)  # type: ignore[var-annotated]
    source.subscribe(window.add)

    thread_scheduler = ThreadPoolScheduler(max_workers=1)  # type: ignore[name-defined]

    def find_best(*_args):  # type: ignore[no-untyped-def]
        if not window._items:
            return None
        return max(window._items, key=lambda img: img.sharpness)

    return rx.interval(1.0 / target_frequency).pipe(
        ops.observe_on(thread_scheduler),
        ops.map(find_best),
        ops.filter(lambda img: img is not None),
    )


def sharpness_barrier(target_frequency: float):  # type: ignore[no-untyped-def]
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


def _parse_lcm_encoding(enc: str):  # type: ignore[no-untyped-def]
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
