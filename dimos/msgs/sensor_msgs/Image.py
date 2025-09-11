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

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    AbstractImage,
    ImageFormat,
    HAS_CUDA,
    NVIMGCODEC_LAST_USED,
    HAS_NVIMGCODEC,
)
from dimos.msgs.sensor_msgs.image_impls.NumpyImage import NumpyImage
from dimos.msgs.sensor_msgs.image_impls.CudaImage import CudaImage

try:
    import cupy as cp  # type: ignore
except Exception:
    cp = None  # type: ignore
import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler
from dimos.types.timestamped import TimestampedBufferCollection
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.std_msgs.Header import Header


class Image:
    msg_name = "sensor_msgs.Image"

    # Construction
    def __init__(
        self,
        impl: AbstractImage | None = None,
        *,
        data=None,
        format: ImageFormat | None = None,
        frame_id: str | None = None,
        ts: float | None = None,
    ):
        """Construct an Image facade.

        Usage:
        - Image(impl=<AbstractImage>)
        - Image(data=<ndarray | cupy.ndarray>, format=ImageFormat.BGR, frame_id=str, ts=float)

        Notes:
        - When constructed from `data`, uses CudaImage if `data` is a CuPy array and CUDA is available; otherwise NumpyImage.
        - `format` defaults to ImageFormat.BGR; `frame_id` defaults to ""; `ts` defaults to `time.time()`.
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
            import cupy as _cp  # type: ignore

            is_cu = isinstance(data, _cp.ndarray)
        except Exception:
            is_cu = False

        if is_cu and HAS_CUDA:
            self._impl = CudaImage(data, fmt, fid, tstamp)  # type: ignore
        else:
            self._impl = NumpyImage(np.asarray(data), fmt, fid, tstamp)

    @classmethod
    def from_impl(cls, impl: AbstractImage) -> "Image":
        return cls(impl)

    @classmethod
    def from_numpy(
        cls,
        np_image: np.ndarray,
        format: ImageFormat = ImageFormat.BGR,
        to_cuda: bool = False,
        **kwargs,
    ) -> "Image":
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
            )  # type: ignore
        return cls(
            NumpyImage(
                np.asarray(np_image),
                format,
                kwargs.get("frame_id", ""),
                kwargs.get("ts", time.time()),
            )
        )

    @classmethod
    def from_file(
        cls, filepath: str, format: ImageFormat = ImageFormat.BGR, to_cuda: bool = False, **kwargs
    ) -> "Image":
        if kwargs.pop("to_gpu", False):
            to_cuda = True
        arr = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
        if arr is None:
            raise ValueError(f"Could not load image from {filepath}")
        if arr.ndim == 2:
            detected = ImageFormat.GRAY16 if arr.dtype == np.uint16 else ImageFormat.GRAY
        elif arr.shape[2] == 3:
            detected = ImageFormat.BGR
        elif arr.shape[2] == 4:
            detected = ImageFormat.BGRA
        else:
            detected = format
        return cls(CudaImage(arr, detected) if to_cuda and HAS_CUDA else NumpyImage(arr, detected))  # type: ignore

    @classmethod
    def from_opencv(
        cls, cv_image: np.ndarray, format: ImageFormat = ImageFormat.BGR, **kwargs
    ) -> "Image":
        """Construct from an OpenCV image (NumPy array)."""
        return cls(
            NumpyImage(cv_image, format, kwargs.get("frame_id", ""), kwargs.get("ts", time.time()))
        )

    @classmethod
    def from_depth(
        cls, depth_data, frame_id: str = "", ts: float = None, to_cuda: bool = False
    ) -> "Image":
        arr = np.asarray(depth_data)
        if arr.dtype != np.float32:
            arr = arr.astype(np.float32)
        impl = (
            CudaImage(arr, ImageFormat.DEPTH, frame_id, time.time() if ts is None else ts)
            if to_cuda and HAS_CUDA
            else NumpyImage(arr, ImageFormat.DEPTH, frame_id, time.time() if ts is None else ts)
        )  # type: ignore
        return cls(impl)

    # Delegation
    @property
    def is_cuda(self) -> bool:
        return self._impl.is_cuda

    @property
    def data(self):
        return self._impl.data
    @data.setter
    def data(self, value) -> None:
        # Preserve backend semantics: ensure array type matches implementation
        if isinstance(self._impl, NumpyImage):
            self._impl.data = np.asarray(value)
        elif isinstance(self._impl, CudaImage):  # type: ignore
            if cp is None:
                raise RuntimeError("CuPy not available to set CUDA image data")
            self._impl.data = cp.asarray(value)  # type: ignore
        else:
            self._impl.data = value

    @property
    def format(self) -> ImageFormat:
        return self._impl.format
    @format.setter
    def format(self, value) -> None:
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
    def shape(self):
        return self._impl.shape

    @property
    def dtype(self):
        return self._impl.dtype

    def copy(self) -> "Image":
        return Image(self._impl.copy())

    def to_cpu(self) -> "Image":
        if isinstance(self._impl, NumpyImage):
            return self.copy()
        return Image(
            NumpyImage(
                np.asarray(self._impl.to_opencv()),
                self._impl.format,
                self._impl.frame_id,
                self._impl.ts,
            )
        )

    def to_cupy(self) -> "Image":
        if isinstance(self._impl, CudaImage):
            return self.copy()
        return Image(
            CudaImage(
                np.asarray(self._impl.data), self._impl.format, self._impl.frame_id, self._impl.ts
            )
        )  # type: ignore

    def to_opencv(self) -> np.ndarray:
        return self._impl.to_opencv()

    def to_rgb(self) -> "Image":
        return Image(self._impl.to_rgb())

    def to_bgr(self) -> "Image":
        return Image(self._impl.to_bgr())

    def to_grayscale(self) -> "Image":
        return Image(self._impl.to_grayscale())

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> "Image":
        return Image(self._impl.resize(width, height, interpolation))

    def sharpness(self) -> float:
        return self._impl.sharpness()

    def save(self, filepath: str) -> bool:
        return self._impl.save(filepath)

    def to_base64(self, quality: int = 80) -> str:
        return self._impl.to_base64(quality)

    # LCM encode/decode
    def lcm_encode(self, frame_id: Optional[str] = None) -> bytes:
        msg = LCMImage()
        msg.header = Header()
        msg.header.seq = 0
        msg.header.frame_id = frame_id or self.frame_id
        if self.ts is not None:
            msg.header.stamp.sec = int(self.ts)
            msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        else:
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nsec = int((now - int(now)) * 1e9)

        arr = (
            self.to_opencv()
            if self.format in (ImageFormat.BGR, ImageFormat.RGB, ImageFormat.RGBA, ImageFormat.BGRA)
            else self.to_opencv()
        )
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = _get_lcm_encoding(self.format, arr.dtype)
        msg.is_bigendian = False
        channels = 1 if arr.ndim == 2 else int(arr.shape[2])
        msg.step = int(arr.shape[1] * arr.dtype.itemsize * channels)
        img_bytes = arr.tobytes()
        msg.data_length = len(img_bytes)
        msg.data = img_bytes
        return msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs) -> "Image":
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
                msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
                if hasattr(msg, "header") and getattr(msg.header, "stamp", None)
                else time.time(),
            )
        )

    # PnP wrappers
    def solve_pnp(self, *args, **kwargs):
        return self._impl.solve_pnp(*args, **kwargs)  # type: ignore

    def solve_pnp_ransac(self, *args, **kwargs):
        return self._impl.solve_pnp_ransac(*args, **kwargs)  # type: ignore

    def solve_pnp_batch(self, *args, **kwargs):
        return self._impl.solve_pnp_batch(*args, **kwargs)  # type: ignore

    def create_csrt_tracker(self, *args, **kwargs):
        return self._impl.create_csrt_tracker(*args, **kwargs)  # type: ignore

    def csrt_update(self, *args, **kwargs):
        return self._impl.csrt_update(*args, **kwargs)  # type: ignore

    def __repr__(self) -> str:
        dev = "cuda" if self.is_cuda else "cpu"
        return f"Image(shape={self.shape}, format={self.format.value}, dtype={self.dtype}, dev={dev}, frame_id='{self.frame_id}', ts={self.ts})"

    def __eq__(self, other) -> bool:
        if not isinstance(other, Image):
            return False
        return (
            np.array_equal(self.to_opencv(), other.to_opencv())
            and self.format == other.format
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __len__(self) -> int:
        return int(self.height * self.width)


# Re-exports for tests
HAS_CUDA = HAS_CUDA
ImageFormat = ImageFormat
NVIMGCODEC_LAST_USED = NVIMGCODEC_LAST_USED
HAS_NVIMGCODEC = HAS_NVIMGCODEC


def sharpness_window(target_frequency: float, source: Observable[Image]) -> Observable[Image]:
    """Periodically emit the sharpest Image seen within the sliding window.

    Args:
        target_frequency: Emission frequency in Hz.
        source: An observable stream of Image instances.

    Returns:
        Observable emitting the sharpest Image in the current window at the
        specified frequency. Emits None if the window is empty.
    """
    window = TimestampedBufferCollection(1.0 / target_frequency)
    source.subscribe(window.add)

    thread_scheduler = ThreadPoolScheduler(max_workers=1)

    def find_best(*argv):
        if not window._items:
            return None
        return max(window._items, key=lambda x: x.sharpness())

    return rx.interval(1.0 / target_frequency).pipe(
        ops.observe_on(thread_scheduler), ops.map(find_best)
    )


def _get_lcm_encoding(fmt: ImageFormat, dtype: np.dtype) -> str:
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


def _parse_lcm_encoding(enc: str):
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
