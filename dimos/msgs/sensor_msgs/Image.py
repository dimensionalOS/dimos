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


class Image:
    msg_name = "sensor_msgs.Image"

    def __init__(self, impl: AbstractImage):
        self._impl = impl

    # Construction
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

    @property
    def format(self) -> ImageFormat:
        return self._impl.format

    @property
    def frame_id(self) -> str:
        return self._impl.frame_id

    @property
    def ts(self) -> float:
        return self._impl.ts

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
