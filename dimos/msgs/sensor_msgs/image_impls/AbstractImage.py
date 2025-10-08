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
import os
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any

import cv2
import numpy as np

try:
    import cupy as cp  # type: ignore

    HAS_CUDA = True
except Exception:  # pragma: no cover - optional dependency
    cp = None  # type: ignore
    HAS_CUDA = False

# Optional nvImageCodec (preferred GPU codec)
USE_NVIMGCODEC = os.environ.get("USE_NVIMGCODEC", "0") == "1"
NVIMGCODEC_LAST_USED = False
try:  # pragma: no cover - optional dependency
    if HAS_CUDA and USE_NVIMGCODEC:
        from nvidia import nvimgcodec  # type: ignore

        try:
            _enc_probe = nvimgcodec.Encoder()  # type: ignore[attr-defined]
            HAS_NVIMGCODEC = True
        except Exception:
            nvimgcodec = None  # type: ignore
            HAS_NVIMGCODEC = False
    else:
        nvimgcodec = None  # type: ignore
        HAS_NVIMGCODEC = False
except Exception:  # pragma: no cover - optional dependency
    nvimgcodec = None  # type: ignore
    HAS_NVIMGCODEC = False


class ImageFormat(Enum):
    BGR = "BGR"
    RGB = "RGB"
    RGBA = "RGBA"
    BGRA = "BGRA"
    GRAY = "GRAY"
    GRAY16 = "GRAY16"
    DEPTH = "DEPTH"
    DEPTH16 = "DEPTH16"


def _is_cu(x) -> bool:
    return HAS_CUDA and cp is not None and isinstance(x, cp.ndarray)  # type: ignore


def _ascontig(x):
    if _is_cu(x):
        return x if x.flags["C_CONTIGUOUS"] else cp.ascontiguousarray(x)  # type: ignore
    return x if x.flags["C_CONTIGUOUS"] else np.ascontiguousarray(x)


def _to_cpu(x):
    return cp.asnumpy(x) if _is_cu(x) else x  # type: ignore


def _to_cu(x):
    if HAS_CUDA and cp is not None and isinstance(x, np.ndarray):  # type: ignore
        return cp.asarray(x)  # type: ignore
    return x


def _encode_nvimgcodec_cuda(bgr_cu, quality: int = 80) -> bytes:  # pragma: no cover - optional
    if not HAS_NVIMGCODEC or nvimgcodec is None:
        raise RuntimeError("nvimgcodec not available")
    if bgr_cu.ndim != 3 or bgr_cu.shape[2] != 3:
        raise RuntimeError("nvimgcodec expects HxWx3 image")
    if bgr_cu.dtype != cp.uint8:  # type: ignore[attr-defined]
        raise RuntimeError("nvimgcodec requires uint8 input")
    if not bgr_cu.flags["C_CONTIGUOUS"]:
        bgr_cu = cp.ascontiguousarray(bgr_cu)  # type: ignore[attr-defined]
    encoder = nvimgcodec.Encoder()  # type: ignore[attr-defined]
    try:
        img = nvimgcodec.Image(bgr_cu, nvimgcodec.PixelFormat.BGR)  # type: ignore[attr-defined]
    except Exception:
        img = nvimgcodec.Image(cp.asnumpy(bgr_cu), nvimgcodec.PixelFormat.BGR)  # type: ignore[attr-defined]
    if hasattr(nvimgcodec, "EncodeParams"):
        params = nvimgcodec.EncodeParams(quality=quality)  # type: ignore[attr-defined]
        bitstreams = encoder.encode([img], [params])
    else:
        bitstreams = encoder.encode([img])
    bs0 = bitstreams[0]
    if hasattr(bs0, "buf"):
        return bytes(bs0.buf)
    return bytes(bs0)


class AbstractImage(ABC):
    data: Any
    format: ImageFormat
    frame_id: str
    ts: float

    @property
    @abstractmethod
    def is_cuda(self) -> bool:  # pragma: no cover - abstract
        ...

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def channels(self) -> int:
        if getattr(self.data, "ndim", 0) == 2:
            return 1
        if getattr(self.data, "ndim", 0) == 3:
            return int(self.data.shape[2])
        raise ValueError("Invalid image dimensions")

    @property
    def shape(self):
        return tuple(self.data.shape)

    @property
    def dtype(self):
        return self.data.dtype

    @abstractmethod
    def to_opencv(self) -> np.ndarray:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_rgb(self) -> "AbstractImage":  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_bgr(self) -> "AbstractImage":  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_grayscale(self) -> "AbstractImage":  # pragma: no cover - abstract
        ...

    @abstractmethod
    def resize(
        self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR
    ) -> "AbstractImage":  # pragma: no cover - abstract
        ...

    @abstractmethod
    def sharpness(self) -> float:  # pragma: no cover - abstract
        ...

    def copy(self) -> "AbstractImage":
        return self.__class__(
            data=self.data.copy(), format=self.format, frame_id=self.frame_id, ts=self.ts
        )  # type: ignore

    def save(self, filepath: str) -> bool:
        global NVIMGCODEC_LAST_USED
        if self.is_cuda and HAS_NVIMGCODEC and nvimgcodec is not None:
            try:
                bgr = self.to_bgr()
                if _is_cu(bgr.data):
                    jpeg = _encode_nvimgcodec_cuda(bgr.data)
                    NVIMGCODEC_LAST_USED = True
                    with open(filepath, "wb") as f:
                        f.write(jpeg)
                    return True
            except Exception:
                NVIMGCODEC_LAST_USED = False
        arr = self.to_opencv()
        return cv2.imwrite(filepath, arr)

    def to_base64(self, quality: int = 80) -> str:
        global NVIMGCODEC_LAST_USED
        if self.is_cuda and HAS_NVIMGCODEC and nvimgcodec is not None:
            try:
                bgr = self.to_bgr()
                if _is_cu(bgr.data):
                    jpeg = _encode_nvimgcodec_cuda(bgr.data, quality=quality)
                    NVIMGCODEC_LAST_USED = True
                    return base64.b64encode(jpeg).decode("utf-8")
            except Exception:
                NVIMGCODEC_LAST_USED = False
        bgr = self.to_bgr()
        success, buffer = cv2.imencode(
            ".jpg", _to_cpu(bgr.data), [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
        if not success:
            raise ValueError("Failed to encode image as JPEG")
        return base64.b64encode(buffer.tobytes()).decode("utf-8")
