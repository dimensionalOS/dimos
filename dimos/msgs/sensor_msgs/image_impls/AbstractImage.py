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

from abc import ABC, abstractmethod
import base64
from enum import Enum
import os
from typing import Any

import cv2
import numpy as np
import rerun as rr

try:
    import cupy as cp  # type: ignore[import-not-found, import-untyped]

    HAS_CUDA = True
except Exception:  # pragma: no cover - optional dependency
    cp = None
    HAS_CUDA = False

# NVRTC defaults to C++11; libcu++ in recent CUDA requires at least C++17.
if HAS_CUDA:
    try:
        import cupy.cuda.compiler as _cupy_compiler  # type: ignore[import-not-found, import-untyped]

        if not getattr(_cupy_compiler, "_dimos_force_cxx17", False):
            _orig_compile_using_nvrtc = _cupy_compiler.compile_using_nvrtc

            def _compile_using_nvrtc(  # type: ignore[no-untyped-def]
                source, options=(), *args, **kwargs
            ):
                filtered = tuple(
                    opt
                    for opt in options
                    if opt not in ("-std=c++11", "--std=c++11", "-std=c++14", "--std=c++14")
                )
                if "--std=c++17" not in filtered and "-std=c++17" not in filtered:
                    filtered = (*filtered, "--std=c++17")
                return _orig_compile_using_nvrtc(source, filtered, *args, **kwargs)

            _cupy_compiler.compile_using_nvrtc = _compile_using_nvrtc
            _cupy_compiler._dimos_force_cxx17 = True
    except Exception:
        pass

# Optional nvImageCodec (preferred GPU codec)
USE_NVIMGCODEC = os.environ.get("USE_NVIMGCODEC", "0") == "1"
NVIMGCODEC_LAST_USED = False
try:  # pragma: no cover - optional dependency
    if HAS_CUDA and USE_NVIMGCODEC:
        from nvidia import nvimgcodec  # type: ignore[import-untyped]

        try:
            _enc_probe = nvimgcodec.Encoder()
            HAS_NVIMGCODEC = True
        except Exception:
            nvimgcodec = None
            HAS_NVIMGCODEC = False
    else:
        nvimgcodec = None
        HAS_NVIMGCODEC = False
except Exception:  # pragma: no cover - optional dependency
    nvimgcodec = None
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


def _is_cu(x) -> bool:  # type: ignore[no-untyped-def]
    return HAS_CUDA and cp is not None and isinstance(x, cp.ndarray)


def _ascontig(x):  # type: ignore[no-untyped-def]
    if _is_cu(x):
        return x if x.flags["C_CONTIGUOUS"] else cp.ascontiguousarray(x)
    return x if x.flags["C_CONTIGUOUS"] else np.ascontiguousarray(x)


def _to_cpu(x):  # type: ignore[no-untyped-def]
    return cp.asnumpy(x) if _is_cu(x) else x


def _to_cu(x):  # type: ignore[no-untyped-def]
    if HAS_CUDA and cp is not None and isinstance(x, np.ndarray):
        return cp.asarray(x)
    return x


def _encode_nvimgcodec_cuda(bgr_cu, quality: int = 80) -> bytes:  # type: ignore[no-untyped-def]  # pragma: no cover - optional
    if not HAS_NVIMGCODEC or nvimgcodec is None:
        raise RuntimeError("nvimgcodec not available")
    if bgr_cu.ndim != 3 or bgr_cu.shape[2] != 3:
        raise RuntimeError("nvimgcodec expects HxWx3 image")
    if bgr_cu.dtype != cp.uint8:
        raise RuntimeError("nvimgcodec requires uint8 input")
    if not bgr_cu.flags["C_CONTIGUOUS"]:
        bgr_cu = cp.ascontiguousarray(bgr_cu)
    encoder = nvimgcodec.Encoder()
    try:
        img = nvimgcodec.Image(bgr_cu, nvimgcodec.PixelFormat.BGR)
    except Exception:
        img = nvimgcodec.Image(cp.asnumpy(bgr_cu), nvimgcodec.PixelFormat.BGR)
    if hasattr(nvimgcodec, "EncodeParams"):
        params = nvimgcodec.EncodeParams(quality=quality)
        bitstreams = encoder.encode([img], [params])
    else:
        bitstreams = encoder.encode([img])
    bs0 = bitstreams[0]
    if hasattr(bs0, "buf"):
        return bytes(bs0.buf)
    return bytes(bs0)


def format_to_rerun(data, fmt: ImageFormat):  # type: ignore[no-untyped-def]
    """Convert image data to Rerun archetype based on format.

    Args:
        data: Image data (numpy array or cupy array on CPU)
        fmt: ImageFormat enum value

    Returns:
        Rerun archetype (rr.Image or rr.DepthImage)
    """
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
    def shape(self):  # type: ignore[no-untyped-def]
        return tuple(self.data.shape)

    @property
    def dtype(self):  # type: ignore[no-untyped-def]
        return self.data.dtype

    @abstractmethod
    def to_opencv(self) -> np.ndarray:  # type: ignore[type-arg]  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_rgb(self) -> AbstractImage:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_bgr(self) -> AbstractImage:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_grayscale(self) -> AbstractImage:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def resize(
        self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR
    ) -> AbstractImage:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def to_rerun(self) -> Any:  # pragma: no cover - abstract
        ...

    @abstractmethod
    def sharpness(self) -> float:  # pragma: no cover - abstract
        ...

    def copy(self) -> AbstractImage:
        return self.__class__(  # type: ignore[call-arg]
            data=self.data.copy(), format=self.format, frame_id=self.frame_id, ts=self.ts
        )

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
            ".jpg",
            _to_cpu(bgr.data),  # type: ignore[no-untyped-call]
            [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
        )
        if not success:
            raise ValueError("Failed to encode image as JPEG")
        return base64.b64encode(buffer.tobytes()).decode("utf-8")
