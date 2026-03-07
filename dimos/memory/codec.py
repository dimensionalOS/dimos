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

import importlib
import pickle
from typing import TYPE_CHECKING, Any, Protocol, TypeVar

from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg

T = TypeVar("T")


class Codec(Protocol[T]):
    """Encodes/decodes payloads for storage."""

    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...


class LcmCodec:
    """Codec for DimosMsg types — uses lcm_encode/lcm_decode."""

    def __init__(self, msg_type: type[DimosMsg]) -> None:
        self._msg_type = msg_type

    def encode(self, value: DimosMsg) -> bytes:
        return value.lcm_encode()

    def decode(self, data: bytes) -> DimosMsg:
        return self._msg_type.lcm_decode(data)


class JpegCodec:
    """Codec for Image types — stores as JPEG bytes (lossy, ~10-20x smaller).

    Uses TurboJPEG (libjpeg-turbo) for 2-5x faster encode/decode vs OpenCV.
    Preserves ``frame_id`` as a short header: ``<len_u16><frame_id_utf8><jpeg_bytes>``.
    Pixel data is lossy-compressed; ``ts`` is NOT preserved (stored in the meta table).
    """

    def __init__(self, quality: int = 50) -> None:
        self._quality = quality
        from turbojpeg import TurboJPEG  # type: ignore[import-untyped]

        self._tj = TurboJPEG()

    _TJPF_MAP: dict[str, int] | None = None

    @staticmethod
    def _get_tjpf_map() -> dict[str, int]:
        if JpegCodec._TJPF_MAP is None:
            from turbojpeg import TJPF_BGR, TJPF_GRAY, TJPF_RGB  # type: ignore[import-untyped]

            JpegCodec._TJPF_MAP = {"BGR": TJPF_BGR, "RGB": TJPF_RGB, "GRAY": TJPF_GRAY}
        return JpegCodec._TJPF_MAP

    def encode(self, value: Any) -> bytes:
        import struct

        from turbojpeg import TJPF_BGR  # type: ignore[import-untyped]

        pf = self._get_tjpf_map().get(value.format.value, TJPF_BGR)
        jpeg_data: bytes = self._tj.encode(value.data, quality=self._quality, pixel_format=pf)
        frame_id = (value.frame_id or "").encode("utf-8")
        header = struct.pack("<H", len(frame_id)) + frame_id
        return header + jpeg_data

    def decode(self, data: bytes) -> Any:
        import struct

        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        fid_len = struct.unpack("<H", data[:2])[0]
        frame_id = data[2 : 2 + fid_len].decode("utf-8")
        jpeg_data = data[2 + fid_len :]
        arr = self._tj.decode(jpeg_data)
        if arr is None:
            raise ValueError("JPEG decoding failed")
        return Image(data=arr, format=ImageFormat.BGR, frame_id=frame_id)


class PickleCodec:
    """Fallback codec for arbitrary Python objects."""

    def encode(self, value: Any) -> bytes:
        return pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)

    def decode(self, data: bytes) -> Any:
        return pickle.loads(data)


def codec_for_type(payload_type: type | None) -> Codec[Any]:
    """Auto-select codec based on payload type."""
    if payload_type is not None:
        # Image → JPEG by default (much smaller than LCM raw pixels)

        if issubclass(payload_type, Image):
            return JpegCodec()
        if hasattr(payload_type, "lcm_encode") and hasattr(payload_type, "lcm_decode"):
            return LcmCodec(payload_type)  # type: ignore[arg-type]
    return PickleCodec()


def type_to_module_path(t: type) -> str:
    """Return fully qualified module path for a type, e.g. 'dimos.msgs.sensor_msgs.Image.Image'."""
    return f"{t.__module__}.{t.__qualname__}"


def module_path_to_type(path: str) -> type | None:
    """Resolve a fully qualified module path back to a type. Returns None on failure."""
    parts = path.rsplit(".", 1)
    if len(parts) != 2:
        return None
    module_path, class_name = parts
    try:
        mod = importlib.import_module(module_path)
        return getattr(mod, class_name, None)  # type: ignore[no-any-return]
    except (ImportError, AttributeError):
        return None
