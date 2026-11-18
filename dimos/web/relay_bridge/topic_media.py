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

"""Fixed media codecs for a standalone provider, without robot message imports.

The bounded readers follow DimOS's Image/CompressedImage/PointCloud2 schemas.
They never resolve a type name from the network. JPEG uses libjpeg-turbo in the
pinned Pillow wheel; depth and clouds use zlib and typed-array wire data. This
module is loaded only when an explicitly enabled media subscription opens.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from io import BytesIO
import math
import re
import struct
from typing import Any, cast
import zlib

import numpy as np
from PIL import Image as PILImage

from dimos.web.relay_bridge.topic_profiles import MEDIA_PROFILES, MediaProfile, normalize_settings
from dimos.web.relay_bridge.topic_protocol import MAX_TOPIC_PAYLOAD_BYTES

MAX_MEDIA_PIXELS = 4_194_304
MAX_CLOUD_POINTS = 1_000_000
DEPTH_ENCODING = "application/vnd.dimos.depth.zlib.v1"
CLOUD_ENCODING = "application/vnd.dimos.cloud.q16.zlib.v1"


@dataclass(frozen=True)
class MediaOutput:
    payload: bytes
    encoding: str
    metadata: dict[str, Any]


class _Reader:
    def __init__(self, data: bytes, fingerprint: str) -> None:
        if len(data) > MAX_TOPIC_PAYLOAD_BYTES or data[:8].hex() != fingerprint:
            raise ValueError("invalid media payload or LCM fingerprint")
        self.data = memoryview(data)
        self.offset = 8

    def take(self, size: int) -> memoryview:
        if size < 0 or self.offset + size > len(self.data):
            raise ValueError("truncated LCM media")
        value = self.data[self.offset : self.offset + size]
        self.offset += size
        return value

    def integer(self) -> int:
        return int(struct.unpack(">i", self.take(4))[0])

    def byte(self) -> int:
        return int(self.take(1)[0])

    def flag(self) -> bool:
        value = self.byte()
        if value not in (0, 1):
            raise ValueError("invalid LCM boolean")
        return bool(value)

    def string(self) -> str:
        length = self.integer()
        if not 1 <= length <= 1025:
            raise ValueError("invalid LCM media string")
        data = self.take(length)
        if data[-1] != 0:
            raise ValueError("LCM string lacks terminator")
        return bytes(data[:-1]).decode("utf-8")

    def header(self) -> dict[str, Any]:
        seq, sec, nsec = self.integer(), self.integer(), self.integer()
        if not 0 <= nsec < 1_000_000_000:
            raise ValueError("invalid acquisition timestamp")
        return {"frameId": self.string(), "acquisitionTs": sec + nsec / 1e9, "headerSeq": seq}

    def end(self) -> None:
        if self.offset != len(self.data):
            raise ValueError("trailing LCM media bytes")


def _dimensions(width: int, height: int, cap: int) -> None:
    if width < 0 or height < 0 or width > cap or height > cap or width * height > cap:
        raise ValueError("media dimensions exceed budget")


def _compress(data: bytes) -> bytes:
    return zlib.compress(data, level=1)


def _jpeg(image: PILImage.Image, meta: dict[str, Any], settings: Mapping[str, Any]) -> MediaOutput:
    # The settings are normalized: the box never exceeds the profile size and
    # the quality never exceeds the profile quality. Report what is in force.
    width, height = settings["maxResolution"]
    quality = int(settings["quality"])
    try:
        image.thumbnail((width, height), PILImage.Resampling.BILINEAR)
        output = BytesIO()
        image.save(output, "JPEG", quality=quality, subsampling=2)
        meta.update(w=image.width, h=image.height, quality=quality)
        return MediaOutput(output.getvalue(), "image/jpeg", meta)
    finally:
        image.close()


def _compressed_image(data: bytes) -> tuple[PILImage.Image, dict[str, Any]]:
    reader = _Reader(data, "b8d011c10412b9a1")
    size = reader.integer()
    meta = reader.header()
    format_name = reader.string().strip().lower()
    if not re.fullmatch(
        r"(?:jpe?g|png|(?:rgb8|bgr8|rgba8|bgra8|mono8);\s*(?:jpeg|png) compressed(?: (?:rgb8|bgr8|rgba8|bgra8|mono8))?)",
        format_name,
    ):
        raise ValueError("unsupported compressed color format")
    pixels = reader.take(size)
    reader.end()
    try:
        with PILImage.open(BytesIO(pixels)) as opened:
            _dimensions(*opened.size, MAX_MEDIA_PIXELS)
            if opened.format not in ("JPEG", "PNG") or opened.mode not in ("L", "RGB", "RGBA", "P"):
                raise ValueError("unsupported compressed color image")
            if getattr(opened, "n_frames", 1) != 1:
                raise ValueError("animated media is unsupported")
            image = opened.convert("RGB")
    except PILImage.DecompressionBombError as error:
        raise ValueError("compressed dimensions exceed budget") from error
    meta.update(w=image.width, h=image.height)
    return image, meta


def decode_color(data: bytes) -> tuple[PILImage.Image, dict[str, Any]]:
    """An RGB picture and its acquisition metadata from an Image or CompressedImage.

    The caller closes the picture. Shared by the JPEG profiles and the H.264
    video encoder, so both read the same bounded LCM layouts.
    """
    if data[:8].hex() == "b8d011c10412b9a1":
        return _compressed_image(data)
    reader = _Reader(data, "535cface1f4f5717")
    size = reader.integer()
    meta = reader.header()
    height, width = reader.integer(), reader.integer()
    _dimensions(width, height, MAX_MEDIA_PIXELS)
    if not width or not height:
        raise ValueError("empty image")
    encoding = reader.string()
    reader.flag()  # byte order applies to multi-byte depth samples only
    step = reader.integer()
    pixels = reader.take(size)
    reader.end()
    meta.update(w=width, h=height)
    if encoding in ("jpeg", "jpg", "png"):
        try:
            with PILImage.open(BytesIO(pixels)) as opened:
                if opened.size != (width, height):
                    raise ValueError("compressed image shape mismatch")
                image = opened.convert("RGB")
        except PILImage.DecompressionBombError as error:
            raise ValueError("compressed dimensions exceed budget") from error
        return image, meta
    modes = {
        "rgb8": (3, "RGB"),
        "bgr8": (3, "BGR"),
        "rgba8": (4, "RGBA"),
        "bgra8": (4, "BGRA"),
        "mono8": (1, "L"),
        "8UC1": (1, "L"),
    }
    if encoding not in modes:
        raise ValueError("unsupported color image encoding")
    channels, mode = modes[encoding]
    if step < width * channels or len(pixels) != step * height:
        raise ValueError("invalid color row stride")
    target_mode = "L" if channels == 1 else "RGBA" if channels == 4 else "RGB"
    # Pillow accepts the buffer protocol; its stub omits memoryview.
    image = PILImage.frombytes(target_mode, (width, height), cast("Any", pixels), "raw", mode, step)
    if channels != 3:
        image = image.convert("RGB")
    return image, meta


def _image(data: bytes, profile: MediaProfile, settings: Mapping[str, Any]) -> MediaOutput:
    if profile.kind != "depth":
        return _jpeg(*decode_color(data), settings)
    if data[:8].hex() == "b8d011c10412b9a1":
        raise ValueError("compressed color image cannot use a depth profile")
    reader = _Reader(data, "535cface1f4f5717")
    size = reader.integer()
    meta = reader.header()
    height, width = reader.integer(), reader.integer()
    _dimensions(width, height, MAX_MEDIA_PIXELS)
    if not width or not height:
        raise ValueError("empty image")
    encoding, big, step = reader.string(), reader.flag(), reader.integer()
    pixels = reader.take(size)
    reader.end()
    meta.update(w=width, h=height)
    if width > 4096 or height > 4096:
        raise ValueError("depth dimensions exceed decoder budget")
    if encoding not in ("16UC1", "32FC1", "mono16"):
        raise ValueError("depth profile requires 16UC1 or 32FC1")
    itemsize = 4 if encoding == "32FC1" else 2
    if step < width * itemsize or len(pixels) != step * height:
        raise ValueError("invalid depth row stride")
    # Slice whole samples rather than convert floats: retain NaN payloads,
    # +/-infinity, signed zero, integer zero and every numeric bit exactly.
    rows = np.frombuffer(pixels, dtype=np.uint8).reshape(height, step)
    packed = rows[:, : width * itemsize].reshape(height, width, itemsize)
    if big:
        packed = packed[:, :, ::-1]
    meta.update(
        pixelEncoding=encoding,
        units="m" if encoding == "32FC1" else "mm",
        unitScale=1 if encoding == "32FC1" else 0.001,
        littleEndian=True,
    )
    return MediaOutput(_compress(packed.tobytes()), DEPTH_ENCODING, meta)


def _cloud(data: bytes, settings: Mapping[str, Any]) -> MediaOutput:
    keep_every = int(settings["keepEvery"])
    wanted = settings["fields"]
    reader = _Reader(data, "f5eb3da1c2853175")
    count, size = reader.integer(), reader.integer()
    if not 0 <= count <= 64:
        raise ValueError("too many point fields")
    meta = reader.header()
    height, width = reader.integer(), reader.integer()
    _dimensions(width, height, MAX_CLOUD_POINTS)
    fields: dict[str, tuple[int, int, int]] = {}
    for _ in range(count):
        name = reader.string()
        offset, kind, components = reader.integer(), reader.byte(), reader.integer()
        if name in fields:
            raise ValueError("duplicate point field")
        fields[name] = (offset, kind, components)
    big, stride, row_stride = reader.flag(), reader.integer(), reader.integer()
    pixels = reader.take(size)
    reader.flag()  # is_dense is a hint; non-finite positions are always filtered.
    reader.end()
    if not 1 <= stride <= 4096 or row_stride < width * stride or len(pixels) != height * row_stride:
        raise ValueError("invalid point cloud strides")
    formats = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}

    def field(name: str, *, packed_color: bool = False) -> np.ndarray[Any, Any]:
        if name not in fields:
            raise ValueError(f"missing point field {name}")
        offset, kind, components = fields[name]
        if kind not in formats or components != 1:
            raise ValueError("unsupported point field")
        dtype = np.dtype((">" if big else "<") + formats[kind])
        if offset < 0 or offset + dtype.itemsize > stride:
            raise ValueError("point field exceeds stride")
        if packed_color:
            if dtype.itemsize != 4:
                raise ValueError("packed color requires four bytes")
            dtype = np.dtype(">u4" if big else "<u4")
        return np.ndarray(
            (height, width), dtype, buffer=pixels, offset=offset, strides=(row_stride, stride)
        ).reshape(-1)

    if width * height:
        xyz = np.column_stack([field(axis) for axis in ("x", "y", "z")]).astype(np.float64)
        indices = np.flatnonzero(np.isfinite(xyz).all(axis=1))[::keep_every]
        xyz = xyz[indices]
    else:
        xyz, indices = np.empty((0, 3)), np.empty(0, dtype=np.intp)
    if xyz.size:
        lo, hi = xyz.min(axis=0), xyz.max(axis=0)
        origin = lo / 2 + hi / 2
        scale = max(float(np.max(hi / 2 - lo / 2)) / 32767, 1e-12)
        if not np.isfinite(origin).all() or not math.isfinite(scale) or np.max(np.abs(xyz)) > 1e30:
            raise ValueError("point coordinates exceed supported numeric range")
        positions = np.clip(np.rint((xyz - origin) / scale), -32767, 32767).astype("<i2")
    else:
        origin, scale, positions = np.zeros(3), 1.0, np.empty((0, 3), dtype="<i2")
    chunks = [positions.tobytes()]
    has_intensity = "intensity" in wanted and "intensity" in fields and width * height > 0
    has_color = "color" in wanted and ("rgb" in fields or "rgba" in fields) and width * height > 0
    if has_intensity:
        chunks.append(field("intensity")[indices].astype("<f4").tobytes())
    if has_color:
        rgb = field("rgb" if "rgb" in fields else "rgba", packed_color=True)[indices]
        color = np.column_stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255]).astype(np.uint8)
        chunks.append(color.tobytes())
    meta.update(
        count=len(indices),
        sourceCount=width * height,
        origin=origin.tolist(),
        scale=scale,
        maxError=scale / 2,
        keepEvery=keep_every,
        intensity=has_intensity,
        color=has_color,
    )
    return MediaOutput(_compress(b"".join(chunks)), CLOUD_ENCODING, meta)


def encode_media(
    profile_name: str, data: bytes, settings: Mapping[str, Any] | None = None
) -> MediaOutput:
    """One bounded conversion; callers select/rate-limit before scheduling it.

    ``settings`` is a request against the profile (absent keys take the
    profile values); the metadata reports the values in force.
    """
    profile = MEDIA_PROFILES[profile_name]
    applied = normalize_settings(profile_name, settings)
    output = _cloud(data, applied) if profile.kind == "cloud" else _image(data, profile, applied)
    if len(output.payload) > MAX_TOPIC_PAYLOAD_BYTES:
        raise ValueError("encoded media exceeds payload budget")
    output.metadata["profile"] = profile_name
    return output
