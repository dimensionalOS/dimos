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

"""Small GLB utilities shared by scene-package tools."""

from __future__ import annotations

from io import BytesIO
import json
from pathlib import Path
import shutil
import struct
import tempfile
from typing import Any

from PIL import Image

GLB_MAGIC = b"glTF"
GLB_VERSION = 2
GLB_HEADER_SIZE = 12
GLB_CHUNK_HEADER_SIZE = 8
GLB_JSON_CHUNK_TYPE = 0x4E4F534A
GLB_BIN_CHUNK_TYPE = 0x004E4942
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
STANDARD_TEXTURE_MIME_TYPES = {"image/png", "image/jpeg"}
STANDARD_TEXTURE_MODES = {"RGB", "RGBA"}


def read_glb(path: Path) -> tuple[dict[str, Any], bytes]:
    """Read a GLB v2 file as a JSON chunk plus one BIN chunk."""
    data = path.read_bytes()
    if len(data) < GLB_HEADER_SIZE:
        raise RuntimeError(f"invalid GLB header: {path}")
    magic, version, declared_length = struct.unpack_from("<4sII", data, 0)
    if magic != GLB_MAGIC or version != GLB_VERSION:
        raise RuntimeError(f"expected GLB v2 file: {path}")
    if declared_length != len(data):
        raise RuntimeError(
            f"GLB length mismatch for {path}: header={declared_length} actual={len(data)}"
        )

    offset = GLB_HEADER_SIZE
    json_bytes: bytes | None = None
    bin_chunk: bytes | None = None
    while offset < len(data):
        if offset + GLB_CHUNK_HEADER_SIZE > len(data):
            raise RuntimeError(f"truncated GLB chunk header: {path}")
        chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
        offset += GLB_CHUNK_HEADER_SIZE
        chunk_end = offset + chunk_length
        if chunk_end > len(data):
            raise RuntimeError(f"truncated GLB chunk payload: {path}")
        chunk = data[offset:chunk_end]
        if chunk_type == GLB_JSON_CHUNK_TYPE:
            json_bytes = chunk
        elif chunk_type == GLB_BIN_CHUNK_TYPE:
            bin_chunk = chunk
        offset = chunk_end

    if json_bytes is None or bin_chunk is None:
        raise RuntimeError(f"GLB must contain JSON and BIN chunks: {path}")
    gltf = json.loads(json_bytes.rstrip(b" \t\r\n\0").decode("utf-8"))
    if not isinstance(gltf, dict):
        raise RuntimeError(f"GLB JSON chunk is not an object: {path}")
    return gltf, bin_chunk


def write_glb(
    path: Path,
    gltf: dict[str, Any],
    bin_chunk: bytes,
    buffer_view_replacements: dict[int, bytes],
) -> None:
    """Rewrite a GLB while preserving or replacing bufferView payloads."""
    buffer_views = gltf.get("bufferViews")
    buffers = gltf.get("buffers")
    if not isinstance(buffer_views, list) or not isinstance(buffers, list) or len(buffers) != 1:
        raise RuntimeError(f"cannot rewrite GLB buffer views: {path}")

    new_bin = bytearray()
    for index, view in enumerate(buffer_views):
        if not isinstance(view, dict):
            raise RuntimeError(f"invalid GLB bufferView at index {index}: {path}")
        payload = buffer_view_replacements.get(index)
        if payload is None:
            payload = buffer_view_bytes(bin_chunk, view)
        _pad_bytearray(new_bin, alignment=4, pad=0)
        view["byteOffset"] = len(new_bin)
        view["byteLength"] = len(payload)
        new_bin.extend(payload)
    _pad_bytearray(new_bin, alignment=4, pad=0)
    buffers[0]["byteLength"] = len(new_bin)

    json_chunk = json.dumps(gltf, separators=(",", ":"), sort_keys=True).encode("utf-8")
    json_chunk = _padded_bytes(json_chunk, alignment=4, pad=b" ")
    bin_bytes = bytes(new_bin)
    total_length = (
        GLB_HEADER_SIZE
        + GLB_CHUNK_HEADER_SIZE
        + len(json_chunk)
        + GLB_CHUNK_HEADER_SIZE
        + len(bin_bytes)
    )
    with tempfile.NamedTemporaryFile("wb", suffix=".glb", delete=False) as temp:
        temp_path = Path(temp.name)
        temp.write(struct.pack("<4sII", GLB_MAGIC, GLB_VERSION, total_length))
        temp.write(struct.pack("<II", len(json_chunk), GLB_JSON_CHUNK_TYPE))
        temp.write(json_chunk)
        temp.write(struct.pack("<II", len(bin_bytes), GLB_BIN_CHUNK_TYPE))
        temp.write(bin_bytes)
    try:
        shutil.move(str(temp_path), path)
    finally:
        temp_path.unlink(missing_ok=True)


def buffer_view_bytes(bin_chunk: bytes, view: dict[str, Any]) -> bytes:
    """Return the payload bytes for one bufferView in a single-BIN GLB."""
    if int(view.get("buffer", 0)) != 0:
        raise RuntimeError("embedded texture normalization only supports buffer 0")
    byte_offset = int(view.get("byteOffset", 0))
    byte_length = int(view["byteLength"])
    return bin_chunk[byte_offset : byte_offset + byte_length]


def demote_required_extensions(path: Path, extensions: set[str]) -> set[str]:
    """Move selected GLB extensions from required to used."""
    gltf, bin_chunk = read_glb(path)
    required = gltf.get("extensionsRequired")
    if not isinstance(required, list):
        return set()

    demoted = {extension for extension in required if extension in extensions}
    if not demoted:
        return set()

    next_required = [extension for extension in required if extension not in demoted]
    if next_required:
        gltf["extensionsRequired"] = next_required
    else:
        gltf.pop("extensionsRequired", None)
    used = gltf.get("extensionsUsed")
    if isinstance(used, list):
        merged = list(dict.fromkeys([*used, *sorted(demoted)]))
        gltf["extensionsUsed"] = merged
    else:
        gltf["extensionsUsed"] = sorted(demoted)
    write_glb(path, gltf, bin_chunk, {})
    return demoted


def normalize_embedded_textures(path: Path) -> int:
    """Rewrite non-standard embedded textures to ordinary 8-bit PNGs."""
    gltf, bin_chunk = read_glb(path)
    images = gltf.get("images")
    buffer_views = gltf.get("bufferViews")
    buffers = gltf.get("buffers")
    if not isinstance(images, list) or not images:
        return 0
    if not isinstance(buffer_views, list) or not isinstance(buffers, list):
        return 0
    if len(buffers) != 1:
        raise RuntimeError(f"cannot normalize textures in multi-buffer GLB: {path}")

    replacements: dict[int, bytes] = {}
    for image_index, image in enumerate(images):
        if not isinstance(image, dict):
            continue
        buffer_view_index = image.get("bufferView")
        if not isinstance(buffer_view_index, int):
            continue
        if buffer_view_index < 0 or buffer_view_index >= len(buffer_views):
            raise RuntimeError(
                f"image {image_index} references missing bufferView {buffer_view_index}: {path}"
            )
        view = buffer_views[buffer_view_index]
        if not isinstance(view, dict):
            continue
        texture_bytes = buffer_view_bytes(bin_chunk, view)
        normalized = normalized_texture_bytes(
            texture_bytes,
            mime_type=image.get("mimeType"),
        )
        if normalized is None:
            continue
        replacements[buffer_view_index] = normalized
        image["mimeType"] = "image/png"
        image.pop("uri", None)

    if not replacements:
        return 0

    write_glb(path, gltf, bin_chunk, replacements)
    return len(replacements)


def normalized_texture_bytes(
    texture_bytes: bytes,
    *,
    mime_type: Any,
) -> bytes | None:
    """Return an 8-bit PNG replacement, or None when the texture is standard."""
    try:
        with Image.open(BytesIO(texture_bytes)) as image:
            image.load()
            if _is_standard_embedded_texture(image, texture_bytes, mime_type):
                return None
            has_alpha = image.mode in {"RGBA", "LA"} or "transparency" in image.info
            mode = "RGBA" if has_alpha else "RGB"
            converted = image.convert(mode)
            out = BytesIO()
            converted.save(out, format="PNG", compress_level=1)
            return out.getvalue()
    except Exception as exc:
        raise RuntimeError("failed to normalize embedded GLB texture to 8-bit PNG") from exc


def _is_standard_embedded_texture(
    image: Image.Image,
    texture_bytes: bytes,
    mime_type: Any,
) -> bool:
    if mime_type not in STANDARD_TEXTURE_MIME_TYPES:
        return False
    if image.mode not in STANDARD_TEXTURE_MODES:
        return False
    return not _is_high_bit_depth_png(texture_bytes)


def _is_high_bit_depth_png(texture_bytes: bytes) -> bool:
    if not texture_bytes.startswith(PNG_SIGNATURE) or len(texture_bytes) < 25:
        return False
    return texture_bytes[24] > 8


def _pad_bytearray(data: bytearray, *, alignment: int, pad: int) -> None:
    while len(data) % alignment:
        data.append(pad)


def _padded_bytes(data: bytes, *, alignment: int, pad: bytes) -> bytes:
    while len(data) % alignment:
        data += pad
    return data
