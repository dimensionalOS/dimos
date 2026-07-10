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

from io import BytesIO
import json
from pathlib import Path
import struct

from PIL import Image

from dimos.experimental.scene_cooking.source_assets.glb import (
    GLB_BIN_CHUNK_TYPE,
    GLB_CHUNK_HEADER_SIZE,
    GLB_HEADER_SIZE,
    GLB_JSON_CHUNK_TYPE,
    GLB_MAGIC,
    GLB_VERSION,
    buffer_view_bytes,
    demote_required_extensions,
    normalize_embedded_textures,
    read_glb,
)


def test_normalize_embedded_textures_preserves_glb_and_rewrites_png8(
    tmp_path: Path,
) -> None:
    geometry_payload = b"meshbytes"
    texture_payload = _png16_texture()
    path = tmp_path / "textured.glb"
    _write_test_glb(path, geometry_payload, texture_payload)

    count = normalize_embedded_textures(path)

    assert count == 1
    gltf, bin_chunk = read_glb(path)
    assert gltf["images"][0]["mimeType"] == "image/png"
    assert buffer_view_bytes(bin_chunk, gltf["bufferViews"][0]) == geometry_payload

    normalized_texture = buffer_view_bytes(bin_chunk, gltf["bufferViews"][1])
    with Image.open(BytesIO(normalized_texture)) as image:
        assert image.format == "PNG"
        assert image.mode == "RGB"


def test_demote_required_extensions_keeps_extension_optional(tmp_path: Path) -> None:
    path = tmp_path / "extension.glb"
    _write_test_glb(
        path,
        geometry_payload=b"meshbytes",
        texture_payload=_png16_texture(),
        required_extensions=["KHR_texture_transform", "EXT_texture_webp"],
        used_extensions=["KHR_texture_transform", "EXT_texture_webp"],
    )

    demoted = demote_required_extensions(path, {"KHR_texture_transform"})

    assert demoted == {"KHR_texture_transform"}
    gltf, _ = read_glb(path)
    assert gltf["extensionsRequired"] == ["EXT_texture_webp"]
    assert "KHR_texture_transform" in gltf["extensionsUsed"]


def _png16_texture() -> bytes:
    image = Image.new("I;16", (2, 2))
    image.putdata([0, 256, 32768, 65535])
    out = BytesIO()
    image.save(out, format="PNG")
    return out.getvalue()


def _write_test_glb(
    path: Path,
    geometry_payload: bytes,
    texture_payload: bytes,
    required_extensions: list[str] | None = None,
    used_extensions: list[str] | None = None,
) -> None:
    bin_chunk = bytearray()
    geometry_offset = len(bin_chunk)
    bin_chunk.extend(geometry_payload)
    _pad(bin_chunk)
    texture_offset = len(bin_chunk)
    bin_chunk.extend(texture_payload)
    _pad(bin_chunk)

    gltf = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(bin_chunk)}],
        "bufferViews": [
            {
                "buffer": 0,
                "byteOffset": geometry_offset,
                "byteLength": len(geometry_payload),
            },
            {
                "buffer": 0,
                "byteOffset": texture_offset,
                "byteLength": len(texture_payload),
            },
        ],
        "images": [{"bufferView": 1, "mimeType": "image/png"}],
    }
    if required_extensions is not None:
        gltf["extensionsRequired"] = required_extensions
    if used_extensions is not None:
        gltf["extensionsUsed"] = used_extensions
    json_chunk = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_chunk = _padded(json_chunk, b" ")
    bin_bytes = bytes(bin_chunk)
    total_length = (
        GLB_HEADER_SIZE
        + GLB_CHUNK_HEADER_SIZE
        + len(json_chunk)
        + GLB_CHUNK_HEADER_SIZE
        + len(bin_bytes)
    )
    with path.open("wb") as file:
        file.write(struct.pack("<4sII", GLB_MAGIC, GLB_VERSION, total_length))
        file.write(struct.pack("<II", len(json_chunk), GLB_JSON_CHUNK_TYPE))
        file.write(json_chunk)
        file.write(struct.pack("<II", len(bin_bytes), GLB_BIN_CHUNK_TYPE))
        file.write(bin_bytes)


def _pad(data: bytearray) -> None:
    while len(data) % 4:
        data.append(0)


def _padded(data: bytes, pad: bytes) -> bytes:
    while len(data) % 4:
        data += pad
    return data
