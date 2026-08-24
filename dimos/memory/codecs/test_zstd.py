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

import numpy as np
import pytest

from dimos.memory.codecs.base import codec_from_id, codec_id
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.zstd import ZstdCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _depth_image() -> Image:
    pixels = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64)
    pixels[::7, ::5] = 0
    return Image(
        data=pixels,
        format=ImageFormat.DEPTH16,
        frame_id="depth_optical",
        ts=42.125,
    )


def test_zstd_lcm_roundtrip_is_exact() -> None:
    source = _depth_image()
    codec = ZstdCodec(LcmCodec(Image))

    decoded = codec.decode(codec.encode(source))

    assert decoded.format is source.format
    assert decoded.frame_id == source.frame_id
    assert decoded.ts == pytest.approx(source.ts)
    assert np.array_equal(decoded.data, source.data)


def test_codec_id_roundtrip() -> None:
    codec = codec_from_id("zstd+lcm", "dimos.msgs.sensor_msgs.Image.Image")

    assert isinstance(codec, ZstdCodec)
    assert codec_id(codec) == "zstd+lcm"


def test_zstd_requires_an_inner_codec() -> None:
    with pytest.raises(ValueError, match="must have an inner codec"):
        codec_from_id("zstd", "dimos.msgs.sensor_msgs.Image.Image")


def test_sqlite_reopen_restores_zstd_codec(tmp_path) -> None:
    db = str(tmp_path / "depth.db")
    source = _depth_image()
    with SqliteStore(path=db) as store:
        store.stream("depth", Image, codec="zstd+lcm").append(source, ts=source.ts)

    with SqliteStore(path=db) as reopened:
        decoded = reopened.stream("depth", Image).first().data

    assert decoded.frame_id == source.frame_id
    assert decoded.ts == pytest.approx(source.ts)
    assert np.array_equal(decoded.data, source.data)
