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

from pathlib import Path
import platform

import numpy as np
import pytest

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_from_id, codec_id
from dimos.memory2.codecs.jpeg import JpegCodec
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.video.h264 import H264ImageCodec
from dimos.msgs.sensor_msgs.Image import H264_IMAGE_ENCODING, Image, ImageFormat

_SKIP_SQLITE_VEC = platform.machine() == "aarch64" or platform.system() == "Darwin"


def _raw_image(seq: int, fmt: ImageFormat = ImageFormat.RGB) -> Image:
    data = np.full((2, 2, 3), seq, dtype=np.uint8)
    if fmt == ImageFormat.GRAY:
        data = np.full((2, 2), seq, dtype=np.uint8)
    return Image.from_numpy(data, format=fmt, frame_id="cam", ts=float(seq))


def _encoded_image(seq: int, *, key: bool = True) -> Image:
    return Image.encoded(
        data=b"\x00\x00\x00\x01\x65" + bytes([seq]),
        encoding=H264_IMAGE_ENCODING,
        format=ImageFormat.RGB,
        frame_id="cam",
        ts=float(seq),
        codec_metadata={
            "seq": seq,
            "codec": "h264",
            "bitstream": "annex_b",
            "is_keyframe": key,
            "keyframe_seq": seq if key else 0,
            "pts": seq * 90,
            "width": 2,
            "height": 2,
            "channels": 3,
            "dtype": "uint8",
        },
    )


def test_h264_image_codec_roundtrips_encoded_image() -> None:
    codec = H264ImageCodec()
    image = _encoded_image(1)

    decoded = codec.decode(codec.encode(image))

    assert decoded == image
    assert decoded.encoding == H264_IMAGE_ENCODING
    assert decoded.codec_metadata["seq"] == 1
    assert decoded.width == 2
    assert decoded.height == 2


def test_h264_image_codec_rejects_raw_images() -> None:
    codec = H264ImageCodec()

    with pytest.raises(ValueError, match="encoded Images"):
        codec.encode(_raw_image(1))


def test_codec_id_and_factory_support_h264_for_image() -> None:
    codec = H264ImageCodec()

    assert codec_id(codec) == "h264"
    assert isinstance(codec_from_id("h264", "dimos.msgs.sensor_msgs.Image.Image"), H264ImageCodec)


def test_h264_stream_stores_encoded_images_with_normal_backend(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "h264.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264")
        stored = stream.append(_encoded_image(1), ts=1.0)
        assert stored.data.encoding == H264_IMAGE_ENCODING
        assert stored.data.codec_metadata["seq"] == 1

    with SqliteStore(path=str(db), must_exist=True) as reopened:
        stream = reopened.stream("cam", Image)
        obs = stream.first()
        assert obs.data.encoding == H264_IMAGE_ENCODING
        assert obs.data.codec_metadata["seq"] == 1
        assert obs.data.width == 2


def test_h264_replay_emits_encoded_images(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    stream = store.stream("cam", Image, codec="h264")
    stream.append(_encoded_image(1), ts=1.0)
    stream.append(_encoded_image(2, key=False), ts=2.0)

    replayed = list(store.replay().streams.cam.iterate())

    assert [image.encoding for image in replayed] == [H264_IMAGE_ENCODING, H264_IMAGE_ENCODING]
    assert [image.codec_metadata["seq"] for image in replayed] == [1, 2]


def test_default_image_stream_still_uses_jpeg_codec(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    store = SqliteStore(path=str(tmp_path / "jpeg.db"))
    stream = store.stream("rgb", Image)

    source = stream._source
    assert isinstance(source, Backend)
    assert isinstance(source.codec, JpegCodec)


def test_encoded_images_reject_pixel_operations() -> None:
    image = _encoded_image(1)

    with pytest.raises(ValueError, match="requires raw Image data"):
        image.to_rgb()
    with pytest.raises(ValueError, match="requires raw Image data"):
        image.as_numpy()
