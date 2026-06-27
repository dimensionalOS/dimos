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
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.store.sqlite import SqliteStore
import dimos.memory2.video.h264 as h264_storage
from dimos.memory2.video.h264 import H264ImageBackend, H264ImageCodec
from dimos.models.embedding.base import Embedding
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.video.h264 import H264Config, H264Packet

_SKIP_SQLITE_VEC = platform.machine() == "aarch64" or platform.system() == "Darwin"


class FakeH264Encoder:
    def __init__(self, _config: object) -> None:
        self.seq = 0
        self.keyframe_seq = -1

    def encode(self, image: Image, *, force_keyframe: bool = False) -> H264Packet:
        if force_keyframe or self.seq == 0:
            self.keyframe_seq = self.seq
            key = True
        else:
            key = False
        packet = H264Packet(
            data=b"\x00\x00\x00\x01" + bytes([self.seq]),
            format=image.format,
            frame_id=image.frame_id,
            ts=image.ts,
            seq=self.seq,
            pts=self.seq * 90,
            is_keyframe=key,
            keyframe_seq=self.keyframe_seq,
            width=image.width,
            height=image.height,
            channels=image.channels,
            dtype=str(image.dtype),
        )
        self.seq += 1
        return packet


class FakeH264Decoder:
    decoded_sequences: list[int] = []

    def __init__(self, _config: object) -> None:
        pass

    def decode(self, packet: H264Packet) -> Image:
        self.decoded_sequences.append(packet.seq)
        return Image(
            data=np.full(
                (packet.height, packet.width, packet.channels), packet.seq, dtype=np.uint8
            ),
            format=packet.format,
            frame_id=packet.frame_id,
            ts=packet.ts,
        )


@pytest.fixture(autouse=True)
def fake_h264_codec(monkeypatch: pytest.MonkeyPatch) -> None:
    FakeH264Decoder.decoded_sequences = []
    monkeypatch.setattr(h264_storage, "H264Encoder", FakeH264Encoder)
    monkeypatch.setattr(h264_storage, "H264Decoder", FakeH264Decoder)


def _raw_image(seq: int, fmt: ImageFormat = ImageFormat.RGB) -> Image:
    data = np.full((2, 2, 3), seq, dtype=np.uint8)
    if fmt == ImageFormat.GRAY:
        data = np.full((2, 2), seq, dtype=np.uint8)
    return Image.from_numpy(data, format=fmt, frame_id="cam", ts=float(seq))


def _emb(vec: list[float]) -> Embedding:
    vector = np.array(vec, dtype=np.float32)
    vector /= np.linalg.norm(vector) + 1e-10
    return Embedding(vector=vector)


def test_h264_image_codec_is_marker_only() -> None:
    codec = H264ImageCodec()

    with pytest.raises(RuntimeError, match="H264ImageBackend"):
        codec.encode(_raw_image(1))
    with pytest.raises(RuntimeError, match="H264ImageBackend"):
        codec.decode(b"packet")


def test_codec_id_and_factory_support_h264_for_image() -> None:
    codec = H264ImageCodec()

    assert codec_id(codec) == "h264"
    assert isinstance(codec_from_id("h264", "dimos.msgs.sensor_msgs.Image.Image"), H264ImageCodec)


def test_h264_stream_stores_raw_images_and_reads_decoded_images(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "h264.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264")
        source = stream._source
        assert isinstance(source, H264ImageBackend)
        stored = stream.append(_raw_image(0), ts=1.0)
        assert stored.data.frame_id == "cam"
        assert int(stored.data.data[0, 0, 0]) == 0

    with SqliteStore(path=str(db), must_exist=True) as reopened:
        stream = reopened.stream("cam", Image)
        obs = stream.first()
        assert obs.data.frame_id == "cam"
        assert obs.data.width == 2
        assert int(obs.data.data[0, 0, 0]) == 0


def test_h264_random_lazy_read_seeks_from_previous_keyframe(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "random.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264")
        stream.append(_raw_image(0), ts=0.0)
        stream.append(_raw_image(1), ts=1.0)
        stream.append(_raw_image(2), ts=2.0)

    with SqliteStore(path=str(db), must_exist=True) as reopened:
        obs = reopened.stream("cam", Image).order_by("ts", desc=True).to_list()[0]
        assert int(obs.data.data[0, 0, 0]) == 2
        assert FakeH264Decoder.decoded_sequences[-3:] == [0, 1, 2]


def test_h264_filter_predicates_run_after_lazy_decode(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "filters.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264")
        stream.append(_raw_image(0), ts=0.0)
        stream.append(_raw_image(1), ts=1.0)
        stream.append(_raw_image(2), ts=2.0)

        matches = stream.filter(lambda obs: int(obs.data.data[0, 0, 0]) == 2).to_list()

    assert len(matches) == 1
    assert int(matches[0].data.data[0, 0, 0]) == 2


def test_h264_vector_search_uses_vector_store(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "vector.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264")
        stream.append(_raw_image(0), ts=0.0, embedding=_emb([0, 1, 0]))
        stream.append(_raw_image(1), ts=1.0, embedding=_emb([1, 0, 0]))

        results = stream.search(_emb([1, 0, 0]), k=1).to_list()

    assert len(results) == 1
    assert int(results[0].data.data[0, 0, 0]) == 1
    assert results[0].similarity is not None


def test_h264_replay_emits_decoded_images(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    stream = store.stream("cam", Image, codec="h264")
    stream.append(_raw_image(0), ts=1.0)
    stream.append(_raw_image(1), ts=2.0)

    replayed = list(store.replay().streams.cam.iterate())

    assert [int(image.data[0, 0, 0]) for image in replayed] == [0, 1]
    assert all(isinstance(image.data, np.ndarray) for image in replayed)


def test_h264_sqlite_registry_persists_config(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    db = tmp_path / "config.db"
    config = H264Config(bitrate=123_456, keyframe_interval=7, max_gop_frames=9)
    with SqliteStore(path=str(db)) as store:
        stream = store.stream("cam", Image, codec="h264", h264_config=config)
        assert isinstance(stream._source, H264ImageBackend)
        assert stream._source.h264_config.bitrate == 123_456

    with SqliteStore(path=str(db), must_exist=True) as reopened:
        stream = reopened.stream("cam", Image)
        assert isinstance(stream._source, H264ImageBackend)
        assert stream._source.h264_config.bitrate == 123_456
        assert stream._source.h264_config.keyframe_interval == 7
        assert stream._source.h264_config.max_gop_frames == 9


def test_memory_store_rejects_h264_without_blob_store() -> None:
    with pytest.raises(RuntimeError, match="BlobStore"):
        MemoryStore().stream("cam", Image, codec="h264")


def test_default_image_stream_still_uses_jpeg_codec(tmp_path: Path) -> None:
    if _SKIP_SQLITE_VEC:
        pytest.skip("sqlite-vec extension not loadable here")
    store = SqliteStore(path=str(tmp_path / "jpeg.db"))
    stream = store.stream("rgb", Image)

    source = stream._source
    assert isinstance(source, Backend)
    assert isinstance(source.codec, JpegCodec)
