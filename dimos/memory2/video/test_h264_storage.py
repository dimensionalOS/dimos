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

import sqlite3

import numpy as np
import pytest

from dimos.memory2.blobstore.sqlite import SqliteBlobStore
from dimos.memory2.codecs.pickle import PickleCodec
from dimos.memory2.observationstore.sqlite import SqliteObservationStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import _UNLOADED
from dimos.memory2.video.h264 import (
    H264FrameIndexStore,
    H264ImageBackend,
    H264ImageStorageConfig,
    storage_config_from_any,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.VideoPacket import VideoPacket
from dimos.protocol.video.h264 import UnsupportedVideoImageError, VideoDecodeGapError


class FakeH264CodecAdapter:
    def encode_image(self, image: Image, *, force_keyframe: bool) -> tuple[bytes, int]:
        return image.data.tobytes(), int(image.ts * 1000)

    def decode_packet(self, packet: VideoPacket) -> Image:
        channels = 1 if packet.format == ImageFormat.GRAY.value else 3
        shape = (
            (packet.height, packet.width)
            if channels == 1
            else (packet.height, packet.width, channels)
        )
        arr = np.frombuffer(packet.data, dtype=np.uint8).copy().reshape(shape)
        return Image.from_numpy(
            arr, format=ImageFormat(packet.format), frame_id=packet.frame_id, ts=packet.ts
        )


def _image(seq: int, fmt: ImageFormat = ImageFormat.RGB) -> Image:
    data = np.full((2, 2, 3), seq, dtype=np.uint8)
    if fmt == ImageFormat.GRAY:
        data = np.full((2, 2), seq, dtype=np.uint8)
    return Image.from_numpy(data, format=fmt, frame_id="cam", ts=float(seq))


def _make_backend(
    conn: sqlite3.Connection, *, config: H264ImageStorageConfig | None = None
) -> H264ImageBackend:
    frame_index = H264FrameIndexStore(conn)
    blob_store = SqliteBlobStore(conn=conn)
    obs_store = SqliteObservationStore(
        conn=conn, name="cam", codec=PickleCodec(), blob_store_conn_match=False, page_size=256
    )
    backend = H264ImageBackend(
        metadata_store=obs_store,
        blob_store=blob_store,
        frame_index=frame_index,
        storage_config=config or H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter()),
    )
    backend.start()
    return backend


def test_storage_config_parse_and_serialize() -> None:
    config = H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
    raw = config.serialize()
    parsed = H264ImageStorageConfig.parse(raw)
    assert parsed.mode == "h264"
    assert parsed.codec == config.codec
    assert storage_config_from_any(raw) == H264ImageStorageConfig(codec=config.codec)
    assert storage_config_from_any({"mode": "jpeg", "codec": raw["codec"]}) is None


def test_store_creates_h264_backend_from_config(tmp_path) -> None:
    store = SqliteStore(path=str(tmp_path / "h264.db"))
    backend = store._create_backend(
        "cam",
        Image,
        image_storage=H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter()),
    )
    assert isinstance(backend, H264ImageBackend)
    assert backend.storage_config.mode == "h264"
    assert isinstance(backend.storage_config.codec_adapter, FakeH264CodecAdapter)


def test_h264_image_stream_keeps_default_jpeg_compatibility(tmp_path) -> None:
    store = SqliteStore(path=str(tmp_path / "jpeg.db"))
    stream = store.stream("rgb", Image)
    obs = stream.append(_image(1))
    assert obs.data.format == ImageFormat.RGB
    assert store.stream("rgb").count() == 1


def test_h264_one_observation_and_one_blob_per_frame(tmp_path) -> None:
    conn = sqlite3.connect(str(tmp_path / "frames.db"))
    backend = _make_backend(conn)
    from dimos.memory2.type.observation import Observation

    stored = backend.append(Observation(data_type=Image, _data=_image(1)))
    assert stored.id == 1
    assert backend.blob_store is not None
    assert backend.blob_store.get("cam", 1)
    assert len(backend.frame_index.rows("cam")) == 1


def test_h264_persistent_gop_index_and_lazy_decode(tmp_path) -> None:
    db = tmp_path / "gop.db"
    with SqliteStore(path=str(db)) as store:
        stream = store.stream(
            "cam", Image, image_storage=H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
        )
        stream.append(_image(1), ts=1.0)
        stream.append(_image(2), ts=2.0)
        obs = list(stream)[1]
        assert obs._loader is not None
        assert obs._data is _UNLOADED
        assert obs.id == 2
        assert obs.ts == 2.0
        assert obs.data.data.shape == (2, 2, 3)
        backend = stream._source
        assert isinstance(backend, H264ImageBackend)
        assert len(backend.frame_index.rows("cam")) == 2

    with SqliteStore(path=str(db), must_exist=True) as reopened:
        stream = reopened.stream("cam", Image)
        assert stream.count() == 2
        backend = stream._source
        assert isinstance(backend, H264ImageBackend)
        assert backend.storage_config.mode == "h264"
        backend.storage_config = H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
        assert reopened.streams.cam.first().data.data.shape == (2, 2, 3)


def test_h264_mid_gop_decode_and_missing_gop_failure(tmp_path) -> None:
    store = SqliteStore(path=str(tmp_path / "gap.db"))
    stream = store.stream(
        "cam", Image, image_storage=H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
    )
    stream.append(_image(1))
    stream.append(_image(2))
    stream.append(_image(3))
    obs = list(stream)[2]
    assert obs.data.data[0, 0, 0] == 3

    backend = stream._source
    assert isinstance(backend, H264ImageBackend)
    backend.frame_index._conn.execute("DELETE FROM h264_frames WHERE observation_id = 2")
    gap_obs = list(stream)[1]
    with pytest.raises(VideoDecodeGapError):
        _ = gap_obs.data


def test_replay_iterate_returns_decoded_images(tmp_path) -> None:
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    stream = store.stream(
        "cam", Image, image_storage=H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
    )
    stream.append(_image(1), ts=1.0)
    stream.append(_image(2), ts=2.0)

    replay = store.replay()
    images = list(replay.streams.cam.iterate())
    assert [img.ts for img in images] == [1.0, 2.0]
    assert [img.data[0, 0, 0] for img in images] == [1, 2]


def test_h264_rejects_unsupported_formats(tmp_path) -> None:
    store = SqliteStore(path=str(tmp_path / "bad.db"))
    stream = store.stream(
        "cam", Image, image_storage=H264ImageStorageConfig(codec_adapter=FakeH264CodecAdapter())
    )
    rgba = np.zeros((2, 2, 4), dtype=np.uint8)
    with pytest.raises(UnsupportedVideoImageError):
        stream.append(Image.from_numpy(rgba, format=ImageFormat.RGBA))
