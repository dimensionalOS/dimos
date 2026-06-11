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

from dataclasses import asdict, dataclass, replace
import sqlite3
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.pickle import PickleCodec
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.type.observation import _UNLOADED
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.VideoPacket import VideoPacket
from dimos.protocol.video.h264 import (
    H264CodecAdapter,
    H264Config,
    H264Decoder,
    H264Encoder,
    VideoDecodeGapError,
)

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.blobstore.base import BlobStore
    from dimos.memory2.notifier.base import Notifier
    from dimos.memory2.observationstore.base import ObservationStore
    from dimos.memory2.type.filter import StreamQuery
    from dimos.memory2.type.observation import Observation
    from dimos.memory2.vectorstore.base import VectorStore

T = TypeVar("T")


@dataclass(frozen=True)
class H264ImageStorageConfig:
    """Per-stream memory2 image storage mode for H.264-backed observations."""

    codec: H264Config = H264Config()
    mode: str = "h264"
    codec_adapter: H264CodecAdapter | None = None

    def serialize(self) -> dict[str, Any]:
        cfg = asdict(self.codec)
        cfg["supported_formats"] = [fmt.value for fmt in self.codec.supported_formats]
        return {"mode": self.mode, "codec": cfg}

    @classmethod
    def parse(cls, raw: H264ImageStorageConfig | dict[str, Any]) -> H264ImageStorageConfig:
        if isinstance(raw, cls):
            return raw
        if not isinstance(raw, dict):
            raise TypeError(f"Cannot parse H.264 image storage config from {type(raw).__name__}")
        mode = raw.get("mode", "h264")
        codec_raw = raw.get("codec", {})
        if isinstance(codec_raw, H264Config):
            codec = codec_raw
        else:
            codec_dict = dict(codec_raw)
            formats = codec_dict.get("supported_formats")
            if formats is not None:
                codec_dict["supported_formats"] = tuple(ImageFormat(fmt) for fmt in formats)
            codec = H264Config(**codec_dict)
        return cls(codec=codec, mode=mode)


@dataclass(frozen=True)
class H264FrameIndexRow:
    stream_name: str
    observation_id: int
    seq: int
    keyframe_observation_id: int
    is_keyframe: bool
    pts: int
    width: int
    height: int
    format: str
    codec: str
    bitstream: str


class H264FrameIndexStore:
    """Persistent GOP/keyframe index for H.264-backed image streams."""

    def __init__(self, conn: sqlite3.Connection) -> None:
        self._conn = conn

    def start(self) -> None:
        self._conn.execute(
            """
            CREATE TABLE IF NOT EXISTS h264_frames (
                stream_name TEXT NOT NULL,
                observation_id INTEGER NOT NULL,
                seq INTEGER NOT NULL,
                keyframe_observation_id INTEGER NOT NULL,
                is_keyframe INTEGER NOT NULL,
                pts INTEGER NOT NULL,
                width INTEGER NOT NULL,
                height INTEGER NOT NULL,
                format TEXT NOT NULL,
                codec TEXT NOT NULL,
                bitstream TEXT NOT NULL,
                PRIMARY KEY (stream_name, observation_id)
            )
            """
        )
        self._conn.execute(
            """
            CREATE INDEX IF NOT EXISTS idx_h264_frames_stream_keyframe
            ON h264_frames(stream_name, is_keyframe, observation_id)
            """
        )

    def stop(self) -> None:
        pass

    def insert(self, stream_name: str, observation_id: int, packet: VideoPacket) -> None:
        keyframe_observation_id = (
            observation_id
            if packet.is_keyframe
            else self._keyframe_observation_id(stream_name, packet.keyframe_seq)
        )
        self._conn.execute(
            """
            INSERT INTO h264_frames (
                stream_name, observation_id, seq, keyframe_observation_id, is_keyframe,
                pts, width, height, format, codec, bitstream
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                stream_name,
                observation_id,
                packet.seq,
                keyframe_observation_id,
                int(packet.is_keyframe),
                packet.pts,
                packet.width,
                packet.height,
                packet.format,
                packet.codec,
                packet.bitstream,
            ),
        )

    def packet_ids_for_decode(self, stream_name: str, observation_id: int) -> list[int]:
        row = self._conn.execute(
            """
            SELECT keyframe_observation_id FROM h264_frames
            WHERE stream_name = ? AND observation_id = ?
            """,
            (stream_name, observation_id),
        ).fetchone()
        if row is None:
            raise VideoDecodeGapError(f"No H.264 GOP index for observation {observation_id}")
        keyframe_id = int(row[0])
        rows = self._conn.execute(
            """
            SELECT observation_id FROM h264_frames
            WHERE stream_name = ? AND observation_id BETWEEN ? AND ?
            ORDER BY observation_id ASC
            """,
            (stream_name, keyframe_id, observation_id),
        ).fetchall()
        ids = [int(item[0]) for item in rows]
        if not ids or ids[0] != keyframe_id or ids[-1] != observation_id:
            raise VideoDecodeGapError(
                f"Incomplete H.264 GOP index for observation {observation_id}"
            )
        return ids

    def rows(self, stream_name: str) -> list[H264FrameIndexRow]:
        rows = self._conn.execute(
            """
            SELECT stream_name, observation_id, seq, keyframe_observation_id, is_keyframe,
                   pts, width, height, format, codec, bitstream
            FROM h264_frames WHERE stream_name = ? ORDER BY observation_id ASC
            """,
            (stream_name,),
        ).fetchall()
        return [
            H264FrameIndexRow(
                stream_name=row[0],
                observation_id=int(row[1]),
                seq=int(row[2]),
                keyframe_observation_id=int(row[3]),
                is_keyframe=bool(row[4]),
                pts=int(row[5]),
                width=int(row[6]),
                height=int(row[7]),
                format=row[8],
                codec=row[9],
                bitstream=row[10],
            )
            for row in rows
        ]

    def _keyframe_observation_id(self, stream_name: str, keyframe_seq: int) -> int:
        row = self._conn.execute(
            """
            SELECT observation_id FROM h264_frames
            WHERE stream_name = ? AND seq = ? AND is_keyframe = 1
            """,
            (stream_name, keyframe_seq),
        ).fetchone()
        if row is None:
            raise VideoDecodeGapError(f"No H.264 keyframe index for seq {keyframe_seq}")
        return int(row[0])


class H264ImageBackend(Backend[Image], Generic[T]):
    """memory2 backend that stores one H.264 packet blob per Image observation."""

    def __init__(
        self,
        *,
        metadata_store: ObservationStore[Image],
        blob_store: BlobStore,
        frame_index: H264FrameIndexStore,
        storage_config: H264ImageStorageConfig | None = None,
        vector_store: VectorStore | None = None,
        notifier: Notifier[Image] | None = None,
        eager_blobs: bool = False,
    ) -> None:
        self.storage_config = storage_config or H264ImageStorageConfig()
        self.frame_index = frame_index
        self._encoder = H264Encoder(
            self.storage_config.codec,
            codec=self.storage_config.codec_adapter,
        )
        super().__init__(
            metadata_store=metadata_store,
            codec=PickleCodec(),
            data_type=Image,
            blob_store=blob_store,
            vector_store=vector_store,
            notifier=notifier or SubjectNotifier(),
            eager_blobs=eager_blobs,
        )

    def start(self) -> None:
        super().start()
        self.frame_index.start()

    def _make_loader(self, row_id: int) -> Any:
        bs = self.blob_store
        if bs is None:
            raise RuntimeError("BlobStore required for H.264 image storage")
        name = self.name
        frame_index = self.frame_index
        storage_config = self.storage_config

        def loader() -> Image:
            packet_ids = frame_index.packet_ids_for_decode(name, row_id)
            decoder = H264Decoder(storage_config.codec, codec=storage_config.codec_adapter)
            decoded: Image | None = None
            for packet_id in packet_ids:
                packet = VideoPacket.lcm_decode(bs.get(name, packet_id))
                decoded = decoder.decode(packet)
            if decoded is None:
                raise VideoDecodeGapError(f"No H.264 packet available for observation {row_id}")
            return decoded

        return loader

    def append(self, obs: Observation[Image]) -> Observation[Image]:
        payload = obs.data
        if not isinstance(payload, Image):
            raise TypeError(f"Stream expects Image, got {type(payload).__qualname__}")
        obs.data_type = Image
        packet = self._encoder.encode(payload)
        encoded = packet.lcm_encode()
        try:
            row_id = self.metadata_store.insert(obs)
            obs.id = row_id
            assert self.blob_store is not None
            self.blob_store.put(self.name, row_id, encoded)
            self.frame_index.insert(self.name, row_id, packet)
            obs._data = _UNLOADED
            obs._loader = self._make_loader(row_id)
            if self.vector_store is not None:
                emb = getattr(obs, "embedding", None)
                if emb is not None:
                    self.vector_store.put(self.name, row_id, emb)
            if hasattr(self.metadata_store, "commit"):
                self.metadata_store.commit()
        except BaseException:
            if hasattr(self.metadata_store, "rollback"):
                self.metadata_store.rollback()
            raise
        self.notifier.notify(obs)
        return obs

    def _attach_loaders(self, it: Iterator[Observation[Image]]) -> Iterator[Observation[Image]]:
        for obs in it:
            obs.data_type = Image
            if obs._loader is None and isinstance(obs._data, type(_UNLOADED)):
                obs._loader = self._make_loader(obs.id)
            yield obs

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[Image]]:
        it = self._attach_loaders(self.metadata_store.query(query))
        if self.eager_blobs:
            for obs in it:
                _ = obs.data
                yield obs
        else:
            yield from it

    def serialize(self) -> dict[str, Any]:
        cfg = super().serialize()
        cfg["codec_id"] = "h264"
        cfg["image_storage"] = self.storage_config.serialize()
        return cfg


def storage_config_from_any(raw: Any) -> H264ImageStorageConfig | None:
    if raw is None:
        return None
    config = H264ImageStorageConfig.parse(raw)
    if config.mode != "h264":
        return None
    return config


def storage_config_with_adapter(
    config: H264ImageStorageConfig,
    adapter: H264CodecAdapter | None,
) -> H264ImageStorageConfig:
    return replace(config, codec_adapter=adapter)


__all__ = [
    "H264FrameIndexRow",
    "H264FrameIndexStore",
    "H264ImageBackend",
    "H264ImageStorageConfig",
    "storage_config_from_any",
    "storage_config_with_adapter",
]
