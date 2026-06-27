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

from collections.abc import Iterator
from dataclasses import replace
import threading
from typing import Any

from dimos.memory2.backend import Backend
from dimos.memory2.type.filter import StreamQuery
from dimos.memory2.type.observation import _UNLOADED, Observation
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.video.h264 import H264Config, H264Decoder, H264Encoder, H264Packet

_TAG_CODEC = "_dimos_codec"
_TAG_IS_KEYFRAME = "_h264_is_keyframe"
_TAG_SEQ = "_h264_seq"
_TAG_KEYFRAME_SEQ = "_h264_keyframe_seq"
_TAG_PTS = "_h264_pts"
_TAG_FORMAT = "_h264_format"
_TAG_WIDTH = "_h264_width"
_TAG_HEIGHT = "_h264_height"
_TAG_CHANNELS = "_h264_channels"
_TAG_DTYPE = "_h264_dtype"


class H264ImageCodec:
    """Marker codec id for opt-in H.264-backed logical Image streams.

    Normal memory2 codecs are stateless per-observation codecs. H.264 is a
    stateful stream codec, so actual encoding/decoding is implemented by
    :class:`H264ImageBackend`. This marker exists so stream registry persistence
    can round-trip ``codec_id == "h264"``.
    """

    CODEC_ID = "h264"

    def encode(self, value: Image) -> bytes:
        raise RuntimeError("H264ImageCodec must be used through H264ImageBackend")

    def decode(self, data: bytes) -> Image:
        raise RuntimeError("H264ImageCodec must be used through H264ImageBackend")


class H264ImageBackend(Backend[Image]):
    """Backend for logical ``Image`` streams physically stored as H.264 packets."""

    def __init__(self, *, h264_config: H264Config | None = None, **kwargs: Any) -> None:
        kwargs.pop("codec", None)
        kwargs.pop("data_type", None)
        kwargs.pop("eager_blobs", None)
        super().__init__(codec=H264ImageCodec(), data_type=Image, eager_blobs=False, **kwargs)
        if self.blob_store is None:
            raise RuntimeError("BlobStore required for H.264 image storage")
        self.h264_config = h264_config or H264Config()
        self._append_lock = threading.Lock()
        self._encoder: H264Encoder | None = None
        self._force_next_keyframe = True

    def _new_encoder(self) -> H264Encoder:
        self._encoder = H264Encoder(self.h264_config)
        self._force_next_keyframe = True
        return self._encoder

    def _packet_tags(self, packet: H264Packet) -> dict[str, Any]:
        return {
            _TAG_CODEC: "h264",
            _TAG_IS_KEYFRAME: packet.is_keyframe,
            _TAG_SEQ: packet.seq,
            _TAG_KEYFRAME_SEQ: packet.keyframe_seq,
            _TAG_PTS: packet.pts,
            _TAG_FORMAT: packet.format.value,
            _TAG_WIDTH: packet.width,
            _TAG_HEIGHT: packet.height,
            _TAG_CHANNELS: packet.channels,
            _TAG_DTYPE: packet.dtype,
        }

    def append(self, obs: Observation[Image]) -> Observation[Image]:
        payload = obs.data
        if not isinstance(payload, Image):
            raise TypeError(f"Stream expects Image, got {type(payload).__qualname__}")
        obs.data_type = Image

        with self._append_lock:
            encoder = self._encoder or self._new_encoder()
            try:
                packet = encoder.encode(payload, force_keyframe=self._force_next_keyframe)
                self._force_next_keyframe = False
                encoded = packet.to_bytes()
                obs.tags = {**obs.tags, **self._packet_tags(packet)}

                row_id = self.metadata_store.insert(obs)
                obs.id = row_id

                if self.blob_store is None:
                    raise RuntimeError("BlobStore required for H.264 image storage")
                self.blob_store.put(self.name, row_id, encoded)
                obs._data = _UNLOADED
                obs._loader = self._make_loader(row_id)

                if self.vector_store is not None:
                    emb = getattr(obs, "embedding", None)
                    if emb is not None:
                        self.vector_store.put(self.name, row_id, emb)

                if hasattr(self.metadata_store, "commit"):
                    self.metadata_store.commit()
            except BaseException:
                self._encoder = None
                self._force_next_keyframe = True
                if hasattr(self.metadata_store, "rollback"):
                    self.metadata_store.rollback()
                raise

        self.notifier.notify(obs)
        return obs

    def _make_loader(self, row_id: int) -> Any:
        def loader() -> Image:
            return self._decode_at(row_id)

        return loader

    def _metadata_rows_by_id(self) -> list[Observation[Image]]:
        rows = list(self.metadata_store.query(StreamQuery(order_field="id")))
        for obs in rows:
            obs.data_type = Image
        return rows

    def _decode_chain_rows(self, target_id: int) -> list[Observation[Image]]:
        rows = [obs for obs in self._metadata_rows_by_id() if obs.id <= target_id]
        keyframes = [obs for obs in rows if obs.tags.get(_TAG_IS_KEYFRAME) is True]
        if not keyframes:
            raise RuntimeError(f"No H.264 keyframe available before observation id={target_id}")
        start_id = keyframes[-1].id
        chain = [obs for obs in rows if start_id <= obs.id <= target_id]
        if not chain or chain[-1].id != target_id:
            raise KeyError(f"No H.264 observation id={target_id}")
        return chain

    def _decode_at(self, target_id: int) -> Image:
        if self.blob_store is None:
            raise RuntimeError("BlobStore required for H.264 image storage")
        decoder = H264Decoder(self.h264_config)
        decoded: Image | None = None
        for obs in self._decode_chain_rows(target_id):
            packet = H264Packet.from_bytes(self.blob_store.get(self.name, obs.id))
            decoded = decoder.decode(packet)
            if obs.id == target_id:
                return decoded
        raise KeyError(f"No H.264 observation id={target_id}")

    def _decode_contiguous_id_order(
        self, rows: list[Observation[Image]]
    ) -> Iterator[Observation[Image]]:
        if self.blob_store is None:
            raise RuntimeError("BlobStore required for H.264 image storage")
        decoder = H264Decoder(self.h264_config)
        expected_id: int | None = None
        for obs in rows:
            if expected_id is not None and obs.id != expected_id:
                # Fall back to correct random keyframe seek when a query skips ids.
                obs._data = self._decode_at(obs.id)
                obs._loader = None
                expected_id = obs.id + 1
                yield obs
                continue
            packet = H264Packet.from_bytes(self.blob_store.get(self.name, obs.id))
            obs._data = decoder.decode(packet)
            obs._loader = None
            obs.data_type = Image
            expected_id = obs.id + 1
            yield obs

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[Image]]:
        # Common efficient path: whole-stream or id-ordered contiguous iteration.
        can_decode_sequentially = (
            query.search_vec is None
            and not query.filters
            and query.order_field in (None, "id")
            and not query.order_desc
            and query.offset_val in (None, 0)
        )
        if can_decode_sequentially:
            id_query = replace(query, order_field="id", order_desc=False)
            rows = list(self.metadata_store.query(id_query))
            yield from self._decode_contiguous_id_order(rows)
            return

        # Arbitrary filtered/ts-ordered/vector queries stay correct by using the
        # base backend path. It preserves vector search and SQLite Python
        # post-filters, while this class's _make_loader still provides H.264
        # keyframe-seeking lazy reads.
        yield from super()._iterate_snapshot(query)


def is_h264_backend_marker(codec: Any) -> bool:
    return getattr(codec, "CODEC_ID", None) == "h264"


__all__ = ["H264ImageBackend", "H264ImageCodec", "is_h264_backend_marker"]
