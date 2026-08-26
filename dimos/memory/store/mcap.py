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

"""Memory store backed by an mcap file.

Write mode (``mode="w"``) is what a Recorder uses: one channel per stream,
``message_encoding="dimos-obs"``, schema name = payload type path, schema data
= codec id. Each message is an envelope: 4-byte big-endian header length, JSON
header ``{"pose": [x,y,z,qx,qy,qz,qw] | null, "tags": {...}}``, codec payload.
No blobs, vectors, or embeddings.

Read mode decodes those channels with the recorded codec. Foreign channels
(e.g. Go2 DDS captures) need injected ``codecs`` (topic -> codec) and an
optional ``streams`` map (friendly stream name -> topic). See
``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 wiring.
Payloads decode lazily on ``obs.data``; counts come from the mcap index.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import replace
from functools import partial
import json
import os
import struct
import threading
from typing import Any, Literal, Protocol, runtime_checkable

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import Codec, codec_for, codec_from_id, codec_id, resolve_payload_type
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory.store.base import Store, StoreConfig
from dimos.memory.type.filter import StreamQuery
from dimos.memory.type.observation import Observation


@runtime_checkable
class StreamCodec(Protocol):
    """What the store needs to turn a channel's stored bytes into a payload."""

    @property
    def payload_type(self) -> type: ...

    def decode(self, data: bytes) -> Any: ...


class _BytesCodec:
    """Identity codec: hands back a codecless channel's stored bytes as ``Stream[bytes]``."""

    payload_type = bytes

    def decode(self, data: bytes) -> bytes:
        return data


_BYTES_CODEC = _BytesCodec()

DIMOS_ENCODING = "dimos-obs"
_HDR = struct.Struct(">I")


class _DimosCodec:
    """Adapts a recorded dimos codec (schema id + payload type) to :class:`StreamCodec`."""

    def __init__(self, codec: Codec[Any], payload_type: type) -> None:
        self.codec = codec
        self.payload_type = payload_type

    def decode(self, data: bytes) -> Any:
        return self.codec.decode(data[_HDR.size + _HDR.unpack_from(data)[0] :])


def _pack(obs: Observation[Any], payload: bytes) -> bytes:
    header = json.dumps({"pose": obs.pose_tuple, "tags": obs.tags}).encode()
    return _HDR.pack(len(header)) + header + payload


def _unpack_header(data: bytes) -> dict[str, Any]:
    n = _HDR.unpack_from(data)[0]
    return json.loads(data[_HDR.size : _HDR.size + n])  # type: ignore[no-any-return]


def _slug(topic: str) -> str:
    """Auto stream name from a topic: drop the ``rt/`` prefix and ``/`` -> ``_``.

    ``rt/`` is the ROS2-over-DDS topic prefix; ``removeprefix`` only strips it
    where present (e.g. app-level ``control_log`` is left alone).
    """
    return topic.removeprefix("rt/").replace("/", "_")


class McapObservationStoreConfig(ObservationStoreConfig):
    name: str = "<mcap>"


class McapObservationStore(ObservationStore[Any]):
    """Read-only metadata/query over one mcap channel. Payloads load lazily."""

    config: McapObservationStoreConfig

    def __init__(self, *, name: str, path: str, topic: str, codec: StreamCodec, count: int) -> None:
        super().__init__(name=name)
        self._path = path
        self._topic = topic
        self._codec = codec
        self._count = count

    @property
    def name(self) -> str:
        return self.config.name

    def _iter(self, reverse: bool = False) -> Iterator[Observation[Any]]:
        from mcap.reader import make_reader  # optional dep (go2/unitree extra)

        decode, dtype, n = self._codec.decode, self._codec.payload_type, self._count
        with open(self._path, "rb") as f:
            msgs = make_reader(f).iter_messages(topics=[self._topic], reverse=reverse)
            for i, (_s, ch, m) in enumerate(msgs):
                hdr = _unpack_header(m.data) if ch.message_encoding == DIMOS_ENCODING else {}
                pose = hdr.get("pose")
                yield Observation(
                    id=(n - 1 - i) if reverse else i,
                    ts=m.log_time / 1e9,
                    data_type=dtype,
                    pose_tuple=tuple(pose) if pose else None,
                    tags=hdr.get("tags"),
                    _loader=partial(decode, m.data),
                )

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        # mcap is natively log-time ordered (== ts == our id), so serve ts/id
        # ordering by iterating forward/reverse instead of materializing + sorting.
        if q.order_field in ("ts", "id"):
            it = self._iter(reverse=q.order_desc)
            q = replace(q, order_field=None, order_desc=False)
            return q.apply(it)
        return q.apply(self._iter())

    def count(self, q: StreamQuery) -> int:
        if not q.filters and q.search_text is None and q.search_vec is None:
            n = self._count
            if q.offset_val:
                n = max(0, n - q.offset_val)
            if q.limit_val is not None:
                n = min(n, q.limit_val)
            return n
        return sum(1 for _ in self.query(q))

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[Any]]:
        want = set(ids)
        return [o for o in self._iter() if o.id in want]

    def insert(self, obs: Observation[Any]) -> int:
        raise NotImplementedError("McapStore opened read-only")


class McapWriteObservationStore(ObservationStore[Any]):
    """Append-only channel writer. Encodes the payload itself (no blob store)."""

    config: McapObservationStoreConfig

    def __init__(
        self, *, name: str, writer: Any, channel_id: int, codec: Codec[Any], lock: threading.Lock
    ) -> None:
        super().__init__(name=name)
        self._writer = writer
        self._channel_id = channel_id
        self._codec = codec
        self._lock = lock
        self._n = 0

    @property
    def name(self) -> str:
        return self.config.name

    def insert(self, obs: Observation[Any]) -> int:
        data = _pack(obs, self._codec.encode(obs.data))
        t = int(obs.ts * 1e9)
        with self._lock:
            row_id = self._n
            self._n += 1
            self._writer.add_message(
                channel_id=self._channel_id, log_time=t, publish_time=t, data=data, sequence=row_id
            )
        return row_id

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        raise NotImplementedError("McapStore opened write-only; reopen the file to read")

    def count(self, q: StreamQuery) -> int:
        return self._n

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[Any]]:
        raise NotImplementedError("McapStore opened write-only; reopen the file to read")


class McapStoreConfig(StoreConfig):
    path: str = ""
    mode: Literal["r", "w"] = "r"


class McapStore(Store):
    """A memory store backed by an mcap file.

    Read mode exposes every channel in the file: dimos-recorded channels decode
    with their recorded codec; others need an injected codec. Names default to
    the slugified topic (see :func:`_slug`); ``streams`` (friendly name -> topic)
    overrides the name for specific topics.

    Write mode creates the file; each ``stream(name, payload_type)`` opens a channel.
    """

    config: McapStoreConfig

    def __init__(
        self,
        *,
        codecs: Mapping[str, StreamCodec] | None = None,
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._codecs: dict[str, StreamCodec] = dict(codecs or {})
        self._stream_topic: dict[str, str] = {}  # stream name -> topic
        self._available: dict[str, int] = {}  # stream name -> message count
        # Channels with no registered codec are still exposed, as Stream[bytes] via
        # _BYTES_CODEC — reachable but undecoded. _raw maps their stream name to the
        # source schema so summary() can flag them [raw bytes: <schema>].
        self._raw: dict[str, str | None] = {}  # raw stream name -> source schema
        self._writer: Any = None
        self._lock = threading.Lock()
        if self.config.mode == "w":
            self._open_writer()
        else:
            self._scan(streams)

    def _open_writer(self) -> None:
        from mcap.writer import Writer

        parent = os.path.dirname(self.config.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(self.config.path, "wb")
        self._writer = Writer(self._file)
        self._writer.start(profile="dimos", library="dimos")

    def _scan(self, streams: dict[str, str] | None) -> None:
        from mcap.reader import make_reader

        name_of = {topic: name for name, topic in (streams or {}).items()}  # topic -> override
        with open(self.config.path, "rb") as f:
            summary = make_reader(f).get_summary()
        if summary is None or summary.statistics is None:
            return
        for cid, ch in summary.channels.items():
            count = summary.statistics.channel_message_counts.get(cid, 0)
            name = name_of.get(ch.topic) or _slug(ch.topic)
            self._stream_topic[name] = ch.topic
            self._available[name] = count
            sch = summary.schemas.get(ch.schema_id)
            if ch.message_encoding == DIMOS_ENCODING and sch is not None:
                ptype = resolve_payload_type(sch.name)
                self._codecs[ch.topic] = _DimosCodec(
                    codec_from_id(sch.data.decode(), sch.name), ptype
                )
            elif ch.topic not in self._codecs:
                self._raw[name] = sch.name if sch else None

    def _create_write_backend(
        self, name: str, payload_type: type | None, raw_codec: Any
    ) -> Backend[Any]:
        if payload_type is None:
            raise TypeError(f"Stream {name!r}: payload_type is required to record")
        codec = self._resolve_codec(payload_type, raw_codec)
        module = f"{payload_type.__module__}.{payload_type.__qualname__}"
        with self._lock:
            schema_id = self._writer.register_schema(
                name=module, encoding="dimos", data=codec_id(codec).encode()
            )
            channel_id = self._writer.register_channel(
                topic=name, message_encoding=DIMOS_ENCODING, schema_id=schema_id
            )
        self._stream_topic[name] = name
        obs = McapWriteObservationStore(
            name=name, writer=self._writer, channel_id=channel_id, codec=codec, lock=self._lock
        )
        return Backend(
            metadata_store=obs,
            codec=codec,
            data_type=payload_type,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )

    def delete_stream(self, name: str) -> None:
        raise NotImplementedError("McapStore cannot delete streams; record to a new file")

    def list_streams(self) -> list[str]:
        return sorted(set(self._available) | set(self._streams))

    def stop(self) -> None:
        super().stop()
        if self._writer is not None:
            with self._lock:
                self._writer.finish()
                self._file.close()
                self._writer = None

    def summary(self) -> str:
        """Base summary, tagging codecless streams with ``[raw bytes: <schema>]``."""
        lines = []
        for name, stream in self.streams.items():
            line = stream.summary()  # "Stream(\"name\"): ..."
            if name in self._raw:
                head = str(stream)  # "Stream(\"name\")"
                line = f"{head} [raw bytes: {self._raw[name] or '?'}]{line[len(head) :]}"
            lines.append(line)
        return "\n".join(lines)

    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend[Any]:
        if self._writer is not None:
            return self._create_write_backend(name, payload_type, config.get("codec"))
        if name not in self._available:
            raise KeyError(f"No stream {name!r}. Available: {sorted(self._available)}")
        topic = self._stream_topic[name]
        codec = self._codecs.get(topic) or _BYTES_CODEC  # no codec -> Stream[bytes]
        ptype = codec.payload_type
        obs = McapObservationStore(
            name=name, path=self.config.path, topic=topic, codec=codec, count=self._available[name]
        )
        return Backend(
            metadata_store=obs,
            codec=codec_for(ptype),  # storage codec, unused (blob_store=None)
            data_type=ptype,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )
