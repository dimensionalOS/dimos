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

"""Memory stores backed by an mcap file: :class:`McapStore` reads, :class:`McapWriteStore` writes.

:class:`McapStore` is generic and codec-injected — it knows nothing about any
robot. The caller supplies ``codecs`` (DDS/wire topic -> codec that decodes a
message's stored bytes) and an optional ``streams`` map (friendly stream name ->
topic). See ``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 wiring.
Channels with no supplied codec whose schema name is a dimos message import path
(what :class:`McapWriteStore` writes) decode through that type's LCM codec.

Read-only: no append, blobs, vectors, or embeddings. Payloads decode lazily on
``obs.data``; ts and counts are cheap (counts come from the mcap index).

:class:`McapWriteStore` is the mirror image — append-only, one channel per
stream, payloads LCM-encoded — and is what ``dimos run --record mcap`` writes.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import replace
from functools import partial
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import Codec, codec_for, resolve_payload_type
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory.registry import qual
from dimos.memory.store.base import Store, StoreConfig
from dimos.memory.type.filter import StreamQuery
from dimos.memory.type.observation import Observation

# mcap message encoding for a payload written with ``lcm_encode``.
LCM_MESSAGE_ENCODING = "lcm"

NANOSECONDS_PER_SECOND = 1e9


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


class _DimosMsgCodec:
    """Decodes a channel whose schema name is a dimos message import path."""

    def __init__(self, payload_type: type) -> None:
        self.payload_type = payload_type

    def decode(self, data: bytes) -> Any:
        return self.payload_type.lcm_decode(data)  # type: ignore[attr-defined,no-any-return]


def _schema_codec(schema_name: str | None) -> StreamCodec | None:
    """Codec for a schema name written by :class:`McapWriteStore`, else ``None``."""
    if not schema_name:
        return None
    try:
        payload_type = resolve_payload_type(schema_name)
    except (ImportError, AttributeError, ValueError):
        return None
    return _DimosMsgCodec(payload_type) if hasattr(payload_type, "lcm_decode") else None


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
            for i, (_s, _c, m) in enumerate(msgs):
                yield Observation(
                    id=(n - 1 - i) if reverse else i,
                    ts=m.log_time / 1e9,
                    data_type=dtype,
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
        raise NotImplementedError("McapStore is read-only")


class McapStoreConfig(StoreConfig):
    path: str = ""


class McapStore(Store):
    """A memory store backed by an mcap file (read-only).

    Every channel present in the file with a codec is exposed. Names default to
    the slugified topic (see :func:`_slug`); ``streams`` (friendly name -> topic)
    overrides the name for specific topics.
    """

    config: McapStoreConfig

    def __init__(
        self,
        *,
        codecs: Mapping[str, StreamCodec],
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        from mcap.reader import make_reader  # optional dep (go2/unitree extra)

        super().__init__(**kwargs)
        self._codecs = codecs
        name_of = {topic: name for name, topic in (streams or {}).items()}  # topic -> override
        with open(self.config.path, "rb") as f:
            summary = make_reader(f).get_summary()
        self._stream_topic: dict[str, str] = {}  # stream name -> topic
        self._available: dict[str, int] = {}  # stream name -> message count
        # Channels with no registered codec are still exposed, as Stream[bytes] via
        # _BYTES_CODEC — reachable but undecoded. _raw maps their stream name to the
        # source schema so summary() can flag them [raw bytes: <schema>].
        self._raw: dict[str, str | None] = {}  # raw stream name -> source schema
        self._schema_codecs: dict[str, StreamCodec] = {}  # stream name -> self-describing codec
        if summary is not None and summary.statistics is not None:
            for cid, ch in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                self._stream_topic[name] = ch.topic
                self._available[name] = count
                if ch.topic not in self._codecs:
                    sch = summary.schemas.get(ch.schema_id)
                    schema_name = sch.name if sch else None
                    codec = _schema_codec(schema_name)
                    if codec is not None:
                        self._schema_codecs[name] = codec
                    else:
                        self._raw[name] = schema_name

    def list_streams(self) -> list[str]:
        return sorted(set(self._available) | set(self._streams))

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
        if name not in self._available:
            raise KeyError(f"No stream {name!r}. Available: {sorted(self._available)}")
        topic = self._stream_topic[name]
        # supplied codec > codec named by the schema > raw Stream[bytes]
        codec = self._codecs.get(topic) or self._schema_codecs.get(name) or _BYTES_CODEC
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


class McapWriteObservationStore(ObservationStore[Any]):
    """Write-only view of one mcap channel: every insert becomes one message."""

    config: McapObservationStoreConfig

    def __init__(self, *, name: str, writer: Any, channel_id: int, codec: Codec[Any]) -> None:
        super().__init__(name=name)
        self._writer = writer
        self._channel_id = channel_id
        self._codec = codec
        self._count = 0

    @property
    def name(self) -> str:
        return self.config.name

    def insert(self, obs: Observation[Any]) -> int:
        log_time = int(obs.ts * NANOSECONDS_PER_SECOND)
        row_id = self._count
        self._writer.add_message(
            channel_id=self._channel_id,
            log_time=log_time,
            publish_time=log_time,
            sequence=row_id,
            data=self._codec.encode(obs.data),
        )
        self._count += 1
        return row_id

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        raise NotImplementedError("McapWriteStore is write-only — reopen with McapStore to read")

    def count(self, q: StreamQuery) -> int:
        raise NotImplementedError("McapWriteStore is write-only — reopen with McapStore to read")

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[Any]]:
        raise NotImplementedError("McapWriteStore is write-only — reopen with McapStore to read")


class McapWriteStoreConfig(StoreConfig):
    path: str = ""


class McapWriteStore(Store):
    """Append-only store writing one mcap file — a channel per stream, payloads LCM-encoded.

    Self-describing: each channel's schema name is its payload type's import
    path, so :class:`McapStore` decodes the file back without a codec map.
    Not thread-safe; the mcap writer takes appends from one thread at a time.
    """

    config: McapWriteStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        from mcap.writer import Writer  # optional dep (`pip install mcap`)

        super().__init__(**kwargs)
        Path(self.config.path).parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.config.path, "wb")
        self._writer = Writer(self._file)
        self._writer.start()

    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend[Any]:
        if payload_type is None or not hasattr(payload_type, "lcm_encode"):
            raise TypeError(f"Stream {name!r}: McapWriteStore needs a dimos message type")
        codec = self._resolve_codec(payload_type, "lcm")
        schema_id = self._writer.register_schema(name=qual(payload_type), encoding="", data=b"")
        channel_id = self._writer.register_channel(
            topic=name, message_encoding=LCM_MESSAGE_ENCODING, schema_id=schema_id
        )
        obs = McapWriteObservationStore(
            name=name, writer=self._writer, channel_id=channel_id, codec=codec
        )
        return Backend(
            metadata_store=obs,
            codec=codec,
            data_type=payload_type,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )

    def stop(self) -> None:
        super().stop()
        self._writer.finish()
        self._file.close()
