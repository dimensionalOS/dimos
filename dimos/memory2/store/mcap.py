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

"""Read-only memory2 store backed by an mcap file.

Generic and codec-injected — it knows nothing about any robot. The caller
supplies ``codecs`` (DDS/wire topic -> codec that decodes a message's stored
bytes) and an optional ``streams`` map (friendly stream name -> topic). See
``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 wiring.

Read-only: no append, blobs, vectors, or embeddings. Payloads decode lazily on
``obs.data``; ts and counts are cheap (counts come from the mcap index).
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import replace
from functools import partial
from typing import Any, Protocol, runtime_checkable

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_for
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory2.store.base import Store, StoreConfig
from dimos.memory2.type.filter import StreamQuery
from dimos.memory2.type.observation import Observation


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
    """A memory2 store backed by an mcap file (read-only).

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
        if summary is not None and summary.statistics is not None:
            for cid, ch in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                self._stream_topic[name] = ch.topic
                self._available[name] = count
                if ch.topic not in self._codecs:
                    sch = summary.schemas.get(ch.schema_id)
                    self._raw[name] = sch.name if sch else None

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
