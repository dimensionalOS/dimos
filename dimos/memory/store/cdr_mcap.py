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

"""Read-only CDR MCAP storage using embedded schemas and installed generated types."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import replace
from functools import partial
from typing import Any, Protocol, runtime_checkable

from dimos_cdrgen.registry import message_types, schema as cdr_schema

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import codec_for
from dimos.memory.codecs.cdr import CdrCodec
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


def _slug(topic: str) -> str:
    return topic.removeprefix("rt/").lstrip("/").replace("/", "_")


class CdrMcapObservationStoreConfig(ObservationStoreConfig):
    name: str = "<mcap>"


class CdrMcapObservationStore(ObservationStore[Any]):
    """Read-only metadata/query over one mcap channel. Payloads load lazily."""

    config: CdrMcapObservationStoreConfig

    def __init__(
        self,
        *,
        name: str,
        path: str,
        topic: str,
        codec: StreamCodec,
        count: int,
        observation_uses_publish_time: bool,
    ) -> None:
        super().__init__(name=name)
        self._path = path
        self._topic = topic
        self._codec = codec
        self._count = count
        # Immutable channel metadata: this store never writes and each iterator
        # owns its own file reader, so timestamp selection has no async state.
        self._observation_uses_publish_time = observation_uses_publish_time

    @property
    def name(self) -> str:
        return self.config.name

    def _iter(self, reverse: bool = False) -> Iterator[Observation[Any]]:
        from mcap.reader import make_reader  # optional mcap dependency

        decode, dtype, n = self._codec.decode, self._codec.payload_type, self._count
        with open(self._path, "rb") as f:
            msgs = make_reader(f).iter_messages(topics=[self._topic], reverse=reverse)
            for i, (_schema, _channel, message) in enumerate(msgs):
                observation_time = (
                    message.publish_time
                    if self._observation_uses_publish_time
                    else message.log_time
                )
                yield Observation(
                    id=(n - 1 - i) if reverse else i,
                    ts=observation_time / 1e9,
                    data_type=dtype,
                    _loader=partial(decode, message.data),
                )

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        # MCAP is natively log-time ordered, so id ordering never needs a sort.
        # Native DimOS recordings expose publish_time as observation ts; source
        # time can differ from log/reception order and must use the generic sort.
        if q.order_field == "id" or (
            q.order_field == "ts" and not self._observation_uses_publish_time
        ):
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
        raise NotImplementedError("CdrMcapStore is read-only")


class CdrMcapStoreConfig(StoreConfig):
    path: str = ""


class CdrMcapStore(Store):
    """A memory store backed by an mcap file (read-only).

    Only CDR channels are accepted. Unknown schemas are exposed as raw bytes. Names default to
    the slugified topic (see :func:`_slug`); ``streams`` (friendly name -> topic)
    overrides the name for specific topics.
    """

    config: CdrMcapStoreConfig

    def __init__(
        self,
        *,
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        from mcap.reader import make_reader  # optional mcap dependency

        super().__init__(**kwargs)
        self._codecs: dict[str, StreamCodec] = {}
        name_of = {topic: name for name, topic in (streams or {}).items()}  # topic -> override
        with open(self.config.path, "rb") as f:
            summary = make_reader(f).get_summary()
        self._stream_topic: dict[str, str] = {}  # stream name -> topic
        self._available: dict[str, int] = {}  # stream name -> message count
        self._observation_uses_publish_time: dict[str, bool] = {}
        # Channels with no registered codec are still exposed, as Stream[bytes] via
        # _BYTES_CODEC — reachable but undecoded. _raw maps their stream name to the
        # source schema so summary() can flag them [raw bytes: <schema>].
        self._raw: dict[str, str | None] = {}  # raw stream name -> source schema
        channel_signatures: dict[str, tuple[Any, ...]] = {}
        if summary is not None and summary.statistics is not None:
            for cid, ch in summary.channels.items():
                if ch.message_encoding != "cdr":
                    raise ValueError(f"CdrMcapStore requires CDR channels: {ch.topic!r}")
                declared = summary.schemas.get(ch.schema_id)
                signature = (
                    ch.message_encoding,
                    (declared.name, declared.encoding, declared.data) if declared else None,
                    ch.metadata.get("dimos.observation_time"),
                )
                if ch.topic in channel_signatures and channel_signatures[ch.topic] != signature:
                    raise ValueError(
                        f"Conflicting schemas or timing policies for topic {ch.topic!r}"
                    )
                channel_signatures[ch.topic] = signature
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                taken = self._stream_topic.get(name)
                if taken is not None and taken != ch.topic:
                    raise ValueError(
                        f"stream {name!r} would name both {taken!r} and {ch.topic!r};"
                        " pass streams= to name them apart"
                    )
                if ch.topic not in self._codecs:
                    # Unknown schemas remain available as raw bytes.
                    self._codecs[ch.topic] = _BYTES_CODEC
                    self._raw[name] = declared.name if declared else None
                    if declared is not None and declared.encoding == "ros2msg":
                        kind = message_types().get(declared.name)
                        if kind is not None:
                            if declared.data.decode("utf-8") != cdr_schema(declared.name):
                                raise ValueError(f"CDR schema mismatch: {declared.name}")
                            self._codecs[ch.topic] = CdrCodec(kind)
                            self._raw.pop(name, None)
                self._stream_topic[name] = ch.topic
                self._available[name] = self._available.get(name, 0) + count
                self._observation_uses_publish_time[name] = (
                    ch.metadata.get("dimos.observation_time") == "publish_time"
                )

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
        codec = self._codecs[topic]
        ptype = codec.payload_type
        obs = CdrMcapObservationStore(
            name=name,
            path=self.config.path,
            topic=topic,
            codec=codec,
            count=self._available[name],
            observation_uses_publish_time=self._observation_uses_publish_time[name],
        )
        return Backend(
            metadata_store=obs,
            codec=codec_for(ptype),  # storage codec, unused (blob_store=None)
            data_type=ptype,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )
