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

Generic and robot-independent. JPEG channels decode automatically because their
payload type is fixed. Other formats use a caller-supplied ``codecs`` map (wire
topic -> codec), while ``streams`` may map friendly stream names to topics. See
``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 DDS wiring.

The channels a RECORDER wrote are read-only: their messages are spread through
chunks all over the file, and appending more would leave a channel out of time
order. New streams are not -- ``store.stream("voxel_keyframe", PointCloud2)``
creates a channel of dimos's own and appends to the recording in place, so
everything built from a recording lives in it rather than in a companion
database beside it. See :mod:`dimos.memory.store.mcap_derived` for the envelope
those streams use and :mod:`dimos.memory.store.mcap_append` for the mechanics.

Payloads decode lazily on ``obs.data``; ts and counts are cheap (counts come
from the mcap index).

One caveat, the same one ``dtk mcap_edit`` carries: a flush rewrites the file's
summary over where the old one sat, with no lock and no way for a reader to
notice, so a reader that is part-way through the index while a flush lands can
read a torn file. Within this process that window is small -- a flush happens
once per chunk written, not once per observation -- and a reader that opens the
file after the flush sees a consistent one. A viewer in another process that is
up while a recording is being written to is on its own.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
from dataclasses import replace
from functools import partial
import json
import threading
from typing import Any, Protocol, runtime_checkable

import numpy as np

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import codec_for, codec_id
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory.store.base import Store, StoreConfig
from dimos.memory.store.mcap_append import McapAppender, read_metadata
from dimos.memory.store.mcap_derived import (
    ENVELOPE_ENCODING,
    REGISTRY_METADATA,
    McapVectorStore,
    channel_spec,
    decode_envelope,
    iter_vectors,
    topic_for,
    write_observation,
)
from dimos.memory.type.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    StreamQuery,
    TimeRangeFilter,
)
from dimos.memory.type.observation import Observation, PoseTuple

# A flush rewrites the whole summary -- 7 MB on a 75 GB recording -- so a stream being
# written flushes per 64 MiB of observations rather than per 4 MiB chunk.
FLUSH_BYTES = 64 << 20


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
    """Metadata/query over one mcap channel. Payloads load lazily.

    Read-only over a channel a recorder wrote. Over a channel dimos wrote --
    one carrying :data:`~dimos.memory.store.mcap_derived.ENVELOPE_ENCODING`
    messages, which is what an *appender* means here -- ``insert`` appends to
    it, and pose, tags and embedding come back out of the envelope on read.
    """

    config: McapObservationStoreConfig

    def __init__(
        self,
        *,
        name: str,
        path: str,
        topic: str,
        codec: StreamCodec,
        count: int,
        observation_uses_publish_time: bool,
        appender: Callable[[], McapAppender] | None = None,
        channel_id: int | None = None,
        enveloped: bool = False,
        payload_type: type | None = None,
    ) -> None:
        super().__init__(name=name)
        self._path = path
        self._topic = topic
        self._codec = codec
        # A read-only channel's codec is a StreamCodec and knows its own payload
        # type; a dimos-written one is handed a real dimos Codec, which does not,
        # so the type comes from the registry alongside it.
        self._payload_type = payload_type or getattr(codec, "payload_type", object)
        self._count = count
        # Immutable channel metadata: each iterator owns its own file reader, so
        # timestamp selection has no async state.
        self._observation_uses_publish_time = observation_uses_publish_time
        # Opened on the first insert, not here: opening one takes the file's write lock,
        # and a process that only READS a dimos stream must not hold it.
        self._appender_of = appender
        self._appender: McapAppender | None = None
        self._channel_id = channel_id
        self._enveloped = enveloped
        # Inserted but not yet flushed. A flush rewrites the file's summary, so
        # doing one per observation would rewrite megabytes per observation on a
        # long recording; instead the envelopes are held here, where a read finds
        # them, until enough have piled up to be worth a flush.
        self._pending: list[tuple[float, bytes]] = []
        self._pending_bytes = 0

    @property
    def name(self) -> str:
        return self.config.name

    def _unwrap(self, data: bytes) -> tuple[bytes, PoseTuple | None, dict[str, Any]]:
        """Payload, pose and tags of one message. Plain bytes outside an envelope."""
        if not self._enveloped:
            return data, None, {}
        header, payload, _vector = decode_envelope(data)
        pose = header.get("pose")
        return payload, (tuple(pose) if pose else None), header.get("tags") or {}

    def _iter(
        self,
        reverse: bool = False,
        start_ns: int | None = None,
        end_ns: int | None = None,
    ) -> Iterator[Observation[Any]]:
        from mcap.reader import make_reader  # optional mcap dependency

        decode, dtype, n = self._codec.decode, self._payload_type, self._count
        with open(self._path, "rb") as f:
            reader = make_reader(f)
            if start_ns is not None or end_ns is not None:
                # A time window skips whole chunks; ids then count from the
                # window's first message, which is all a windowed read needs.
                msgs = reader.iter_messages(
                    topics=[self._topic], start_time=start_ns, end_time=end_ns, reverse=reverse
                )
            else:
                msgs = reader.iter_messages(topics=[self._topic], reverse=reverse)
            on_disk = n - len(self._pending)
            if reverse:
                yield from self._iter_pending(reverse=True, start_ns=start_ns, end_ns=end_ns)
            for i, (_schema, _channel, message) in enumerate(msgs):
                observation_time = (
                    message.publish_time
                    if self._observation_uses_publish_time
                    else message.log_time
                )
                payload, pose, tags = self._unwrap(message.data)
                yield Observation(
                    id=(on_disk - 1 - i) if reverse else i,
                    ts=observation_time / 1e9,
                    data_type=dtype,
                    pose_tuple=pose,
                    tags=tags,
                    _loader=partial(decode, payload),
                )
            if not reverse:
                yield from self._iter_pending(reverse=False, start_ns=start_ns, end_ns=end_ns)

    def _iter_pending(
        self, *, reverse: bool, start_ns: int | None, end_ns: int | None
    ) -> Iterator[Observation[Any]]:
        """The observations inserted since the last flush, which are not on disk yet."""
        if not self._pending:
            return
        first = self._count - len(self._pending)
        rows = list(enumerate(self._pending))
        if reverse:
            rows.reverse()
        for offset, (ts, data) in rows:
            at = int(ts * 1e9)
            if (start_ns is not None and at < start_ns) or (end_ns is not None and at > end_ns):
                continue
            payload, pose, tags = self._unwrap(data)
            yield Observation(
                id=first + offset,
                ts=ts,
                data_type=self._payload_type,
                pose_tuple=pose,
                tags=tags,
                _loader=partial(self._codec.decode, payload),
            )

    def iter_raw(self) -> Iterator[bytes]:
        """Every message on the channel, in order, undecoded. Feeds the vector index."""
        from mcap.reader import make_reader  # optional mcap dependency

        with open(self._path, "rb") as f:
            for _schema, _channel, message in make_reader(f).iter_messages(topics=[self._topic]):
                yield message.data
        for _ts, data in self._pending:
            yield data

    def _time_window(self, q: StreamQuery) -> tuple[int | None, int | None]:
        """Log-time bounds (ns) implied by the query's time filters, or None.

        Without this every ``.at(t)`` reads the whole channel off disk. The
        filters still run on what comes back, so the window only prunes; it
        is widened when observation time is publish time, which can differ
        from the log time the mcap index is keyed on.
        """
        lo, hi = -np.inf, np.inf
        for f in q.filters:
            if isinstance(f, AtFilter):
                lo, hi = max(lo, f.t - f.tolerance), min(hi, f.t + f.tolerance)
            elif isinstance(f, TimeRangeFilter):
                lo, hi = max(lo, f.t1), min(hi, f.t2)
            elif isinstance(f, AfterFilter):
                lo = max(lo, f.t)
            elif isinstance(f, BeforeFilter):
                hi = min(hi, f.t)
        if lo == -np.inf and hi == np.inf:
            return None, None
        slack = 1.0 if self._observation_uses_publish_time else 1e-3
        start = None if lo == -np.inf else int((lo - slack) * 1e9)
        end = None if hi == np.inf else int((hi + slack) * 1e9) + 1
        return start, end

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        start_ns, end_ns = self._time_window(q)
        # MCAP is natively log-time ordered, so id ordering never needs a sort.
        # Native DimOS recordings expose publish_time as observation ts; source
        # time can differ from log/reception order and must use the generic sort.
        if q.order_field == "id" or (
            q.order_field == "ts" and not self._observation_uses_publish_time
        ):
            it = self._iter(reverse=q.order_desc, start_ns=start_ns, end_ns=end_ns)
            q = replace(q, order_field=None, order_desc=False)
            return q.apply(it)
        return q.apply(self._iter(start_ns=start_ns, end_ns=end_ns))

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
        """Append one observation to this stream's channel; its id is its position.

        The payload is encoded HERE rather than handed to a blob store, because an
        mcap message is the whole observation: payload, pose, tags and embedding in
        one record. Ids count up from zero in insertion order, which is the order a
        read comes back in, because a dimos stream is only ever appended to.
        """
        if self._appender_of is None or self._channel_id is None:
            raise NotImplementedError(
                f"{self._topic!r} was written by the recorder; dimos appends to its own "
                "channels only. Write to a new stream name instead."
            )
        payload = self._codec.encode(obs.data)  # type: ignore[attr-defined]  # write streams get a real Codec
        if self._appender is None:
            self._appender = self._appender_of()
        envelope = write_observation(self._appender, self._channel_id, obs, payload)
        row_id = self._count
        self._count += 1
        self._pending.append((obs.ts, envelope))
        self._pending_bytes += len(envelope)
        return row_id

    def commit(self) -> None:
        """``Backend.append`` calls this after each observation.

        A flush rewrites the file's summary, so one per observation would cost
        megabytes per observation on a long recording. Instead a flush happens
        once the unflushed envelopes are worth a chunk -- until then they are
        served out of :attr:`_pending`, so a read still sees them.
        """
        if self._pending_bytes >= FLUSH_BYTES:
            self.flush()

    def flush(self) -> None:
        """Put everything inserted so far on disk, where another process can read it."""
        if self._appender is None:
            return
        self._appender.flush()
        self._pending = []
        self._pending_bytes = 0


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
        codecs: Mapping[str, StreamCodec] | None = None,
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        from mcap.reader import make_reader  # optional mcap dependency

        super().__init__(**kwargs)
        self._codecs = dict(codecs or {})
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
        self._enveloped: dict[str, int] = {}  # dimos-written stream name -> channel id
        if summary is not None and summary.statistics is not None:
            for cid, ch in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                if ch.message_encoding == ENVELOPE_ENCODING:
                    # A stream dimos wrote. It carries its own name, because
                    # slugging the topic back is lossy -- a leading slash becomes an
                    # underscore. Its codec comes from the registry rather than from
                    # the caller's map, and it is never "raw bytes".
                    name = ch.metadata.get("dimos.stream") or name
                    self._enveloped[name] = cid
                    self._available[name] = count
                    self._stream_topic[name] = ch.topic
                    continue
                if ch.topic not in self._codecs and ch.message_encoding == "jpeg":
                    self._codecs[ch.topic] = JpegCodec()
                self._stream_topic[name] = ch.topic
                self._available[name] = count
                self._observation_uses_publish_time[name] = (
                    ch.metadata.get("dimos.observation_time") == "publish_time"
                )
                if ch.topic not in self._codecs:
                    sch = summary.schemas.get(ch.schema_id)
                    self._raw[name] = sch.name if sch else None
        self._registry: dict[str, dict[str, Any]] = {
            key: json.loads(value)
            for key, value in read_metadata(self.config.path).get(REGISTRY_METADATA, {}).items()
        }
        self._appender: McapAppender | None = None
        self._write_lock = threading.RLock()

    def _writer(self) -> McapAppender:
        """The one appender this store writes through, opened on first write."""
        with self._write_lock:
            if self._appender is None:
                self._appender = McapAppender(self.config.path)
            return self._appender

    def list_streams(self) -> list[str]:
        return sorted(set(self._available) | set(self._registry) | set(self._streams))

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
        if name in self._registry:
            return self._dimos_backend(name, self._registry[name], payload_type)
        if name in self._available:
            return self._recorded_backend(name)
        if payload_type is None:
            raise KeyError(
                f"No stream {name!r}. Available: {sorted(set(self._available) | set(self._registry))}"
                " — pass a payload type to create it."
            )
        return self._new_backend(name, payload_type, config.get("codec"))

    def _recorded_backend(self, name: str) -> Backend[Any]:
        """A channel the recorder wrote: read-only, decoded by the caller's codec map."""
        topic = self._stream_topic[name]
        codec = self._codecs.get(topic) or _BYTES_CODEC  # no codec -> Stream[bytes]
        ptype = codec.payload_type
        obs = McapObservationStore(
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

    def _dimos_backend(
        self, name: str, stored: dict[str, Any], payload_type: type | None
    ) -> Backend[Any]:
        """A channel dimos wrote, reopened from what the registry says about it."""
        from dimos.memory.codecs.base import codec_from_id, resolve_payload_type

        module = stored["payload_module"]
        if payload_type is not None:
            asked = f"{payload_type.__module__}.{payload_type.__qualname__}"
            if asked != module:
                raise ValueError(
                    f"Stream {name!r} was created with type {module}, but opened with {asked}"
                )
        codec = codec_from_id(stored["codec_id"], module)
        return self._backend_for(name, resolve_payload_type(module), codec)

    def _new_backend(self, name: str, payload_type: type, raw_codec: Any) -> Backend[Any]:
        """Declare a channel for a stream that does not exist yet, and record it."""
        module = f"{payload_type.__module__}.{payload_type.__qualname__}"
        codec = self._resolve_codec(payload_type, raw_codec)
        with self._write_lock:
            writer = self._writer()
            channel_id = writer.add_channel(channel_spec(name, module))
            self._registry[name] = {"payload_module": module, "codec_id": codec_id(codec)}
            self._enveloped[name] = channel_id
            self._stream_topic[name] = topic_for(name)
            self._available[name] = 0
            self._observation_uses_publish_time[name] = False
            writer.put_metadata(
                REGISTRY_METADATA,
                {key: json.dumps(value) for key, value in self._registry.items()},
            )
            writer.flush()  # the channel exists on disk before anything is written to it
        return self._backend_for(name, payload_type, codec)

    def _backend_for(self, name: str, payload_type: type, codec: Any) -> Backend[Any]:
        """The read/write backend over a dimos channel, however it was reached."""
        with self._write_lock:
            channel_id = self._enveloped.get(name)
            if channel_id is None:
                raise KeyError(f"{name!r} is in the registry but has no channel in the file")
        obs = McapObservationStore(
            name=name,
            path=self.config.path,
            topic=topic_for(name),
            codec=codec,
            count=self._available.get(name, 0),
            observation_uses_publish_time=False,
            appender=self._writer,
            channel_id=channel_id,
            enveloped=True,
            payload_type=payload_type,
        )
        return Backend(
            metadata_store=obs,
            codec=codec,
            data_type=payload_type,
            # The envelope holds the payload, so there is no blob store to hold it
            # again; the vector store is a search index over vectors already there.
            blob_store=None,
            vector_store=McapVectorStore(load=lambda: iter_vectors(obs.iter_raw())),
            notifier=SubjectNotifier(),
        )

    def delete_stream(self, name: str) -> None:
        raise NotImplementedError(
            "an mcap is appended to, not rewritten in place; remove a topic with "
            "`dtk mcap_edit --delete`"
        )

    def stop(self) -> None:
        if self._appender is not None:
            self._appender.close()
            self._appender = None
        super().stop()
