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

"""Read-only memory store backed by an mcap file.

Generic and robot-independent. JPEG channels decode automatically because their
payload type is fixed. Other formats use a caller-supplied ``codecs`` map (wire
topic -> codec), while ``streams`` may map friendly stream names to topics. See
``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 DDS wiring.

Channels published by dimos itself need neither: a dimos topic carries its
message type in its own name, so it is decoded and named from the wire with no
registry at all (``_dimos_wire``). Neither do ROS 2 (CDR) channels whose type
has a dimos twin: the mcap embeds their schema (``_Ros2Codec``). An injected
codec still wins where these apply.

Read-only: no append, blobs, vectors, or embeddings. Payloads decode lazily on
``obs.data``; ts and counts are cheap (counts come from the mcap index).
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, replace
from functools import partial
from typing import Any, Protocol, runtime_checkable

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import codec_for
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.codecs.json import JsonCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory.store.base import Store, StoreConfig
from dimos.memory.type.filter import StreamQuery
from dimos.memory.type.observation import Observation
from dimos.msgs.helpers import lcm_msg_type
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.rospubsub_conversion import get_dimos_type


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


@dataclass(frozen=True)
class _DimosCodec:
    """Decodes a dimos wire channel through its own message type."""

    payload_type: type

    def decode(self, data: bytes) -> Any:
        return self.payload_type.lcm_decode(data)  # type: ignore[attr-defined]


_LCM_PRIMITIVES = {
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "float",
    "double",
    "string",
    "boolean",
    "byte",
}


def _ros_to_lcm(ros: Any, lcm_type: Any) -> Any:
    """Copy a decoded ROS 2 message into its dimos_lcm twin, which shares its field names."""
    out = lcm_type()
    package = lcm_type.__module__.split(".")[-2]  # dimos_lcm.sensor_msgs.PointCloud2
    for name, kind, dims in zip(
        lcm_type.__slots__, lcm_type.__typenames__, lcm_type.__dimensions__, strict=True
    ):
        value = getattr(ros, "nanosec" if name == "nsec" else name, None)
        if value is None:  # a `_length` field, or one ROS 2 dropped (Header.seq)
            continue
        if kind not in _LCM_PRIMITIVES:
            sub = lcm_msg_type(kind if "." in kind else f"{package}.{kind}")
            value = [_ros_to_lcm(v, sub) for v in value] if dims else _ros_to_lcm(value, sub)
        elif dims:
            value = bytes(value) if kind == "byte" else list(value)
        setattr(out, name, value)
        if dims and isinstance(dims[0], str):  # variable length: LCM carries the count
            setattr(out, dims[0], len(value))
    return out


@dataclass(frozen=True)
class _Ros2Codec:
    """Decodes a ROS 2 channel with the schema the mcap embeds, then into its dimos type."""

    payload_type: type
    msg_type: type
    decode_ros: Callable[[bytes], Any]
    # A flat dimos type that is its own LCM struct is filled directly: an LCM round
    # trip would push ROS's unsigned ints through signed ones (a uint32 of 3e9 fails)
    direct: bool = False

    def decode(self, data: bytes) -> Any:
        ros = self.decode_ros(data)
        if self.direct:
            return _ros_to_lcm(ros, self.msg_type)
        twin = lcm_msg_type(self.msg_type.msg_name)  # type: ignore[attr-defined]
        msg = self.msg_type.lcm_decode(_ros_to_lcm(ros, twin).lcm_encode())  # type: ignore[attr-defined]
        return msg if self.payload_type is self.msg_type else msg.decode()


def _ros2_codec(schema: Any) -> _Ros2Codec | None:
    """A codec for a ``ros2msg`` schema with a dimos twin, else None."""
    package, _, name = schema.name.partition("/msg/")  # sensor_msgs/msg/PointCloud2
    msg_type = get_dimos_type(f"{package}.{name}") if name else None
    if msg_type is None:
        return None
    from mcap_ros2.decoder import DecoderFactory  # optional mcap dependency

    decode_ros = DecoderFactory().decoder_for("cdr", schema)
    if decode_ros is None:
        return None
    # A compressed frame decodes to an Image, as the jpeg channels do
    payload = Image if msg_type.msg_name == "sensor_msgs.CompressedImage" else msg_type
    twin = lcm_msg_type(msg_type.msg_name)
    direct = issubclass(msg_type, twin) and set(twin.__typenames__) <= _LCM_PRIMITIVES
    return _Ros2Codec(payload, msg_type, decode_ros, direct)


def _dimos_wire(topic: str) -> tuple[str, type] | None:
    """Split a dimos wire topic into its port name and message type.

    A dimos topic embeds the type in its last segment,
    ``dimos/<port>/<msg_name>`` (built by ``zenohpubsub.Topic.key_expr``), so
    every dimos channel describes itself and no per-topic registry is needed.
    None for anything that is not one, which leaves DDS and app channels to
    the injected codecs.
    """
    prefix, _, rest = topic.partition("/")
    port, _, msg_name = rest.rpartition("/")
    if prefix != "dimos" or not port:
        return None
    msg_type = get_dimos_type(msg_name)
    return (port, msg_type) if msg_type is not None else None


def _slug(topic: str) -> str:
    """Auto stream name from a topic: drop the ``rt/`` prefix and the ROS 2 leading ``/``,
    then ``/`` -> ``_``.

    ``rt/`` is the ROS2-over-DDS topic prefix; ``removeprefix`` only strips it
    where present (e.g. app-level ``control_log`` is left alone). A dimos topic
    names itself by its port, dropping the type segment: the dot in
    ``nav_msgs.Path`` is not attribute-addressable on ``store.streams``.
    """
    wire = _dimos_wire(topic)
    if wire is not None:
        return wire[0].replace("/", "_")
    return topic.removeprefix("rt/").strip("/").replace("/", "_")


class McapObservationStoreConfig(ObservationStoreConfig):
    name: str = "<mcap>"


class McapObservationStore(ObservationStore[Any]):
    """Read-only metadata/query over one mcap channel. Payloads load lazily."""

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
        if summary is not None and summary.statistics is not None:
            # ROS topics folding to one slug (`a/b_c`, `a_b/c`) keep their `/` as `__` instead
            folded = Counter(
                _slug(t)
                for t in {ch.topic for ch in summary.channels.values()}
                if _dimos_wire(t) is None
            )
            for cid, ch in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                if ch.topic not in name_of and folded[name] > 1 and _dimos_wire(ch.topic) is None:
                    name = ch.topic.removeprefix("rt/").strip("/").replace("/", "__")
                taken = self._stream_topic.get(name)
                if taken is not None and taken != ch.topic:
                    # Two topics, one name. One port carrying two types is
                    # the type's to tell apart (`shared` -> `shared_Imu`); two
                    # ports folding to one slug is a naming decision for
                    # `streams=`, not something to guess at.
                    wire, had = _dimos_wire(ch.topic), _dimos_wire(taken)
                    if wire is not None and had is not None and wire[0] == had[0]:
                        name = f"{name}_{wire[1].__name__}"
                    if self._stream_topic.get(name, ch.topic) != ch.topic:
                        raise ValueError(
                            f"stream {name!r} would name both {taken!r} and {ch.topic!r};"
                            " pass streams= to name them apart"
                        )
                if ch.topic not in self._codecs and ch.message_encoding == "jpeg":
                    self._codecs[ch.topic] = JpegCodec()
                if ch.topic not in self._codecs and ch.message_encoding == "json":
                    kind = JsonCodec.payload_type
                    if (
                        ch.metadata.get("dimos.payload_type")
                        == f"{kind.__module__}.{kind.__qualname__}"
                    ):
                        self._codecs[ch.topic] = JsonCodec()
                sch = summary.schemas.get(ch.schema_id)
                if (
                    ch.topic not in self._codecs
                    and ch.message_encoding == "cdr"
                    and sch is not None
                    and sch.encoding == "ros2msg"
                    and (ros2 := _ros2_codec(sch)) is not None
                ):
                    self._codecs[ch.topic] = ros2
                self._stream_topic[name] = ch.topic
                self._available[name] = count
                self._observation_uses_publish_time[name] = (
                    ch.metadata.get("dimos.observation_time") == "publish_time"
                )
                if ch.topic not in self._codecs and _dimos_wire(ch.topic) is None:
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
        # Injected codecs win; a dimos channel decodes itself; the rest stay bytes.
        wire = _dimos_wire(topic)
        fallback: StreamCodec = _DimosCodec(wire[1]) if wire is not None else _BYTES_CODEC
        codec = self._codecs.get(topic) or fallback
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
