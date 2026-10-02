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

"""Schema-aware, read-only store for native CDR/JSON MCAP recordings."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
from typing import Any

try:
    from mcap.reader import make_reader
    from mcap.records import Schema
    from mcap_ros2.decoder import DecoderFactory
except ImportError as exc:
    raise ImportError("MCAP recording support requires pip install 'dimos[recording]'") from exc

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import codec_for
from dimos.memory.codecs.ros import ROS_READERS
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.store.base import Store
from dimos.memory.store.mcap import McapObservationStore, McapStoreConfig
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D


@dataclass(frozen=True)
class _Decoder:
    payload_type: type
    decode_wire: Callable[[bytes], Any]
    convert: Callable[[Any], Any]

    def decode(self, data: bytes) -> Any:
        return self.convert(self.decode_wire(data))


def _identity(value: Any) -> Any:
    return value


def _segments(value: Any) -> LineSegments3D:
    if len(value["segments"]) != len(value["weights"]):
        raise ValueError("LineSegments3D requires one weight per segment")
    return LineSegments3D(**value)


def _decoder(encoding: str, schema: Schema, factory: DecoderFactory) -> _Decoder:
    if encoding == "cdr" and schema.encoding == "ros2msg":
        decode = factory.decoder_for(encoding, schema)
        if decode is None:
            raise ValueError(f"Cannot decode CDR schema {schema.name!r}")
        payload_type, convert = ROS_READERS.get(schema.name, (object, _identity))
        return _Decoder(payload_type, decode, convert)
    if encoding == "json" and schema.encoding == "jsonschema":
        if schema.name == "dimos.LineSegments3D":
            return _Decoder(LineSegments3D, json.loads, _segments)
        return _Decoder(dict, json.loads, _identity)
    raise ValueError(f"Unsupported MCAP message/schema encodings: {encoding!r}/{schema.encoding!r}")


class McapRecordingStore(Store):
    """Open a completed recording using embedded schemas and trusted conversions.

    File metadata never triggers a Python import, and decoding never passes
    through LCM; unknown ROS schemas remain decoded objects and JSON remains dicts.
    """

    config: McapStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        with open(self.config.path, "rb") as source:
            summary = make_reader(source).get_summary()
        if summary is None or summary.statistics is None:
            raise ValueError("McapRecordingStore requires a finalized MCAP summary and statistics")
        factory = DecoderFactory()
        self._channels: dict[str, tuple[str, _Decoder, int, bool]] = {}
        topics: set[str] = set()
        for channel_id, channel in summary.channels.items():
            schema = summary.schemas.get(channel.schema_id)
            if schema is None or not schema.data:
                raise ValueError(f"Channel {channel.topic!r} has no schema definition")
            name = channel.metadata.get("dimos.stream_name", channel.topic)
            if name in self._channels or channel.topic in topics:
                raise ValueError(f"Duplicate recording stream or topic: {name!r}/{channel.topic!r}")
            topics.add(channel.topic)
            self._channels[name] = (
                channel.topic,
                _decoder(channel.message_encoding, schema, factory),
                summary.statistics.channel_message_counts.get(channel_id, 0),
                channel.metadata.get("dimos.observation_time") == "publish_time",
            )

    def list_streams(self) -> list[str]:
        return sorted(self._channels)

    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend[Any]:
        topic, decoder, count, publish_time = self._channels[name]
        if payload_type is not None and payload_type is not decoder.payload_type:
            raise TypeError(f"Stream {name!r} contains {decoder.payload_type.__name__}")
        return Backend(
            metadata_store=McapObservationStore(
                name=name,
                path=self.config.path,
                topic=topic,
                codec=decoder,
                count=count,
                observation_uses_publish_time=publish_time,
            ),
            codec=codec_for(decoder.payload_type),
            data_type=decoder.payload_type,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )
