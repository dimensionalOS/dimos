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

"""Self-describing CDR recording, independent of transport and generated classes."""

from __future__ import annotations

from pathlib import Path
from types import TracebackType

from mcap.writer import CompressionType, Writer
from typing_extensions import Self


class CdrMcapWriter:
    """Write raw generated CDR with embedded ROS2 definitions and chunk compression.

    Callers supply source and reception timestamps explicitly. No message package
    is imported, so a recorder can accept schemas from external stream providers.
    """

    def __init__(self, path: str | Path) -> None:
        self._stream = Path(path).open("wb")
        try:
            self._writer = Writer(self._stream, compression=CompressionType.ZSTD)
            self._writer.start(profile="ros2", library="dimos")
        except BaseException:
            self._stream.close()
            raise
        self._schemas: dict[tuple[str, str], int] = {}
        self._channels: dict[tuple[str, int], int] = {}
        self._closed = False

    def write(
        self,
        topic: str,
        payload: bytes,
        *,
        schema_name: str,
        schema: str,
        log_time_ns: int,
        publish_time_ns: int | None = None,
        sequence: int = 0,
    ) -> None:
        if self._closed:
            raise ValueError("MCAP writer is closed")
        schema_key = (schema_name, schema)
        if schema_key not in self._schemas:
            self._schemas[schema_key] = self._writer.register_schema(
                name=schema_name, encoding="ros2msg", data=schema.encode("utf-8")
            )
        schema_id = self._schemas[schema_key]
        channel_key = (topic, schema_id)
        if channel_key not in self._channels:
            self._channels[channel_key] = self._writer.register_channel(
                topic=topic,
                message_encoding="cdr",
                schema_id=schema_id,
                metadata={"offered_qos_profiles": "[]"},
            )
        self._writer.add_message(
            channel_id=self._channels[channel_key],
            data=payload,
            log_time=log_time_ns,
            publish_time=log_time_ns if publish_time_ns is None else publish_time_ns,
            sequence=sequence,
        )

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            try:
                self._writer.finish()
            finally:
                self._stream.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
