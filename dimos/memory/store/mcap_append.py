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

"""Append channels and messages to a finished mcap, without copying it.

An mcap is a list of self-contained chunks followed by a summary that says where
they are, and nothing in the data section refers to an offset after itself. So new
messages go over the top of the old DataEnd record -- where the data section ended
-- and a fresh summary is written past them. Appending a megabyte to a 70 GB
recording costs a megabyte plus one summary, not 70 GB.

That is what lets derived data live in the recording it came from. The alternative
was a companion ``.derived.db``, and a companion goes stale the moment either half
is moved, trimmed or copied on its own.

The file is a valid mcap after every :meth:`McapAppender.flush`, which is what a
reader in another process sees. Between flushes the pending messages exist only in
this process; a crash loses them and leaves the recording exactly as it was.

The record-level encoding here is the same one ``dtk mcap_edit`` uses; the spec is
at https://mcap.dev/spec.
"""

from __future__ import annotations

import os
from pathlib import Path
import struct
from typing import Any
import zlib

MAGIC = b"\x89MCAP0\r\n"

OP_HEADER = 0x01
OP_FOOTER = 0x02
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_MESSAGE_INDEX = 0x07
OP_CHUNK_INDEX = 0x08
OP_ATTACHMENT_INDEX = 0x0A
OP_STATISTICS = 0x0B
OP_METADATA = 0x0C
OP_METADATA_INDEX = 0x0D
OP_SUMMARY_OFFSET = 0x0E
OP_DATA_END = 0x0F

# A record is an opcode and a length before anything else, so nothing shorter
# than this is a record.
RECORD_OVERHEAD = 9

# What a chunk is allowed to reach before it is written out. The summary is
# rewritten once per flush, so a small target makes every flush pay for the
# whole summary again; 4 MiB is what mcap_edit uses and what recorders write.
CHUNK_TARGET = 4 << 20

PROFILE = "ros2"
LIBRARY = "dimos"


# ---- record primitives -------------------------------------------------------


class _Reader:
    """Little-endian cursor over one record body."""

    def __init__(self, data: bytes, at: int = 0) -> None:
        self.data = data
        self.at = at

    def u16(self) -> int:
        value: int = struct.unpack_from("<H", self.data, self.at)[0]
        self.at += 2
        return value

    def u32(self) -> int:
        value: int = struct.unpack_from("<I", self.data, self.at)[0]
        self.at += 4
        return value

    def u64(self) -> int:
        value: int = struct.unpack_from("<Q", self.data, self.at)[0]
        self.at += 8
        return value

    def text(self) -> str:
        length = self.u32()
        value = self.data[self.at : self.at + length].decode("utf-8")
        self.at += length
        return value

    def blob(self) -> bytes:
        length = self.u32()
        value = bytes(self.data[self.at : self.at + length])
        self.at += length
        return value


def _put_text(out: bytearray, value: str) -> None:
    raw = value.encode("utf-8")
    out.extend(struct.pack("<I", len(raw)))
    out.extend(raw)


def _record(opcode: int, body: bytes | bytearray) -> bytes:
    return struct.pack("<BQ", opcode, len(body)) + bytes(body)


def _compress(compression: str, data: bytes) -> bytes:
    if compression == "":
        return data
    if compression == "zstd":
        import zstandard

        return bytes(zstandard.ZstdCompressor(level=3).compress(data))
    if compression == "lz4":
        import lz4.frame

        return bytes(lz4.frame.compress(data))
    raise ValueError(f"unsupported chunk compression {compression!r}")


def _encode_schema(schema_id: int, name: str, encoding: str, data: bytes) -> bytes:
    out = bytearray(struct.pack("<H", schema_id))
    _put_text(out, name)
    _put_text(out, encoding)
    out.extend(struct.pack("<I", len(data)))
    out.extend(data)
    return _record(OP_SCHEMA, out)


def _encode_channel(
    channel_id: int,
    schema_id: int,
    topic: str,
    message_encoding: str,
    metadata: list[tuple[str, str]],
) -> bytes:
    out = bytearray(struct.pack("<HH", channel_id, schema_id))
    _put_text(out, topic)
    _put_text(out, message_encoding)
    pairs = bytearray()
    for key, value in metadata:
        _put_text(pairs, key)
        _put_text(pairs, value)
    out.extend(struct.pack("<I", len(pairs)))
    out.extend(pairs)
    return _record(OP_CHANNEL, out)


def _encode_metadata(name: str, entries: dict[str, str]) -> bytes:
    out = bytearray()
    _put_text(out, name)
    pairs = bytearray()
    for key, value in entries.items():
        _put_text(pairs, key)
        _put_text(pairs, value)
    out.extend(struct.pack("<I", len(pairs)))
    out.extend(pairs)
    return _record(OP_METADATA, out)


def _encode_metadata_index(offset: int, length: int, name: str) -> bytes:
    out = bytearray(struct.pack("<QQ", offset, length))
    _put_text(out, name)
    return _record(OP_METADATA_INDEX, out)


def _encode_chunk(start: int, end: int, uncompressed: bytes, compression: str) -> bytes:
    payload = _compress(compression, uncompressed)
    out = bytearray(struct.pack("<QQQI", start, end, len(uncompressed), zlib.crc32(uncompressed)))
    _put_text(out, compression)
    out.extend(struct.pack("<Q", len(payload)))
    out.extend(payload)
    return _record(OP_CHUNK, out)


def _encode_message_index(channel_id: int, entries: list[tuple[int, int]]) -> bytes:
    body = bytearray(struct.pack("<H", channel_id))
    packed = bytearray()
    for log_time, offset in entries:
        packed.extend(struct.pack("<QQ", log_time, offset))
    body.extend(struct.pack("<I", len(packed)))
    body.extend(packed)
    return _record(OP_MESSAGE_INDEX, body)


def _encode_chunk_index(index: dict[str, Any]) -> bytes:
    out = bytearray(
        struct.pack(
            "<QQQQ",
            index["message_start_time"],
            index["message_end_time"],
            index["chunk_start_offset"],
            index["chunk_length"],
        )
    )
    offsets = bytearray()
    for channel_id, offset in index["message_index_offsets"]:
        offsets.extend(struct.pack("<HQ", channel_id, offset))
    out.extend(struct.pack("<I", len(offsets)))
    out.extend(offsets)
    out.extend(struct.pack("<Q", index["message_index_length"]))
    _put_text(out, index["compression"])
    out.extend(struct.pack("<QQ", index["compressed_size"], index["uncompressed_size"]))
    return _record(OP_CHUNK_INDEX, out)


def _parse_chunk_index(body: bytes) -> dict[str, Any]:
    r = _Reader(body)
    index: dict[str, Any] = {
        "message_start_time": r.u64(),
        "message_end_time": r.u64(),
        "chunk_start_offset": r.u64(),
        "chunk_length": r.u64(),
    }
    offsets_length = r.u32()
    stop = r.at + offsets_length
    channels = []
    while r.at < stop:
        channels.append((r.u16(), r.u64()))
    index["message_index_offsets"] = channels
    index["message_index_length"] = r.u64()
    index["compression"] = r.text()
    index["compressed_size"] = r.u64()
    index["uncompressed_size"] = r.u64()
    return index


def _parse_statistics(body: bytes) -> dict[str, Any]:
    r = _Reader(body)
    stats: dict[str, Any] = {
        "message_count": r.u64(),
        "schema_count": r.u16(),
        "channel_count": r.u32(),
        "attachment_count": r.u32(),
        "metadata_count": r.u32(),
        "chunk_count": r.u32(),
        "message_start_time": r.u64(),
        "message_end_time": r.u64(),
    }
    counts_length = r.u32()
    stop = r.at + counts_length
    counts: dict[int, int] = {}
    while r.at < stop:
        channel_id, count = r.u16(), r.u64()
        counts[channel_id] = count
    stats["channel_message_counts"] = counts
    return stats


def _encode_statistics(stats: dict[str, Any]) -> bytes:
    out = bytearray(
        struct.pack(
            "<QHIIIIQQ",
            stats["message_count"],
            stats["schema_count"],
            stats["channel_count"],
            stats["attachment_count"],
            stats["metadata_count"],
            stats["chunk_count"],
            stats["message_start_time"],
            stats["message_end_time"],
        )
    )
    counts = bytearray()
    for channel_id, count in sorted(stats["channel_message_counts"].items()):
        counts.extend(struct.pack("<HQ", channel_id, count))
    out.extend(struct.pack("<I", len(counts)))
    out.extend(counts)
    return _record(OP_STATISTICS, out)


# ---- the appender ------------------------------------------------------------


class ChannelSpec:
    """What a channel needs before anything can be written to it."""

    __slots__ = (
        "message_encoding",
        "metadata",
        "schema_data",
        "schema_encoding",
        "schema_name",
        "topic",
    )

    def __init__(
        self,
        topic: str,
        message_encoding: str,
        *,
        schema_name: str = "",
        schema_encoding: str = "",
        schema_data: bytes = b"",
        metadata: dict[str, str] | None = None,
    ) -> None:
        self.topic = topic
        self.message_encoding = message_encoding
        self.schema_name = schema_name
        self.schema_encoding = schema_encoding
        self.schema_data = schema_data
        self.metadata = dict(metadata or {})


class McapAppender:
    """Adds channels, messages and metadata records to an mcap file.

    Creating one on a path that does not exist writes a new, empty mcap there;
    on a path that does, it reads the summary (a few seeks, not a pass over the
    file) and positions itself to write over the DataEnd record.

    Not thread-safe on its own -- :class:`~dimos.memory.store.mcap.McapStore`
    owns one per file and serialises writes through it.
    """

    def __init__(self, path: str | os.PathLike[str], *, compression: str = "zstd") -> None:
        self.path = str(path)
        self._pending = bytearray()
        self._entries: dict[int, list[tuple[int, int]]] = {}
        self._span: list[int | None] = [None, None]
        self._new_metadata: list[tuple[str, dict[str, str]]] = []
        self._dirty = False
        if not Path(self.path).exists():
            self._create(compression)
        else:
            self._open()

    # -- opening ---------------------------------------------------------------

    def _create(self, compression: str) -> None:
        parent = os.path.dirname(os.path.abspath(self.path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        header = bytearray()
        _put_text(header, PROFILE)
        _put_text(header, LIBRARY)
        with open(self.path, "wb") as f:
            f.write(MAGIC)
            f.write(_record(OP_HEADER, header))
        self.file = open(self.path, "r+b")
        self.compression = compression
        self.schemas: dict[int, bytes] = {}
        self.schema_names: dict[int, str] = {}
        self.channels: dict[int, bytes] = {}
        self.channel_topic: dict[int, str] = {}
        self.chunk_indexes: list[dict[str, Any]] = []
        self.other_summary: list[bytes] = []
        self.statistics: dict[str, Any] = {
            "message_count": 0,
            "schema_count": 0,
            "channel_count": 0,
            "attachment_count": 0,
            "metadata_count": 0,
            "chunk_count": 0,
            "message_start_time": 0,
            "message_end_time": 0,
            "channel_message_counts": {},
        }
        self.write_at = self.file.seek(0, os.SEEK_END)
        self._declared: set[int] = set()
        self._dirty = True  # an mcap without its DataEnd, summary and footer is not one yet
        self.flush()

    def _open(self) -> None:
        self.file = open(self.path, "r+b")
        size = self.file.seek(0, os.SEEK_END)
        if size < len(MAGIC) + 29 + len(MAGIC) or self._read_at(0, 8) != MAGIC:
            raise ValueError(f"{self.path}: not an mcap")
        if self._read_at(size - 8, 8) != MAGIC:
            raise ValueError(f"{self.path}: no closing magic -- run `dtk mcap_recover` first")
        footer = self._read_at(size - 8 - 29, 29)
        if footer[0] != OP_FOOTER:
            raise ValueError(f"{self.path}: footer is not where the spec puts it")
        summary_start, summary_offset_start, _ = struct.unpack("<QQI", footer[9:29])
        if summary_start == 0:
            raise ValueError(
                f"{self.path}: has no summary section; nothing to append against. "
                "Run `dtk mcap_recover` to rebuild one."
            )
        # New records go over the top of the DataEnd record -- leaving it in place
        # would put records after the marker that says there are none.
        self.write_at = summary_start - (RECORD_OVERHEAD + 4)
        if self._read_at(self.write_at, 1)[0] != OP_DATA_END:
            raise ValueError(f"{self.path}: no DataEnd record before the summary")
        self._read_summary(summary_start, summary_offset_start or (size - 8 - 29))
        # Whatever the file already uses, so one recording does not end up half
        # zstd and half uncompressed.
        self.compression = self.chunk_indexes[0]["compression"] if self.chunk_indexes else "zstd"
        self._declared = set(self.channels)

    def _read_at(self, offset: int, length: int) -> bytes:
        self.file.seek(offset)
        return self.file.read(length)

    def _read_summary(self, start: int, end: int) -> None:
        blob = self._read_at(start, end - start)
        self.schemas = {}
        self.schema_names = {}
        self.channels = {}
        self.channel_topic = {}
        self.chunk_indexes = []
        self.other_summary = []
        self.statistics = {}
        at = 0
        while at + RECORD_OVERHEAD <= len(blob):
            opcode = blob[at]
            length = struct.unpack_from("<Q", blob, at + 1)[0]
            body = blob[at + RECORD_OVERHEAD : at + RECORD_OVERHEAD + length]
            if opcode == OP_SCHEMA:
                r = _Reader(body)
                schema_id = r.u16()
                self.schema_names[schema_id] = r.text()
                self.schemas[schema_id] = _record(OP_SCHEMA, body)
            elif opcode == OP_CHANNEL:
                r = _Reader(body)
                channel_id = r.u16()
                r.u16()  # schema id
                self.channel_topic[channel_id] = r.text()
                self.channels[channel_id] = _record(OP_CHANNEL, body)
            elif opcode == OP_CHUNK_INDEX:
                self.chunk_indexes.append(_parse_chunk_index(body))
            elif opcode == OP_STATISTICS:
                self.statistics = _parse_statistics(body)
            elif opcode in (OP_ATTACHMENT_INDEX, OP_METADATA_INDEX):
                # Their offsets point into the data section, which an append never
                # moves, so they are carried through byte for byte.
                self.other_summary.append(_record(opcode, body))
            at += RECORD_OVERHEAD + length
        self.chunk_indexes.sort(key=lambda index: index["chunk_start_offset"])
        if not self.statistics:
            raise ValueError(
                f"{self.path}: summary has no Statistics record; append cannot keep the "
                "message counts honest. Run `dtk mcap_recover` to rebuild the summary."
            )

    # -- declaring ------------------------------------------------------------

    def topics(self) -> dict[str, int]:
        """Topic -> channel id, for everything the file already declares."""
        return {topic: cid for cid, topic in self.channel_topic.items()}

    def add_channel(self, spec: ChannelSpec) -> int:
        """Declare a new channel and return its id. The topic must be unused.

        An existing topic is not reopened for writing: its messages are spread
        through chunks all over the file, and appending more would leave the
        channel's messages out of time order, which an indexed reader does not
        expect. Derived data gets its own topic.
        """
        if spec.topic in self.topics():
            raise ValueError(f"{spec.topic!r} is already a topic in {self.path}")
        schema_id = 0
        if spec.schema_name:
            schema_id = max(self.schemas, default=0) + 1
            self.schemas[schema_id] = _encode_schema(
                schema_id, spec.schema_name, spec.schema_encoding, spec.schema_data
            )
            self.schema_names[schema_id] = spec.schema_name
        channel_id = max(self.channels, default=-1) + 1
        self.channels[channel_id] = _encode_channel(
            channel_id,
            schema_id,
            spec.topic,
            spec.message_encoding,
            sorted(spec.metadata.items()),
        )
        self.channel_topic[channel_id] = spec.topic
        self.statistics["channel_message_counts"].setdefault(channel_id, 0)
        self._dirty = True
        return channel_id

    def put_metadata(self, name: str, entries: dict[str, str]) -> None:
        """Queue an mcap Metadata record. Written outside any chunk, on flush.

        This is where a store keeps what a sqlite one keeps in a table of its own
        -- which streams exist and how each is encoded. Metadata records are not
        deduplicated by the format, so the LAST record with a given name wins and
        readers must take it that way.
        """
        self._new_metadata.append((name, dict(entries)))
        self._dirty = True

    # -- writing ---------------------------------------------------------------

    def add_message(
        self,
        channel_id: int,
        *,
        log_time: int,
        data: bytes,
        publish_time: int | None = None,
        sequence: int = 0,
    ) -> None:
        if channel_id not in self.channels:
            raise KeyError(f"no channel {channel_id} in {self.path}")
        publish = log_time if publish_time is None else publish_time
        body = struct.pack("<HIQQ", channel_id, sequence, log_time, publish) + data
        self._entries.setdefault(channel_id, []).append((log_time, len(self._pending)))
        if channel_id not in self._declared:
            # A reader walking the data section meets a channel where it is
            # declared, so the declaration goes at the head of the first chunk
            # that carries the channel's messages.
            schema_id = struct.unpack_from("<H", self.channels[channel_id], RECORD_OVERHEAD + 2)[0]
            if schema_id:
                self._pending.extend(self.schemas[schema_id])
            self._pending.extend(self.channels[channel_id])
            self._declared.add(channel_id)
            # The offset recorded above was taken before the declaration was
            # written, so it has to be retaken.
            self._entries[channel_id][-1] = (log_time, len(self._pending))
        self._pending.extend(_record(OP_MESSAGE, body))
        self._span[0] = log_time if self._span[0] is None else min(self._span[0], log_time)
        self._span[1] = log_time if self._span[1] is None else max(self._span[1], log_time)
        counts = self.statistics["channel_message_counts"]
        counts[channel_id] = counts.get(channel_id, 0) + 1
        self.statistics["message_count"] = self.statistics.get("message_count", 0) + 1
        self._dirty = True
        if len(self._pending) >= CHUNK_TARGET:
            self._write_chunk()

    def _write_chunk(self) -> None:
        """Put the buffered records on disk as one chunk plus its message indexes."""
        if not self._entries:
            return
        blob = bytes(self._pending)
        start, end = self._span[0] or 0, self._span[1] or 0
        chunk = _encode_chunk(start, end, blob, self.compression)
        offsets: list[tuple[int, int]] = []
        index_bytes = bytearray()
        for channel_id, entries in sorted(self._entries.items()):
            offsets.append((channel_id, self.write_at + len(chunk) + len(index_bytes)))
            index_bytes.extend(_encode_message_index(channel_id, entries))
        # The compressed length sits past the fixed head of the chunk record and a
        # variable-length compression name, so it is read back rather than recomputed.
        compressed_size = struct.unpack_from(
            "<Q", chunk, RECORD_OVERHEAD + 8 + 8 + 8 + 4 + 4 + len(self.compression)
        )[0]
        self.chunk_indexes.append(
            {
                "message_start_time": start,
                "message_end_time": end,
                "chunk_start_offset": self.write_at,
                "chunk_length": len(chunk),
                "message_index_offsets": offsets,
                "message_index_length": len(index_bytes),
                "compression": self.compression,
                "compressed_size": compressed_size,
                "uncompressed_size": len(blob),
            }
        )
        self.file.seek(self.write_at)
        self.file.write(chunk)
        self.file.write(index_bytes)
        self.write_at += len(chunk) + len(index_bytes)
        self.statistics["chunk_count"] = self.statistics.get("chunk_count", 0) + 1
        self._pending = bytearray()
        self._entries = {}
        self._span = [None, None]

    def flush(self) -> None:
        """Make everything written so far part of the file on disk.

        Writes the pending chunk, any queued metadata records, and then a fresh
        summary, offsets and footer. The file is a valid mcap when this returns.
        """
        if not self._dirty:
            return
        self._write_chunk()
        metadata_indexes = []
        for name, entries in self._new_metadata:
            raw = _encode_metadata(name, entries)
            self.file.seek(self.write_at)
            self.file.write(raw)
            metadata_indexes.append(_encode_metadata_index(self.write_at, len(raw), name))
            self.write_at += len(raw)
            self.statistics["metadata_count"] = self.statistics.get("metadata_count", 0) + 1
        self.other_summary.extend(metadata_indexes)
        self._new_metadata = []
        self._write_tail()
        self.file.flush()
        os.fsync(self.file.fileno())
        self._dirty = False

    def _write_tail(self) -> None:
        """DataEnd, the summary, the summary offsets and the footer, in one pass."""
        target = self.file
        target.seek(self.write_at)
        target.write(_record(OP_DATA_END, struct.pack("<I", 0)))
        summary_start = target.tell()

        groups: list[tuple[int, int, int]] = []
        blob = bytearray()

        for _schema_id, raw in sorted(self.schemas.items()):
            blob.extend(raw)
        if blob:
            groups.append((OP_SCHEMA, summary_start, len(blob)))

        channel_start = summary_start + len(blob)
        channel_bytes = bytearray()
        for _channel_id, raw in sorted(self.channels.items()):
            channel_bytes.extend(raw)
        blob.extend(channel_bytes)
        if channel_bytes:
            groups.append((OP_CHANNEL, channel_start, len(channel_bytes)))

        index_start = summary_start + len(blob)
        index_bytes = bytearray()
        for index in self.chunk_indexes:
            index_bytes.extend(_encode_chunk_index(index))
        blob.extend(index_bytes)
        if index_bytes:
            groups.append((OP_CHUNK_INDEX, index_start, len(index_bytes)))

        for raw in self.other_summary:
            blob.extend(raw)

        stats = dict(self.statistics)
        counts = stats["channel_message_counts"]
        stats["message_count"] = sum(counts.values())
        stats["channel_count"] = len(self.channels)
        stats["schema_count"] = len(self.schemas)
        spans = [i for i in self.chunk_indexes if i["message_end_time"]]
        if spans:
            stats["message_start_time"] = min(i["message_start_time"] for i in spans)
            stats["message_end_time"] = max(i["message_end_time"] for i in spans)
        stats_start = summary_start + len(blob)
        raw = _encode_statistics(stats)
        blob.extend(raw)
        groups.append((OP_STATISTICS, stats_start, len(raw)))
        self.statistics = stats

        target.write(blob)
        summary_offset_start = target.tell()
        offsets = bytearray()
        for opcode, start, length in groups:
            offsets.extend(_record(OP_SUMMARY_OFFSET, struct.pack("<BQQ", opcode, start, length)))
        target.write(offsets)
        target.write(
            _record(OP_FOOTER, struct.pack("<QQI", summary_start, summary_offset_start, 0))
        )
        target.write(MAGIC)
        target.truncate()

    def close(self) -> None:
        if self.file.closed:
            return
        try:
            self.flush()
        finally:
            self.file.close()

    def __enter__(self) -> McapAppender:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_metadata(path: str | os.PathLike[str], prefix: str = "") -> dict[str, dict[str, str]]:
    """Every Metadata record in the file, by name, last one winning.

    Read through the summary's MetadataIndex records, so this costs a seek per
    record rather than a pass over the file. A file with no summary offsets for
    them simply has none to return.
    """
    found: dict[str, dict[str, str]] = {}
    with open(path, "rb") as f:
        size = f.seek(0, os.SEEK_END)
        if size < len(MAGIC) + 29 + len(MAGIC):
            return found
        f.seek(size - 8 - 29)
        footer = f.read(29)
        if footer[0] != OP_FOOTER:
            return found
        summary_start, summary_offset_start, _ = struct.unpack("<QQI", footer[9:29])
        if summary_start == 0:
            return found
        end = summary_offset_start or (size - 8 - 29)
        f.seek(summary_start)
        blob = f.read(end - summary_start)
        at = 0
        locations: list[tuple[int, int, str]] = []
        while at + RECORD_OVERHEAD <= len(blob):
            opcode = blob[at]
            length = struct.unpack_from("<Q", blob, at + 1)[0]
            if opcode == OP_METADATA_INDEX:
                r = _Reader(blob[at + RECORD_OVERHEAD : at + RECORD_OVERHEAD + length])
                offset, record_length = r.u64(), r.u64()
                locations.append((offset, record_length, r.text()))
            at += RECORD_OVERHEAD + length
        for offset, record_length, name in locations:
            if prefix and not name.startswith(prefix):
                continue
            f.seek(offset)
            raw = f.read(record_length)
            r = _Reader(raw, RECORD_OVERHEAD)
            r.text()  # the name again, from the record itself
            pairs_length = r.u32()
            stop = r.at + pairs_length
            entries: dict[str, str] = {}
            while r.at < stop:
                # Read in two statements: Python evaluates the right-hand side of
                # `d[r.text()] = r.text()` first, which would swap key and value.
                key = r.text()
                entries[key] = r.text()
            found[name] = entries
    return found
