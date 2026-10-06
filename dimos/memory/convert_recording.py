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

"""Offline migration of explicitly typed legacy memory recordings to generated CDR.

Never instantiate classes named by a recording. Legacy pickle and external blob
stores are deliberately unsupported. Output publication is exclusive and atomic.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
import importlib
import io
import json
import math
import os
from pathlib import Path
import sqlite3
import tempfile
from typing import Any

import lz4.frame
from mcap.reader import make_reader
from mcap.records import Attachment, Channel, Metadata, Schema
from mcap.stream_reader import StreamReader

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.utils.validation import validate_identifier
from dimos.msgs.helpers import resolve_msg_type
from dimos.protocol.cdr_mcap import CdrMcapWriter

# Deliberate, reviewable mappings. No suffix-based arbitrary Python imports.
TYPES = {
    "std_msgs": ("String", "Header", "Bool", "Float32", "Float64", "Int32", "UInt32"),
    "builtin_interfaces": ("Time", "Duration"),
    "geometry_msgs": (
        "Point",
        "PointStamped",
        "Vector3",
        "Quaternion",
        "Pose",
        "PoseStamped",
        "PoseWithCovariance",
        "PoseWithCovarianceStamped",
        "Transform",
        "TransformStamped",
        "Twist",
        "TwistStamped",
        "TwistWithCovariance",
        "TwistWithCovarianceStamped",
        "Wrench",
        "WrenchStamped",
    ),
    "sensor_msgs": (
        "Image",
        "CompressedImage",
        "PointCloud2",
        "PointField",
        "Imu",
        "CameraInfo",
        "JointState",
        "Joy",
    ),
    "nav_msgs": ("Odometry", "Path", "OccupancyGrid", "MapMetaData"),
    "tf2_msgs": ("TFMessage",),
}
ROS_TYPES = {f"{package}/msg/{name}" for package, names in TYPES.items() for name in names}
LEGACY_TYPES = {
    path: f"{package}/msg/{name}"
    for package, names in TYPES.items()
    for name in names
    for path in (
        f"dimos.msgs.{package}.{name}.{name}",
        f"dimos.msgs.{package}.{name}",
        f"dimos_lcm.{package}.{name}.{name}",
        f"lcm_msgs.{package}.{name}.{name}",
    )
}
CODECS = {"lcm", "jpeg", "json", "cdr", "lz4+lcm", "lz4+jpeg", "lz4+json"}


@dataclass(frozen=True)
class Stream:
    name: str
    ros_type: str
    codec: str
    schema: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def target(self) -> Any:
        name = "sensor_msgs/msg/CompressedImage" if self.codec.endswith("jpeg") else self.ros_type
        result = resolve_msg_type(name)
        if result is None:
            raise ValueError(f"Generated message package does not provide {name}")
        return result


@dataclass
class Row:
    stream: Stream
    data: bytes
    log_ns: int
    publish_ns: int | None
    sequence: int | None
    observation_ts: float
    metadata: dict[str, Any] = field(default_factory=dict)


def _stream(name: str, payload: str, codec: str, schema: str = "", **metadata: Any) -> Stream:
    ros_type = payload if payload in ROS_TYPES else LEGACY_TYPES.get(payload)
    if ros_type is None or codec not in CODECS:
        raise ValueError(
            f"{name}: unsupported type={payload!r}, codec={codec!r} (pickle is never loaded)"
        )
    if codec.endswith("jpeg") and ros_type != "sensor_msgs/msg/Image":
        raise ValueError(f"{name}: jpeg requires the legacy Image envelope")
    if codec.endswith("json") and ros_type != "std_msgs/msg/String":
        raise ValueError(f"{name}: json requires explicitly declared std_msgs/String")
    result = Stream(name, ros_type, codec, schema, metadata)
    if codec == "cdr" and schema != result.target.schema:
        raise ValueError(f"{name}: CDR schema differs from the installed generated definition")
    return result


def _mcap_stream(channel: Channel, schemas: dict[int, Schema]) -> Stream:
    schema = schemas.get(channel.schema_id)
    payload = channel.metadata.get("dimos.payload_type", "")
    if channel.message_encoding == "cdr" and schema is not None and schema.encoding == "ros2msg":
        payload = schema.name
    if not payload and channel.topic.startswith("dimos/"):
        # Historical DimOS wire names encode the type as the last segment.
        typename = channel.topic.rsplit("/", 1)[-1]
        matches = [t for t in ROS_TYPES if t.rsplit("/", 1)[-1] == typename]
        if len(matches) == 1:
            payload = matches[0]
    return _stream(
        channel.topic,
        payload,
        channel.message_encoding,
        schema.data.decode() if schema else "",
        channel_metadata=channel.metadata,
    )


@contextmanager
def read_mcap(path: Path) -> Iterator[tuple[list[Stream], Iterator[Row]]]:
    with path.open("rb") as handle:
        schemas: dict[int, Schema] = {}
        channels: dict[int, Channel] = {}
        metadata_records: list[dict[str, Any]] = []
        for record in StreamReader(handle, validate_crcs=True).records:
            if isinstance(record, Attachment):
                raise ValueError(
                    "MCAP attachments need a separate export; refusing to discard them"
                )
            if isinstance(record, Metadata):
                metadata_records.append({"name": record.name, "metadata": record.metadata})
            if isinstance(record, Schema):
                schemas[record.id] = record
            elif isinstance(record, Channel):
                channels[record.id] = record
        streams: dict[int, Stream] = {}
        errors = []
        names: dict[str, Stream] = {}
        for cid, channel in channels.items():
            try:
                stream = _mcap_stream(channel, schemas)
                if metadata_records:
                    stream.metadata["source_mcap_metadata"] = metadata_records
                if stream.name in names and names[stream.name] != stream:
                    raise ValueError(f"{stream.name}: conflicting MCAP channel definitions")
                names[stream.name] = stream
                streams[cid] = stream
            except ValueError as exc:
                errors.append(str(exc))
        if errors:
            raise ValueError("Unsupported streams; nothing converted:\n" + "\n".join(errors))
        handle.seek(0)

        def rows() -> Iterator[Row]:
            for _, channel, msg in make_reader(handle, validate_crcs=True).iter_messages(
                log_time_order=False
            ):
                ts = (
                    msg.publish_time
                    if channel.metadata.get("dimos.observation_time") == "publish_time"
                    else msg.log_time
                )
                yield Row(
                    streams[channel.id],
                    msg.data,
                    msg.log_time,
                    msg.publish_time,
                    msg.sequence,
                    ts / 1e9,
                )

        yield list(names.values()), rows()


@contextmanager
def read_sqlite(path: Path) -> Iterator[tuple[list[Stream], Iterator[Row]]]:
    conn = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    try:
        conn.execute("PRAGMA trusted_schema=OFF")
        conn.execute("BEGIN")  # Stable read-only snapshot; never reconstruct registry components.
        streams = []
        stream_counts: dict[str, int] = {}
        errors = []
        for name, text in conn.execute("SELECT name, config FROM _streams ORDER BY name"):
            try:
                validate_identifier(name)
                config = json.loads(text)
                blob = config.get("blob_store", {})
                if (
                    blob.get("class", "").replace("dimos.memory2.", "dimos.memory.")
                    != "dimos.memory.blobstore.sqlite.SqliteBlobStore"
                    or blob.get("config", {}).get("path") is not None
                ):
                    raise ValueError(f"{name}: external/custom blob store is unsupported")
                stream = _stream(name, config["payload_module"], config["codec_id"])
                if conn.execute(
                    "SELECT 1 FROM sqlite_master WHERE name=?", (name + "_vec",)
                ).fetchone():
                    raise ValueError(
                        f"{name}: vector embeddings need a separate migration; refusing to discard them"
                    )
                stream_counts[name] = conn.execute(f'SELECT count(*) FROM "{name}"').fetchone()[0]
                # Missing blobs must not disappear through an inner join.
                missing = (
                    0
                    if stream_counts[name] == 0
                    else conn.execute(
                        f'SELECT count(*) FROM "{name}" m LEFT JOIN "{name}_blob" b ON b.id=m.id WHERE b.data IS NULL'
                    ).fetchone()[0]
                )
                if missing:
                    raise ValueError(f"{name}: {missing} observations have no payload blob")
                streams.append(stream)
            except (ValueError, KeyError, sqlite3.Error) as exc:
                errors.append(str(exc))
        if errors:
            raise ValueError("Unsupported streams; nothing converted:\n" + "\n".join(errors))

        def rows() -> Iterator[Row]:
            for stream in streams:
                name = stream.name
                if not stream_counts[name]:
                    continue
                sql = f'SELECT m.id,m.ts,m.value,m.pose_x,m.pose_y,m.pose_z,m.pose_qx,m.pose_qy,m.pose_qz,m.pose_qw,json(m.tags),b.data FROM "{name}" m JOIN "{name}_blob" b ON b.id=m.id ORDER BY m.id'
                for record in conn.execute(sql):
                    ident, ts, value, *rest = record
                    if not math.isfinite(ts) or ts < 0:
                        raise ValueError(f"{name}: invalid observation timestamp {ts}")
                    yield Row(
                        stream,
                        rest[-1],
                        round(ts * 1e9),
                        None,
                        None,
                        ts,
                        {
                            "sqlite_id": ident,
                            "value": value,
                            "pose": rest[:7],
                            "tags": json.loads(rest[-2]),
                        },
                    )

        yield streams, rows()
    finally:
        conn.close()


def _convert_fields(old: Any, sequences: dict[str, int], path: str = "") -> Any:
    if isinstance(old, (list, tuple)):
        return [_convert_fields(item, sequences, f"{path}[{i}]") for i, item in enumerate(old)]
    if not hasattr(old, "__slots__"):
        return old
    package = type(old).__module__.split(".")[-2]
    name = type(old).__name__
    if package == "std_msgs" and name in {"Time", "Duration"}:
        package = "builtin_interfaces"
    ros_type = f"{package}/msg/{name}"
    if ros_type not in ROS_TYPES:
        raise ValueError(f"Unsupported nested LCM type {ros_type}")
    target = resolve_msg_type(ros_type)
    if target is None:
        raise ValueError(f"Missing generated {ros_type}")
    values = {}
    for key in old.__slots__:
        value = getattr(old, key)
        if ros_type == "std_msgs/msg/Header" and key == "seq":
            sequences[f"{path}.seq".lstrip(".")] = value
            continue
        if key.endswith("_length") and key[:-7] in old.__slots__:
            if value != len(getattr(old, key[:-7])):
                raise ValueError(f"Invalid LCM sequence length at {path}.{key}")
            continue
        new_key = "nanosec" if package == "builtin_interfaces" and key == "nsec" else key
        if new_key not in target.__annotations__:
            raise ValueError(f"No lossless field mapping for {ros_type}.{key}")
        values[new_key] = _convert_fields(value, sequences, f"{path}.{key}".lstrip("."))
    return target(**values)


def decode(row: Row) -> tuple[Any, dict[str, int]]:
    codec, data = row.stream.codec, row.data
    if codec.startswith("lz4+"):
        data = lz4.frame.decompress(data)
        codec = codec[4:]
    if codec == "cdr":
        return row.stream.target.decode(data), {}
    if codec == "json":
        text = data.decode("utf-8")
        json.loads(
            text,
            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"Invalid JSON {value}")),
        )
        return row.stream.target(data=text), {}
    package, _, name = row.stream.ros_type.split("/")
    try:
        module = importlib.import_module(f"dimos_lcm.{package}.{name}")
    except ImportError as exc:
        raise ValueError(
            "LCM input needs the optional decoder: pip install dimos-lcm==0.1.4"
        ) from exc
    buffer = io.BytesIO(data)
    old = getattr(module, name).lcm_decode(buffer)  # Fingerprint verified by the official decoder.
    if buffer.tell() != len(data):
        raise ValueError(f"{row.stream.name}: trailing bytes after LCM message")
    sequences: dict[str, int] = {}
    if codec == "jpeg":
        if old.encoding != "jpeg":
            raise ValueError(f"{row.stream.name}: jpeg codec contains {old.encoding!r}")
        h = _convert_fields(old.header, sequences, "header")
        return row.stream.target(header=h, format="jpeg", data=old.data), sequences
    if name == "Image" and old.encoding == "jpeg":
        raise ValueError(f"{row.stream.name}: JPEG Image requires explicit jpeg codec metadata")
    return _convert_fields(old, sequences), sequences


def convert(source: Path, destination: Path) -> dict[str, Any]:
    source, destination = source.resolve(), destination.absolute()
    report = destination.with_name(destination.name + ".conversion.jsonl")
    if destination.exists() or report.exists() or destination.is_symlink() or report.is_symlink():
        raise FileExistsError(f"Refusing to overwrite {destination} or its conversion report")
    if destination.suffix not in {".mcap", ".db"}:
        raise ValueError("Output must end in .mcap or .db")
    if source.suffix not in {".mcap", ".db", ".sqlite"} or not source.is_file():
        raise ValueError(
            "Input must be a local MCAP or memory SQLite file; pickle directories are unsupported"
        )
    reader = read_mcap if source.suffix == ".mcap" else read_sqlite
    with reader(source) as (streams, rows):
        if not streams:
            raise ValueError("Input contains no declared streams")
        for stream in streams:
            if destination.suffix != ".mcap":
                validate_identifier(stream.name)
            _ = stream.target  # Validate generated type availability before creating output.
        with tempfile.TemporaryDirectory(prefix=".cdr-convert-", dir=destination.parent) as tmp:
            temp = Path(tmp) / destination.name
            sidecar = Path(tmp) / report.name
            counts: Counter[str] = Counter({stream.name: 0 for stream in streams})
            writer: Any = (
                CdrMcapWriter(temp)
                if destination.suffix == ".mcap"
                else SqliteStore(path=str(temp))
            )
            with writer, sidecar.open("w") as audit:
                audit.write(
                    json.dumps({"source": str(source), "streams": [s.__dict__ for s in streams]})
                    + "\n"
                )
                if destination.suffix == ".mcap":
                    for stream in streams:
                        writer.register_stream(
                            stream.name,
                            schema_name=stream.target.msg_name,
                            schema=stream.target.schema,
                            metadata={
                                **stream.metadata.get("channel_metadata", {}),
                                "dimos.payload_type": f"{stream.target.__module__}.{stream.target.__name__}",
                            },
                        )
                else:
                    for stream in streams:
                        writer.stream(stream.name, stream.target, codec="cdr")
                for row in rows:
                    try:
                        value, sequences = decode(row)
                        payload = value.encode()
                    except Exception as exc:
                        raise ValueError(
                            f"{row.stream.name}[{counts[row.stream.name]}]: {exc}"
                        ) from exc
                    header = getattr(value, "header", None)
                    source_ns = (
                        (header.stamp.sec * 1_000_000_000 + header.stamp.nanosec)
                        if header is not None
                        else None
                    )
                    publish_ns = (
                        row.publish_ns
                        if row.publish_ns is not None
                        else (source_ns if source_ns is not None else row.log_ns)
                    )
                    sequence = (
                        row.sequence
                        if row.sequence is not None
                        else sequences.get("header.seq", sequences.get("seq", 0))
                    )
                    if not (
                        0 <= row.log_ns < 2**64
                        and 0 <= publish_ns < 2**64
                        and 0 <= sequence < 2**32
                    ):
                        raise ValueError(
                            f"{row.stream.name}: time/sequence cannot be represented in MCAP"
                        )
                    meta = {
                        "stream": row.stream.name,
                        "index": counts[row.stream.name],
                        "log_time_ns": row.log_ns,
                        "publish_time_ns": publish_ns,
                        "source_header_time_ns": source_ns,
                        "sequence": sequence,
                        "legacy_header_sequences": sequences,
                        "original": row.metadata,
                    }
                    if destination.suffix == ".mcap":
                        writer.write(
                            row.stream.name,
                            payload,
                            schema_name=value.msg_name,
                            schema=value.schema,
                            log_time_ns=row.log_ns,
                            publish_time_ns=publish_ns,
                            sequence=sequence,
                        )
                    else:
                        writer.stream(row.stream.name).append(
                            value,
                            ts=row.observation_ts,
                            pose=tuple(row.metadata["pose"])
                            if row.metadata.get("pose")
                            and all(v is not None for v in row.metadata["pose"])
                            else None,
                            tags={"cdr_conversion": meta},
                        )
                    counts[row.stream.name] += 1
                    audit.write(json.dumps(meta) + "\n")
                summary = {
                    "counts": dict(counts),
                    "total": sum(counts.values()),
                    "output": str(destination),
                    "report": str(report),
                }
                audit.write(json.dumps({"summary": summary}) + "\n")
            # Hard-link publication is atomic and never overwrites a racing writer.
            os.link(sidecar, report)
            try:
                os.link(temp, destination)
            except BaseException:
                report.unlink()
                raise
    return summary
