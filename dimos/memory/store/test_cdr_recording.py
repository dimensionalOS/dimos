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

"""CDR passes through real mem2 storage and self-describing MCAP replay."""

from pathlib import Path
import subprocess
import sys

from dimos_cdrgen.registry import encode, schema
from mcap.reader import make_reader
import pytest

from dimos.memory.codecs.base import codec_for, codec_from_id, codec_id
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.cdr_mcap import CdrMcapWriter
from dimos.types.timestamped import message_timestamp


def message(sec=17):
    return LineSegments3D(
        header=Header(stamp=Time(sec=sec, nanosec=250_000_000), frame_id="imu"),
        segments=[LineSegment3D(Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), 0.75)],
    )


def test_sqlite_close_reopen_and_mcap_replay_preserve_type_bytes_and_source_time(tmp_path):
    expected = message()
    codec = codec_for(LineSegments3D)
    assert isinstance(codec, CdrCodec)
    assert codec_id(codec) == "cdr"
    rebuilt = codec_from_id("cdr", f"{LineSegments3D.__module__}.{LineSegments3D.__qualname__}")
    assert rebuilt.decode(codec.encode(expected)) == expected
    database = tmp_path / "memory.db"
    with SqliteStore(path=str(database)) as store:
        store.stream("imu", LineSegments3D).append(expected, ts=message_timestamp(expected))
    path = tmp_path / "recording.mcap"
    with SqliteStore(path=str(database), must_exist=True) as store, CdrMcapWriter(path) as writer:
        writer.register_stream(
            "imu",
            schema_name=LineSegments3D.__msgtype__,
            schema=schema(LineSegments3D.__msgtype__),
            metadata={"dimos.observation_time": "publish_time"},
        )
        assert list(store.replay().streams.imu.iterate_ts()) == [(17.25, expected)]
        values = list(store.streams.imu)
        assert len(values) == 1
        assert type(values[0].data) is LineSegments3D
        assert values[0].data == expected
        assert values[0].ts == 17.25
        writer.write(
            "imu",
            encode(values[0].data),
            schema_name=LineSegments3D.__msgtype__,
            schema=schema(LineSegments3D.__msgtype__),
            log_time_ns=20_000_000_000,
            publish_time_ns=17_250_000_000,
        )
    with path.open("rb") as stream:
        recorded_schema, channel, recorded = next(make_reader(stream).iter_messages())
        assert channel.message_encoding == "cdr"
        assert recorded_schema.encoding == "ros2msg"
        assert "MSG: std_msgs/Header" in recorded_schema.data.decode()
        assert "MSG: builtin_interfaces/Time" in recorded_schema.data.decode()
        assert recorded.data == encode(expected)
        assert recorded.log_time == 20_000_000_000
        assert recorded.publish_time == 17_250_000_000
    with McapStore(path=str(path)) as store:
        observations = list(store.streams.imu)
        assert len(observations) == 1
        assert observations[0].ts == 17.25
        assert type(observations[0].data) is LineSegments3D
        assert type(observations[0].data.header) is Header
        assert observations[0].data == expected


def test_mismatched_known_schema_is_rejected_before_decoding(tmp_path):
    path = tmp_path / "bad.mcap"
    with CdrMcapWriter(path) as writer:
        writer.write(
            "imu",
            encode(message()),
            schema_name=LineSegments3D.__msgtype__,
            schema="int32 changed\n",
            log_time_ns=1,
        )
    with pytest.raises(ValueError, match="CDR schema mismatch"):
        McapStore(path=str(path))


def test_unknown_cdr_schema_never_uses_a_legacy_topic_decoder(tmp_path):
    path = tmp_path / "unknown.mcap"
    with CdrMcapWriter(path) as writer:
        writer.write(
            "dimos/imu/sensor_msgs.Imu",
            b"unknown bytes",
            schema_name="future_msgs/msg/New",
            schema="uint32 value\n",
            log_time_ns=1,
        )
    with McapStore(path=str(path)) as store:
        observations = list(store.streams.imu)
        assert observations[0].data == b"unknown bytes"
        assert "raw bytes: future_msgs/msg/New" in store.summary()


def test_conflicting_channel_schemas_are_rejected(tmp_path):
    path = tmp_path / "conflict.mcap"
    with CdrMcapWriter(path) as writer:
        for text in ("uint32 value\n", "uint64 value\n"):
            writer.write("same", b"", schema_name="future_msgs/msg/New", schema=text, log_time_ns=1)
    with pytest.raises(ValueError, match="Conflicting schemas"):
        McapStore(path=str(path))


def test_generated_zero_stamp_is_preserved_and_wrong_codec_value_fails():
    value = message(0)
    value.header.stamp.nanosec = 0
    assert message_timestamp(value) == 0.0
    assert message_timestamp(object()) is None
    with pytest.raises(TypeError, match="Expected"):
        CdrCodec(LineSegments3D).encode(Header(Time(0, 0), "wrong-type"))
    with pytest.raises(TypeError, match="installed generated"):
        CdrCodec(str)


def test_documented_recording_example_runs_and_refuses_to_overwrite(tmp_path):
    root = Path(__file__).resolve().parents[3]
    script = root / "examples/message-recording/demo_cdr_mem2.py"
    destination = tmp_path / "demo"
    result = subprocess.run(
        [sys.executable, str(script), str(destination)],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )
    assert "PASS: mem2 SQLite close/reopen" in result.stdout
    assert "PASS: MCAP embeds complete ros2msg schemas" in result.stdout
    original = {name: (destination / name).read_bytes() for name in ("memory.db", "recording.mcap")}
    repeated = subprocess.run(
        [sys.executable, str(script), str(destination)],
        cwd=tmp_path,
        check=False,
        capture_output=True,
        text=True,
    )
    assert repeated.returncode != 0
    assert "FileExistsError" in repeated.stderr
    assert {name: (destination / name).read_bytes() for name in original} == original
