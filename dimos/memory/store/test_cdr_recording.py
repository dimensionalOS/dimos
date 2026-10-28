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

"""Read-only CDR stores decode self-describing MCAP recordings."""

from contextlib import contextmanager

from dimos_cdrgen.registry import encode, schema
from mcap.writer import Writer
import pytest

from dimos.memory.codecs.base import codec_for, codec_from_id, codec_id
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.store.cdr_mcap import CdrMcapStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header


def message(sec=17):
    return LineSegments3D(
        header=Header(stamp=Time(sec=sec, nanosec=250_000_000), frame_id="imu"),
        segments=[LineSegment3D(Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), 0.75)],
    )


@contextmanager
def mcap_fixture(path):
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2")
        try:
            yield writer
        finally:
            writer.finish()


def test_cdr_codec_restores_generated_values():
    expected = message()
    codec = codec_for(LineSegments3D)
    assert isinstance(codec, CdrCodec)
    assert codec_id(codec) == "cdr"
    rebuilt = codec_from_id("cdr", f"{LineSegments3D.__module__}.{LineSegments3D.__qualname__}")
    assert rebuilt.decode(encode(expected)) == expected
    with pytest.raises(TypeError, match="declared type"):
        codec.encode(Header(Time(0, 0), "wrong-type"))
    with pytest.raises(TypeError, match="Unsupported"):
        CdrCodec(str)


@pytest.mark.parametrize("payload", [b"malformed", b"\x00\x07\x00\x00" + bytes(4)])
def test_cdr_storage_rejects_invalid_encapsulation(payload):
    with pytest.raises(ValueError, match="XCDR1 encapsulation"):
        CdrCodec(LineSegments3D).decode(payload)


def test_mismatched_known_schema_is_rejected_before_decoding(tmp_path):
    path = tmp_path / "bad.mcap"
    with mcap_fixture(path) as writer:
        schema_id = writer.register_schema(
            LineSegments3D.__msgtype__, "ros2msg", b"int32 changed\n"
        )
        channel = writer.register_channel("imu", "cdr", schema_id)
        writer.add_message(channel, log_time=1, publish_time=1, data=encode(message()))
    with pytest.raises(ValueError, match="CDR schema mismatch"):
        CdrMcapStore(path=str(path))


def test_unknown_cdr_schema_never_uses_a_legacy_topic_decoder(tmp_path):
    path = tmp_path / "unknown.mcap"
    with mcap_fixture(path) as writer:
        schema_id = writer.register_schema("future_msgs/msg/New", "ros2msg", b"uint32 value\n")
        channel = writer.register_channel("dimos/imu/sensor_msgs.Imu", "cdr", schema_id)
        writer.add_message(channel, log_time=1, publish_time=1, data=b"unknown bytes")
    with CdrMcapStore(path=str(path)) as store:
        observations = list(store.stream("dimos_imu_sensor_msgs.Imu"))
        assert observations[0].data == b"unknown bytes"
        assert "raw bytes: future_msgs/msg/New" in store.summary()


def test_conflicting_channel_schemas_are_rejected(tmp_path):
    path = tmp_path / "conflict.mcap"
    with mcap_fixture(path) as writer:
        for text in (b"uint32 value\n", b"uint64 value\n"):
            schema_id = writer.register_schema("future_msgs/msg/New", "ros2msg", text)
            channel = writer.register_channel("same", "cdr", schema_id)
            writer.add_message(channel, log_time=1, publish_time=1, data=b"")
    with pytest.raises(ValueError, match="Conflicting schemas"):
        CdrMcapStore(path=str(path))


def test_cdr_store_rejects_non_cdr_channels(tmp_path):
    path = tmp_path / "legacy.mcap"
    with mcap_fixture(path) as writer:
        channel = writer.register_channel("imu", "lcm", 0)
        writer.add_message(channel, log_time=1, publish_time=1, data=b"legacy")
    with pytest.raises(ValueError, match="requires CDR channels"):
        CdrMcapStore(path=str(path))


def test_cdr_store_remaps_topics_and_queries_source_time(tmp_path):
    path = tmp_path / "source-order.mcap"
    with mcap_fixture(path) as writer:
        schema_id = writer.register_schema(
            LineSegments3D.__msgtype__, "ros2msg", schema(LineSegments3D.__msgtype__).encode()
        )
        channel = writer.register_channel(
            "/sensors/segments",
            "cdr",
            schema_id,
            metadata={"dimos.observation_time": "publish_time"},
        )
        for log_ns, sec in ((1, 20), (2, 10)):
            writer.add_message(
                channel,
                log_time=log_ns,
                publish_time=sec * 1_000_000_000,
                data=encode(message(sec)),
            )
    with CdrMcapStore(path=str(path), streams={"segments": "/sensors/segments"}) as store:
        stream = store.stream("segments")
        assert store.list_streams() == ["segments"]
        assert stream.count() == 2
        assert stream.order_by("ts").first().data == message(10)
        assert stream.order_by("ts", desc=True).first().data == message(20)
        assert stream.order_by("id", desc=True).first().data == message(10)
        with pytest.raises(NotImplementedError, match="read-only"):
            stream.append(message(30), ts=30)
