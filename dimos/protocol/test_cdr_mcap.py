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

from mcap.reader import make_reader
from mcap.records import Chunk
from mcap.stream_reader import StreamReader
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.message_codegen.definitions import Definitions
from dimos.protocol.cdr_mcap import CdrMcapWriter


@pytest.fixture
def point_message():
    name = "geometry_msgs/msg/Point"
    schema = Definitions([]).schema(name)
    reference = get_typestore(Stores.ROS2_JAZZY)
    value = reference.types[name](x=1.25, y=-2.5, z=3.0)
    return name, schema, bytes(reference.serialize_cdr(value, name))


def test_embeds_schemas_and_preserves_cdr_timestamps_and_sequence(tmp_path, point_message):
    name, schema, payload = point_message
    path = tmp_path / "capture.mcap"
    with CdrMcapWriter(path) as writer:
        for topic, timestamp in (("/first", 100), ("/second", 200)):
            writer.write(
                topic,
                payload,
                schema_name=name,
                schema=schema,
                log_time_ns=timestamp,
                publish_time_ns=0,
                sequence=7,
            )
    with path.open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True)
        assert reader.get_header().profile == "ros2"
        summary = reader.get_summary()
        assert len(summary.schemas) == 1
        assert len(summary.channels) == 2
        messages = list(reader.iter_messages())
    assert [message.log_time for _, _, message in messages] == [100, 200]
    reference = get_typestore(Stores.EMPTY)
    for stored_schema, channel, message in messages:
        assert stored_schema.encoding == "ros2msg"
        assert stored_schema.name == name
        assert stored_schema.data.decode() == schema
        assert channel.message_encoding == "cdr"
        assert channel.metadata["offered_qos_profiles"] == "[]"
        assert message.data == payload
        assert message.publish_time == 0
        assert message.sequence == 7
        reference.register(get_types_from_msg(stored_schema.data.decode(), stored_schema.name))
        decoded = reference.deserialize_cdr(message.data, name)
        assert (decoded.x, decoded.y, decoded.z) == (1.25, -2.5, 3.0)


def test_unstamped_messages_use_reception_time_and_chunks_are_compressed(tmp_path, point_message):
    name, schema, payload = point_message
    path = tmp_path / "capture.mcap"
    with CdrMcapWriter(path) as writer:
        writer.write("/point", payload, schema_name=name, schema=schema, log_time_ns=123)
    with path.open("rb") as stream:
        messages = list(make_reader(stream).iter_messages())
    assert messages[0][2].publish_time == 123
    with path.open("rb") as stream:
        chunks = [
            record
            for record in StreamReader(stream, emit_chunks=True).records
            if isinstance(record, Chunk)
        ]
    assert chunks
    assert all(chunk.compression == "zstd" for chunk in chunks)


def test_context_finalizes_recording_when_producer_raises(tmp_path, point_message):
    name, schema, payload = point_message
    path = tmp_path / "capture.mcap"
    with pytest.raises(RuntimeError, match="producer"), CdrMcapWriter(path) as writer:
        writer.write("/point", payload, schema_name=name, schema=schema, log_time_ns=123)
        raise RuntimeError("producer failed")
    with path.open("rb") as stream:
        assert make_reader(stream).get_summary().statistics.message_count == 1
    with pytest.raises(ValueError, match="closed"):
        writer.write("/point", payload, schema_name=name, schema=schema, log_time_ns=124)


def test_nested_schema_uses_resolvable_resource_names():
    schema = Definitions([]).schema("sensor_msgs/msg/Image")

    assert "MSG: std_msgs/Header\n" in schema
    assert "MSG: builtin_interfaces/Time\n" in schema
    assert set(get_types_from_msg(schema, "sensor_msgs/msg/Image")) == {
        "sensor_msgs/msg/Image",
        "std_msgs/msg/Header",
        "builtin_interfaces/msg/Time",
    }
