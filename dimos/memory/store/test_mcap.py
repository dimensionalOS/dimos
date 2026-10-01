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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Vector3
from dimos_generated.sensor_msgs.msg import CompressedImage, Imu
from dimos_generated.std_msgs.msg import Header
from mcap.writer import Writer
import numpy as np
import pytest

from dimos.memory.cli.dataset import open_store
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.store.mcap import McapStore
from dimos.msgs.image import image_from_array, image_to_jpeg
from dimos.protocol.cdr_mcap import CdrMcapWriter


@pytest.mark.parametrize("explicit", [False, True])
def test_cdr_channel_decodes_and_orders_by_declared_source_time(tmp_path, explicit):
    path = tmp_path / "recording.mcap"
    expected = Imu(
        header=Header(stamp=Time(sec=12, nanosec=500000000), frame_id="imu_link"),
        angular_velocity=Vector3(x=1, y=2, z=3),
    )
    earlier = Imu(header=Header(stamp=Time(sec=11, nanosec=500000000), frame_id="earlier"))
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2", library="test")
        schema = writer.register_schema(
            name=Imu.msg_name, encoding="ros2msg", data=Imu.schema.encode()
        )
        channel = writer.register_channel(
            topic="imu",
            message_encoding="cdr",
            schema_id=schema,
            metadata={"dimos.observation_time": "publish_time"},
        )
        for index, (message, stamp) in enumerate([(expected, 12500000000), (earlier, 11500000000)]):
            writer.add_message(
                channel_id=channel,
                log_time=13000000000 + index * 1000000000,
                publish_time=stamp,
                data=message.encode(),
            )
        writer.finish()
    with McapStore(path=str(path), codecs={"imu": CdrCodec(Imu)} if explicit else None) as store:
        assert store.list_streams() == ["imu"]
        observations = list(store.stream("imu").order_by("ts"))
        assert [observation.ts for observation in observations] == [11.5, 12.5]
        assert [observation.data for observation in observations] == [earlier, expected]
        assert store.stream("imu").order_by("ts", desc=True).first().data == expected


def test_standard_compressed_image_uses_cdr_and_generic_dataset_dispatch(tmp_path):
    path = tmp_path / "camera.mcap"
    image = image_from_array(
        np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8),
        encoding="rgb8",
        header=Header(frame_id="camera", stamp=Time(sec=12, nanosec=123456789)),
    )
    expected = CompressedImage(
        header=image.header, format="rgb8; jpeg compressed bgr8", data=image_to_jpeg(image)
    )
    with CdrMcapWriter(path) as writer:
        writer.write(
            "color_image",
            expected.encode(),
            schema_name=expected.msg_name,
            schema=expected.schema,
            log_time_ns=13000000000,
            publish_time_ns=12123456789,
        )
    with open_store(path) as store:
        observation = store.stream("color_image").first()
        assert observation.ts == 13.0
        assert observation.data == expected
        assert observation.data.header.stamp.nanosec == 123456789
        assert observation.data.format == "rgb8; jpeg compressed bgr8"


def test_unknown_schema_metadata_never_imports_a_payload_module(tmp_path, monkeypatch):
    path = tmp_path / "untrusted.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2", library="test")
        schema = writer.register_schema(
            name="untrusted/msg/Payload", encoding="ros2msg", data=b"uint8 value\n"
        )
        channel = writer.register_channel(
            topic="untrusted",
            message_encoding="cdr",
            schema_id=schema,
            metadata={"dimos.payload_type": "untrusted_module.Payload"},
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=b"raw payload")
        writer.finish()

    def fail_import(name):
        raise AssertionError(f"artifact metadata imported {name!r}")

    monkeypatch.setattr("dimos.memory.codecs.base.resolve_payload_type", fail_import)
    with McapStore(path=str(path)) as store:
        assert store.stream("untrusted").first().data == b"raw payload"
        assert "untrusted/msg/Payload" in store.summary()


@pytest.mark.parametrize("second_type", [Imu, CompressedImage])
def test_duplicate_topic_channels_count_together_or_reject_conflicting_schemas(
    tmp_path, second_type
):
    path = tmp_path / "channels.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="ros2")
        for cls in [Imu, second_type]:
            schema = writer.register_schema(
                name=cls.msg_name, encoding="ros2msg", data=cls.schema.encode()
            )
            channel = writer.register_channel(
                topic="sensor", message_encoding="cdr", schema_id=schema
            )
            writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=cls().encode())
        writer.finish()
    if second_type is Imu:
        with McapStore(path=str(path)) as store:
            assert store.stream("sensor").count() == 2
            assert [observation.data for observation in store.stream("sensor")] == [Imu(), Imu()]
    else:
        with pytest.raises(ValueError, match="conflicting channel schemas"):
            McapStore(path=str(path))
