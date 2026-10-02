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

from __future__ import annotations

import json
from pathlib import Path
import struct
from types import SimpleNamespace as Message

from mcap.writer import Writer
from mcap_ros2.writer import Writer as RosWriter
import numpy as np
import pytest

from dimos.memory.codecs.ros import image, pointcloud
from dimos.memory.store.mcap_recording import McapRecordingStore


def test_json_queries_are_lazy_and_source_time_ordered(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "generic.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        schema = writer.register_schema("custom.State", "jsonschema", b'{"type":"object"}')
        channel = writer.register_channel(
            "state",
            "json",
            schema,
            {
                "dimos.payload_type": "untrusted_module.Payload",
                "dimos.observation_time": "publish_time",
            },
        )
        for log_time, publish_time, value in [
            (3, 2, b'{"n":2}'),
            (4, 1, b'{"n":1}'),
            (5, 3, b"invalid"),
        ]:
            writer.add_message(channel, log_time * 10**9, value, publish_time * 10**9)
        writer.finish()

    def fail_import(name: str) -> None:
        raise AssertionError(f"untrusted file imported {name}")

    monkeypatch.setattr("dimos.memory.codecs.base.importlib.import_module", fail_import)
    with McapRecordingStore(path=str(path)) as store:
        stream = store.stream("state")
        assert stream.count() == 3
        assert stream.get_time_range() == (1, 3)
        assert [obs.ts for obs in stream.order_by("ts")] == [1, 2, 3]
        assert stream.order_by("ts").first().data == {"n": 1}
        assert stream.order_by("ts").limit(2).count() == 2
        with pytest.raises(json.JSONDecodeError):
            _ = stream.order_by("ts", desc=True).first().data


def test_unknown_ros_schema_remains_inspectable(tmp_path: Path) -> None:
    path = tmp_path / "custom.mcap"
    with RosWriter(str(path)) as writer:
        schema = writer.register_msgdef("custom_msgs/msg/State", "string label\nfloat64 value\n")
        writer.write_message("state", schema, {"label": "ready", "value": 2.5}, log_time=10**9)
    with McapRecordingStore(path=str(path)) as store:
        observation = store.stream("state").first()
        assert observation.ts == 1
        assert observation.data.label == "ready" and observation.data.value == 2.5


@pytest.mark.parametrize(
    "schema_encoding,message_encoding,data",
    [(None, "lcm", b""), ("jsonschema", "json", b""), ("protobuf", "protobuf", b"schema")],
)
def test_missing_or_unsupported_schema_is_rejected(
    tmp_path: Path, schema_encoding: str | None, message_encoding: str, data: bytes
) -> None:
    path = tmp_path / "invalid.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start(profile="dimos")
        schema = writer.register_schema("unknown", schema_encoding, data) if schema_encoding else 0
        writer.register_channel("unknown", message_encoding, schema)
        writer.finish()
    with pytest.raises(ValueError, match="schema"):
        McapRecordingStore(path=str(path))


@pytest.mark.parametrize("bigendian", [False, True])
def test_image_preserves_padded_depth_rows(bigendian: bool) -> None:
    endian = ">" if bigendian else "<"
    message = Message(
        header=Message(stamp=Message(sec=12, nanosec=500_000_000), frame_id="camera"),
        width=2,
        height=2,
        step=6,
        is_bigendian=bigendian,
        encoding="16UC1",
        data=struct.pack(endian + "HHxxHHxx", 1, 1000, 65535, 42),
    )
    result = image(message)
    np.testing.assert_array_equal(result.data, [[1, 1000], [65535, 42]])
    assert result.ts == 12.5 and result.frame_id == "camera"
    assert result.data.dtype == np.uint16


@pytest.mark.parametrize("bigendian", [False, True])
def test_pointcloud_preserves_padding_endian_intensity_and_color(bigendian: bool) -> None:
    endian = ">" if bigendian else "<"
    message = Message(
        header=Message(stamp=Message(sec=12, nanosec=0), frame_id="lidar"),
        height=2,
        width=1,
        point_step=24,
        row_step=28,
        is_bigendian=bigendian,
        fields=[
            Message(name=name, offset=offset, datatype=kind, count=1)
            for name, offset, kind in [
                ("x", 4, 7),
                ("y", 8, 7),
                ("z", 12, 7),
                ("intensity", 16, 7),
                ("rgb", 20, 7),
            ]
        ],
        data=struct.pack(
            endian + "4xffffI4x4xffffI4x", 1, 2, 3, 11, 0xFF0000, 4, 5, 6, 22, 0x00FF00
        ),
    )
    cloud = pointcloud(message)
    np.testing.assert_array_equal(cloud.points(), [[1, 2, 3], [4, 5, 6]])
    np.testing.assert_array_equal(cloud.intensities_f32(), [11, 22])
    np.testing.assert_array_equal(
        cloud.pointcloud_tensor.point["colors"].numpy(), [[1, 0, 0], [0, 1, 0]]
    )
