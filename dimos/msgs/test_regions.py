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

from copy import deepcopy
import subprocess
import sys

from dimos_cdrgen.registry import decode, encode, schema
from mcap.reader import make_reader
import numpy as np
import pytest

from dimos.messages.dimos_msgs.msg import (
    LineSegment3D,
    LineSegments3D,
    LineSegments3DRegion,
    PointCloudRegion,
    RegionBounds,
)
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.cdr_mcap import CdrMcapWriter


def region_values(region_id):
    header = Header(frame_id="map", stamp=time_from_nanoseconds(1700000000123456789))
    cloud = pointcloud_from_xyz(np.array([[1, 2, 3]], dtype=np.float32), header=deepcopy(header))
    lines = LineSegments3D(
        header=deepcopy(header),
        segments=[LineSegment3D(start=Point(x=1, y=2, z=3), end=Point(x=4, y=5, z=6), weight=0.75)],
    )
    return [
        PointCloudRegion(header=deepcopy(header), region_id=region_id, cloud=cloud),
        RegionBounds(
            header=deepcopy(header),
            region_id=region_id,
            center=Point(x=1, y=2, z=0),
            radius=3.5,
            z_min=-1.25,
            z_max=4.5,
        ),
        LineSegments3DRegion(header=deepcopy(header), region_id=region_id, lines=lines),
    ]


@pytest.mark.parametrize("region_id", [-(2**31), 0, 2**31 - 1])
@pytest.mark.parametrize("index", range(3))
def test_region_id_and_canonical_dependency_types_survive_cdr(region_id, index):
    value = region_values(region_id)[index]
    result = decode(encode(value), type(value))
    assert encode(result) == encode(value)
    assert result.region_id == region_id
    assert type(result.header) is Header
    assert to_nanoseconds(result.header.stamp) == 1700000000123456789
    if index == 0:
        assert type(result.cloud) is PointCloud2 and type(result.cloud.header) is Header
    elif index == 1:
        assert type(result.center) is Point
    else:
        assert type(result.lines) is LineSegments3D
        assert type(result.lines.segments[0].start) is Point


def test_region_mcap_embeds_all_dependency_schemas_for_an_independent_reader(tmp_path):
    path = tmp_path / "regions.mcap"
    values = region_values(-2147483648)
    with CdrMcapWriter(path) as writer:
        for value in values:
            name = value.__msgtype__
            writer.write(
                "/" + name,
                encode(value),
                schema_name=name,
                schema=schema(name),
                log_time_ns=1700000000123456790,
                publish_time_ns=1700000000123456789,
            )
    with path.open("rb") as stream:
        messages = list(make_reader(stream, validate_crcs=True).iter_messages())
    assert len(messages) == 3
    for stored_schema, channel, message in messages:
        assert stored_schema.encoding == "ros2msg" and channel.message_encoding == "cdr"
        assert message.publish_time == 1700000000123456789
        assert "MSG: std_msgs/Header" in stored_schema.data.decode()
    # The independent type store owns a global module; use a separate interpreter
    # so it cannot replace canonical Python message class identities in this test run.
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            """
from mcap.reader import make_reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
import sys
reference = get_typestore(Stores.EMPTY)
with open(sys.argv[1], "rb") as stream:
    for schema, _, message in make_reader(stream).iter_messages():
        reference.register(get_types_from_msg(schema.data.decode(), schema.name))
        value = reference.deserialize_cdr(message.data, schema.name)
        assert value.region_id == -2147483648
        assert value.header.stamp.sec == 1700000000
        assert value.header.stamp.nanosec == 123456789
        if schema.name.endswith("PointCloudRegion"):
            assert value.cloud.width == 1 and value.cloud.point_step == 12
        elif schema.name.endswith("RegionBounds"):
            assert (value.center.x,value.center.y,value.radius,value.z_min,value.z_max) == (1,2,3.5,-1.25,4.5)
        else:
            assert value.lines.segments[0].weight == 0.75
            assert value.lines.segments[0].end.z == 6
""",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
