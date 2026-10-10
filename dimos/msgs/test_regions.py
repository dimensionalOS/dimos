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
from dimos_generated.dimos_msgs.msg import (
    LineSegment3D,
    LineSegments3D,
    RegionBounds,
    RegionLineSegments3D,
    RegionPointCloud2,
)
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode, encode, schema
from mcap.reader import make_reader
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import message_header, to_nanoseconds
from dimos.protocol.cdr_mcap import CdrMcapWriter


@pytest.mark.parametrize("region_id", [-(2**31), (-3 << 16) | 5, 2**31-1])
def test_region_wire_retains_signed_id_and_empty_replacement(region_id):
    header = Header(stamp=Time(sec=12, nanosec=34), frame_id="map")
    for xyz in [np.array([[1., 2., 3.]]), np.empty((0, 3))]:
        value = RegionPointCloud2(region_id=region_id, cloud=pointcloud_from_xyz(xyz, header=header))
        decoded = decode(encode(value), RegionPointCloud2)
        assert decoded.region_id == region_id
        assert type(decoded.cloud.header) is Header
        assert message_header(decoded) == header
        np.testing.assert_array_equal(pointcloud_xyz(decoded.cloud), xyz)


def test_region_mcap_contains_full_schema_and_nested_source_clock(tmp_path):
    header = Header(stamp=Time(sec=0, nanosec=34), frame_id="odom")
    value = RegionLineSegments3D(region_id=-2, lines=LineSegments3D(header=header, segments=[
        LineSegment3D(start=Point(x=1., y=2., z=3.), end=Point(x=4., y=5., z=6.), weight=0.5),
        LineSegment3D(start=Point(x=9., y=8., z=7.), end=Point(x=6., y=5., z=4.), weight=100.),
    ]))
    path = tmp_path / "regions.mcap"
    with CdrMcapWriter(path) as writer:
        writer.write("node_edges", encode(value), schema_name=value.__msgtype__, schema=schema(value.__msgtype__),
                     publish_time_ns=to_nanoseconds(message_header(value).stamp), log_time_ns=100)
    with path.open("rb") as stream:
        embedded, channel, record = next(make_reader(stream).iter_messages())
        assert embedded.encoding == "ros2msg" and channel.message_encoding == "cdr"
        for dependency in ["dimos_msgs/LineSegments3D", "dimos_msgs/LineSegment3D", "std_msgs/Header", "builtin_interfaces/Time", "geometry_msgs/Point"]:
            assert dependency in embedded.data.decode()
        assert (record.publish_time, record.log_time) == (34, 100)
        decoded = decode(record.data, RegionLineSegments3D)
        assert decoded == value
        assert [segment.weight for segment in decoded.lines.segments] == [0.5, 100.]


def test_explicit_cylinder_roundtrip_retains_limits_and_signed_id():
    value = RegionBounds(header=Header(stamp=Time(sec=1, nanosec=2), frame_id="map"), region_id=-17,
                         center=Point(x=-3., y=5., z=0.), radius=2.5, z_min=-1., z_max=4.)
    assert decode(encode(value), RegionBounds) == value
