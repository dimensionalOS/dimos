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

"""Generated planner messages retain geometry through the Rerun adapters."""

import struct

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.navigation.nav_3d.mls_planner.viz import (
    render_node_edges,
    render_nodes,
    render_surface_map,
)


def test_surface_clearance_filter_after_cdr() -> None:
    cloud = PointCloud2(
        height=1,
        width=2,
        point_step=16,
        row_step=32,
        fields=[
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z", "intensity"))
        ],
        data=struct.pack("<8f", 1, 2, 3, 0.1, 4, 5, 6, 0.8),
    )
    decoded = PointCloud2.decode(cloud.encode())
    before = decoded.encode()
    rendered = render_surface_map(decoded, wall_clearance_m=0.2)
    assert rendered.positions.as_arrow_array().to_pylist() == [[4.0, 5.0, 6.0]]
    assert decoded.encode() == before


def test_nodes_lift_and_surface_without_intensity() -> None:
    cloud = pointcloud_from_xyz(np.array([[1.0, 2.0, 3.0]]), header=Header())
    before = cloud.encode()
    np.testing.assert_allclose(
        render_nodes(cloud).positions.as_arrow_array().to_pylist(), [[1, 2, 3.05]]
    )
    assert render_surface_map(cloud).positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert cloud.encode() == before


def test_weighted_edges_and_empty_messages() -> None:
    message = LineSegments3D(
        segments=[
            LineSegment3D(start=Point(), end=Point(x=1), weight=1),
            LineSegment3D(start=Point(x=1), end=Point(x=2), weight=100),
        ]
    )
    decoded = LineSegments3D.decode(message.encode())
    rendered = render_node_edges(decoded)
    np.testing.assert_allclose(
        rendered.strips.as_arrow_array().to_pylist(),
        [[[0, 0, 0.05], [1, 0, 0.05]], [[1, 0, 0.05], [2, 0, 0.05]]],
    )
    assert rendered.colors.as_arrow_array().to_pylist() == [0x00FF3CDC, 0xFF003CDC]
    assert decoded.segments[0].start.z == 0
    assert render_node_edges(LineSegments3D()).strips.as_arrow_array().to_pylist() == []
    empty = pointcloud_from_xyz(np.empty((0, 3)), header=Header())
    assert render_nodes(empty).positions.as_arrow_array().to_pylist() == []
