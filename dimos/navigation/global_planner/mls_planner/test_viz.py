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

from functools import partial
import struct

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import (
    LineSegment3D,
    LineSegments3D,
    RegionLineSegments3D,
    RegionPointCloud2,
)
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.navigation.global_planner.mls_planner import viz
from dimos.navigation.global_planner.mls_planner.viz import (
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
        data=np.frombuffer(struct.pack("<8f", 1, 2, 3, 0.1, 4, 5, 6, 0.8), dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_bigendian=False,
        is_dense=False,
    )
    decoded = cdr_decode(cdr_encode(cloud), PointCloud2)
    before = cdr_encode(decoded)
    rendered = render_surface_map(decoded, wall_clearance_m=0.2)
    assert rendered.positions.as_arrow_array().to_pylist() == [[4.0, 5.0, 6.0]]
    assert cdr_encode(decoded) == before


def test_nodes_lift_and_surface_without_intensity() -> None:
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 2.0, 3.0]]), header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    )
    before = cdr_encode(cloud)
    np.testing.assert_allclose(
        render_nodes(cloud).positions.as_arrow_array().to_pylist(), [[1, 2, 3.05]]
    )
    assert render_surface_map(cloud).positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert cdr_encode(cloud) == before


def test_weighted_edges_and_empty_messages() -> None:
    message = LineSegments3D(
        segments=[
            LineSegment3D(start=Point(x=0.0, y=0.0, z=0.0), end=Point(x=1, y=0.0, z=0.0), weight=1),
            LineSegment3D(start=Point(x=1, y=0.0, z=0.0), end=Point(x=2, y=0.0, z=0.0), weight=100),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    decoded = cdr_decode(cdr_encode(message), LineSegments3D)
    rendered = render_node_edges(decoded)
    np.testing.assert_allclose(
        rendered.strips.as_arrow_array().to_pylist(),
        [[[0, 0, 0.05], [1, 0, 0.05]], [[1, 0, 0.05], [2, 0, 0.05]]],
    )
    assert rendered.colors.as_arrow_array().to_pylist() == [0x00FF3CDC, 0xFF003CDC]
    assert decoded.segments[0].start.z == 0
    assert (
        render_node_edges(
            LineSegments3D(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""), segments=[])
        )
        .strips.as_arrow_array()
        .to_pylist()
        == []
    )
    empty = pointcloud_from_xyz(
        np.empty((0, 3)), header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    )
    assert render_nodes(empty).positions.as_arrow_array().to_pylist() == []


def _rgb(packed: int) -> tuple[int, int, int]:
    return (packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF


def test_surface_drops_cells_below_the_wall_clearance() -> None:
    pts = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=np.float32)
    clearance = np.array([0.05, 0.2, 2.0], dtype=np.float32)
    arch = viz.surface_points(pts, clearance, 0.1, wall_clearance_m=0.1, clearance_clamp_m=1.0)
    assert arch.positions.as_arrow_array().to_pylist() == [[1, 0, 0], [2, 0, 0]]


def test_graph_edges_take_the_binding_layout() -> None:
    edges = np.array([[0, 0, 0, 1, 0, 0, 1.0], [1, 0, 0, 1, 1, 0, 10.0]], dtype=np.float32)
    arch = viz.graph_edges(edges)
    strips = arch.strips.as_arrow_array().to_pylist()
    lift = viz.GRAPH_Z_LIFT
    np.testing.assert_allclose(
        strips, [[[0, 0, lift], [1, 0, lift]], [[1, 0, lift], [1, 1, lift]]], atol=1e-6
    )
    cheap, costly = (_rgb(c) for c in arch.colors.as_arrow_array().to_pylist())
    assert costly[0] > cheap[0] and costly[1] < cheap[1]
    assert (
        viz.graph_edges(np.zeros((0, 7), dtype=np.float32)).strips.as_arrow_array().to_pylist()
        == []
    )


def test_graph_nodes_are_lifted_off_the_surface() -> None:
    pts = np.array([[1.0, 2.0, 0.0], [3.0, 4.0, 1.0]], dtype=np.float32)
    arch = viz.graph_nodes(pts)
    lift = viz.GRAPH_Z_LIFT
    np.testing.assert_allclose(
        arch.positions.as_arrow_array().to_pylist(), [[1, 2, lift], [3, 4, 1 + lift]], atol=1e-6
    )
    assert pts[:, 2].tolist() == [0.0, 1.0], "the caller's points are left alone"

    empty = viz.graph_nodes(np.zeros((0, 3), dtype=np.float32))
    assert len(empty.positions.as_arrow_array()) == 0


def test_region_renders_are_static_and_an_empty_cell_still_lands() -> None:
    cell = RegionPointCloud2(
        region_id=1 << 16,
        cloud=pointcloud_from_xyz(
            np.array([[1.0, 1.0, 0.0]], dtype=np.float32),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id="map"),
        ),
    )
    (path, arch, static) = viz.render_surface_region(cell, 0.1, 0.1, 1.0)[0]
    assert path == "world/surface_map/1_0" and static
    assert len(arch.positions.as_arrow_array()) == 1

    emptied = RegionPointCloud2(
        region_id=1 << 16,
        cloud=pointcloud_from_xyz(
            np.zeros((0, 3), dtype=np.float32),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id="map"),
        ),
    )
    (path, arch, static) = viz.render_surface_region(emptied, 0.1, 0.1, 1.0)[0]
    assert path == "world/surface_map/1_0" and static
    assert len(arch.positions.as_arrow_array()) == 0

    edges = RegionLineSegments3D(
        region_id=(2 << 16) | 3,
        lines=LineSegments3D(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id="map"), segments=[]
        ),
    )
    (path, arch, static) = viz.render_edge_region(edges)[0]
    assert path == "world/node_edges/2_3" and static
    assert arch.strips.as_arrow_array().to_pylist() == []


def test_overrides_follow_the_planner_publish_rate() -> None:
    off = viz.planner_visual_override(0.0, 0.08, 0.1)
    on = viz.planner_visual_override(2.0, 0.08, 0.1)
    assert off == {"world/surface_map": None, "world/nodes": None, "world/node_edges": None}
    surface = on["world/surface_map"]
    assert isinstance(surface, partial) and surface.func is viz.render_surface_region
    assert surface.keywords == {
        "voxel_size": 0.08,
        "wall_clearance_m": 0.1,
        "clearance_clamp_m": 1.0,
    }
    assert on["world/nodes"] is viz.render_nodes
    assert on["world/node_edges"] is viz.render_edge_region
