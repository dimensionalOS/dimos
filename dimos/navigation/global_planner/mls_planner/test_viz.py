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

from functools import partial

import numpy as np

from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.global_planner.mls_planner import viz
from dimos.visualization.rerun.bridge import region_entity


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


def test_region_cells_unpack_from_the_seq_the_planner_packs() -> None:
    assert region_entity("world/surface_map", (-3 << 16) | (5 & 0xFFFF)) == "world/surface_map/-3_5"
    assert region_entity("world/node_edges", (7 << 16) | (-2 & 0xFFFF)) == "world/node_edges/7_-2"


def test_region_renders_are_static_and_an_empty_cell_still_lands() -> None:
    cell = PointCloud2.from_numpy(np.array([[1.0, 1.0, 0.0]], dtype=np.float32))
    cell.seq = 1 << 16
    (path, arch, static) = viz.render_surface_region(cell, 0.1, 0.1, 1.0)[0]
    assert path == "world/surface_map/1_0" and static
    assert len(arch.positions.as_arrow_array()) == 1

    emptied = PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32))
    emptied.seq = 1 << 16
    (path, arch, static) = viz.render_surface_region(emptied, 0.1, 0.1, 1.0)[0]
    assert path == "world/surface_map/1_0" and static
    assert len(arch.positions.as_arrow_array()) == 0

    edges = LineSegments3D(segments=np.zeros((0, 2, 3)), seq=(2 << 16) | 3)
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
