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


"""Rendering for what the planner searched over: its surface, nodes and weighted edges."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING

from dimos_generated.dimos_msgs.msg import (
    LineSegment3D,
    LineSegments3D,
    RegionLineSegments3D,
    RegionPointCloud2,
)
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import PointCloud2
import numpy as np

from dimos.msgs.pointcloud import pointcloud_view, pointcloud_xyz
from dimos.msgs.time import header_now
from dimos.visualization.rerun.bridge import RerunEntry, keyed_by_region, region_entity

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

    from dimos.visualization.rerun.bridge import RerunMulti, VisualOverride

# rerun is imported inside the renderers: it is heavy and only viewers load it.

SURFACE_MAP_ENTITY = "world/surface_map"
NODES_ENTITY = "world/nodes"
NODE_EDGES_ENTITY = "world/node_edges"

GRAPH_Z_LIFT = 0.05
NODE_RADIUS = 0.05

TIGHT_COLOR = (4.0, 8.0, 48.0)
OPEN_COLOR = (150.0, 200.0, 255.0)
NODE_COLOR = (255, 200, 0)


def clearance_colors(clearance: NDArray[np.float32], clamp_m: float) -> NDArray[np.uint8]:
    """Blue ramp from tight to open, saturating at clamp_m of clearance."""
    norm = np.clip(np.nan_to_num(clearance / clamp_m, nan=1.0, posinf=1.0), 0.0, 1.0)
    tight, open_ = np.array(TIGHT_COLOR), np.array(OPEN_COLOR)
    return np.asarray(tight + norm[:, None] * (open_ - tight), dtype=np.uint8)


def surface_points(
    pts: NDArray[np.float32],
    clearance: NDArray[np.float32],
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float,
) -> Archetype:
    """Floor cells colored by wall clearance, dropping the ones the robot cannot fit on."""
    import rerun as rr

    passable = clearance >= wall_clearance_m
    pts, clearance = pts[passable], clearance[passable]
    return rr.Points3D(
        positions=pts,
        colors=clearance_colors(clearance, clearance_clamp_m),
        radii=voxel_size * 0.5,
    )


@keyed_by_region
def render_surface_region(
    msg: RegionPointCloud2,
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float,
    z_band: tuple[float, float] | None = None,
) -> RerunMulti:
    """One cell of the surface on its own static entity, empty when the cell emptied.

    Clearance rides the intensity channel, flat blue without it. Points outside z_band
    (world frame) are dropped, to show one storey of a multi-storey map.
    """
    pts = pointcloud_xyz(msg.cloud).astype(np.float32)
    records = pointcloud_view(msg.cloud)
    clearance = records["intensity"].ravel() if "intensity" in (records.dtype.names or ()) else None
    if z_band is not None:
        keep = (pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])
        pts = pts[keep]
        if clearance is not None:
            clearance = clearance[keep]
    if clearance is None or len(clearance) != len(pts):
        import rerun as rr

        cell: Archetype = rr.Points3D(pts, radii=voxel_size * 0.5, colors=[40, 75, 130])
    else:
        cell = surface_points(pts, clearance, voxel_size, wall_clearance_m, clearance_clamp_m)
    return [RerunEntry(region_entity(SURFACE_MAP_ENTITY, msg.region_id), cell, static=True)]


def graph_nodes(pts: NDArray[np.float32]) -> Archetype:
    import rerun as rr

    if len(pts) == 0:
        return rr.Points3D([])
    lifted = pts.copy()
    lifted[:, 2] += GRAPH_Z_LIFT
    return rr.Points3D(positions=lifted, colors=[NODE_COLOR], radii=NODE_RADIUS)


def render_nodes(msg: PointCloud2) -> Archetype:
    return graph_nodes(pointcloud_xyz(msg).astype(np.float32))


def graph_edges(edges: NDArray[np.float32]) -> Archetype:
    """Edges as ``[x0, y0, z0, x1, y1, z1, cost]`` rows, colored green to red by cost."""
    segments = LineSegments3D(
        header=header_now(),
        segments=[
            LineSegment3D(
                start=Point(x=float(row[0]), y=float(row[1]), z=float(row[2])),
                end=Point(x=float(row[3]), y=float(row[4]), z=float(row[5])),
                weight=float(row[6]),
            )
            for row in edges
        ],
    )
    return render_node_edges(segments)


def render_node_edges(msg: LineSegments3D) -> Archetype:
    import rerun as rr

    if not msg.segments:
        return rr.LineStrips3D([])
    strips = np.array(
        [
            [
                [segment.start.x, segment.start.y, segment.start.z],
                [segment.end.x, segment.end.y, segment.end.z],
            ]
            for segment in msg.segments
        ],
        dtype=np.float32,
    )
    strips[:, :, 2] += GRAPH_Z_LIFT
    weights = np.array([segment.weight for segment in msg.segments], dtype=np.float64)
    log_weights = np.log10(np.maximum(weights, 1e-6))
    low, high = float(log_weights.min()), float(log_weights.max())
    norm = (log_weights - low) / (high - low) if high > low else np.zeros_like(log_weights)
    red = (255 * norm).astype(np.uint8)
    green = (255 * (1 - norm)).astype(np.uint8)
    colors = np.column_stack([red, green, np.full_like(red, 60), np.full_like(red, 220)])
    return rr.LineStrips3D(strips, colors=colors, radii=0.01)


@keyed_by_region
def render_edge_region(msg: RegionLineSegments3D) -> RerunMulti:
    return [
        RerunEntry(
            region_entity(NODE_EDGES_ENTITY, msg.region_id),
            render_node_edges(msg.lines),
            static=True,
        )
    ]


def render_surface_map(
    msg: PointCloud2,
    voxel_size: float = 0.1,
    wall_clearance_m: float = 0.0,
    clearance_clamp_m: float = 1.0,
) -> Archetype:
    entry = render_surface_region(
        RegionPointCloud2(region_id=0, cloud=msg), voxel_size, wall_clearance_m, clearance_clamp_m
    )[0]
    assert isinstance(entry, RerunEntry)
    return entry.archetype


def planner_visual_override(
    viz_publish_hz: float,
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float = 1.0,
) -> dict[str, VisualOverride]:
    """Bridge overrides for the planner's debug entities, keyed off its own publish rate.

    Pass the same values given to ``MLSPlannerNative.blueprint(...)``.
    """
    on = viz_publish_hz > 0.0
    surface = partial(
        render_surface_region,
        voxel_size=voxel_size,
        wall_clearance_m=wall_clearance_m,
        clearance_clamp_m=clearance_clamp_m,
    )
    return {
        SURFACE_MAP_ENTITY: surface if on else None,
        NODES_ENTITY: render_nodes if on else None,
        NODE_EDGES_ENTITY: render_edge_region if on else None,
    }
