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

import numpy as np
import rerun as rr

from dimos.messages.dimos_msgs.msg import LineSegments3D, LineSegments3DRegion, PointCloudRegion
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.msgs.pointcloud import pointcloud_view, pointcloud_xyz
from dimos.visualization.rerun.bridge import RerunEntry, keyed_by_seq, region_entity

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

    from dimos.visualization.rerun.bridge import RerunMulti, VisualOverride

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
    passable = clearance >= wall_clearance_m
    pts, clearance = pts[passable], clearance[passable]
    return rr.Points3D(
        positions=pts,
        colors=clearance_colors(clearance, clearance_clamp_m),
        radii=voxel_size * 0.5,
    )


@keyed_by_seq
def render_surface_region(
    msg: PointCloudRegion,
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float,
    z_band: tuple[float, float] | None = None,
) -> RerunMulti:
    """One cell of the surface on its own static entity, empty when the cell emptied.

    Clearance rides the intensity channel, flat blue without it. Points outside z_band
    (world frame) are dropped, to show one storey of a multi-storey map.
    """
    cloud = msg.cloud
    pts = np.asarray(pointcloud_xyz(cloud), dtype=np.float32)
    records = pointcloud_view(cloud)
    clearance = (
        np.asarray(records["intensity"].ravel(), dtype=np.float32)
        if "intensity" in (records.dtype.names or ())
        else None
    )
    cell: Archetype
    if clearance is None or len(clearance) != len(pts):
        if z_band is not None:
            pts = pts[(pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])]
        cell = (
            rr.Points3D(pts, colors=[40, 75, 130], radii=voxel_size / 2)
            if len(pts)
            else rr.Points3D([])
        )
    else:
        if z_band is not None:
            keep = (pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])
            pts, clearance = pts[keep], clearance[keep]
        cell = surface_points(pts, clearance, voxel_size, wall_clearance_m, clearance_clamp_m)
    return [RerunEntry(region_entity(SURFACE_MAP_ENTITY, msg.region_id), cell, static=True)]


def graph_nodes(pts: NDArray[np.float32]) -> Archetype:
    if len(pts) == 0:
        return rr.Points3D([])
    lifted = pts.copy()
    lifted[:, 2] += GRAPH_Z_LIFT
    return rr.Points3D(positions=lifted, colors=[NODE_COLOR], radii=NODE_RADIUS)


def render_nodes(msg: PointCloud2) -> Archetype:
    return graph_nodes(np.asarray(pointcloud_xyz(msg), dtype=np.float32))


def _weighted_lines(strips: NDArray[np.float32], weights: NDArray[np.float64]) -> Archetype:
    if len(strips) == 0:
        return rr.LineStrips3D([])
    positions = strips.astype(np.float32, copy=True)
    positions[:, :, 2] += GRAPH_Z_LIFT
    log_weights = np.log10(np.maximum(weights, 1e-6))
    low, high = float(log_weights.min()), float(log_weights.max())
    norm = (log_weights - low) / (high - low) if high > low else np.zeros_like(log_weights)
    red = (255 * norm).astype(np.uint8)
    green = (255 * (1 - norm)).astype(np.uint8)
    colors = np.column_stack([red, green, np.full_like(red, 60), np.full_like(red, 220)])
    return rr.LineStrips3D(positions, colors=colors, radii=0.01)


def graph_edges(edges: NDArray[np.float32]) -> Archetype:
    """Edges as [x0,y0,z0,x1,y1,z1,cost] rows, colored green to red by cost."""
    return _weighted_lines(
        edges[:, :6].reshape(-1, 2, 3), np.asarray(edges[:, 6], dtype=np.float64)
    )


def render_node_edges(msg: LineSegments3D) -> Archetype:
    points = np.array(
        [
            [[edge.start.x, edge.start.y, edge.start.z], [edge.end.x, edge.end.y, edge.end.z]]
            for edge in msg.segments
        ],
        dtype=np.float32,
    ).reshape(-1, 2, 3)
    weights = np.array([edge.weight for edge in msg.segments], dtype=np.float64)
    return _weighted_lines(points, weights)


@keyed_by_seq
def render_edge_region(msg: LineSegments3DRegion) -> RerunMulti:
    """One keyed cell of weighted edge corridors, including empty clears."""
    return [
        RerunEntry(
            region_entity(NODE_EDGES_ENTITY, msg.region_id),
            render_node_edges(msg.lines),
            static=True,
        )
    ]


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
