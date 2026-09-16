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

from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

    from dimos.visualization.rerun.bridge import VisualOverride

GRAPH_Z_LIFT = 0.05

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


def render_surface_map(
    msg: PointCloud2,
    voxel_size: float,
    wall_clearance_m: float,
    clearance_clamp_m: float,
) -> Archetype:
    """Surface cells with clearance on the intensity channel, flat blue without it."""
    pts = msg.points_f32()
    clearance = msg.intensities_f32()
    if clearance is None or len(clearance) != len(pts):
        return msg.to_rerun(voxel_size=voxel_size, colors=[40, 75, 130])
    return surface_points(pts, clearance, voxel_size, wall_clearance_m, clearance_clamp_m)


def graph_nodes(pts: NDArray[np.float32]) -> Archetype:
    import rerun as rr

    if len(pts) == 0:
        return rr.Points3D([])
    lifted = pts.copy()
    lifted[:, 2] += GRAPH_Z_LIFT
    return rr.Points3D(positions=lifted, colors=[NODE_COLOR], radii=0.05)


def render_nodes(msg: PointCloud2) -> Archetype:
    return graph_nodes(msg.points_f32())


def graph_edges(edges: NDArray[np.float32]) -> Archetype:
    """Edges as ``[x0, y0, z0, x1, y1, z1, cost]`` rows, colored green to red by cost."""
    segments = LineSegments3D(
        segments=edges[:, :6].reshape(-1, 2, 3) if len(edges) else None,
        weights=edges[:, 6] if len(edges) else None,
    )
    return render_node_edges(segments)


def render_node_edges(msg: LineSegments3D) -> Archetype:
    return msg.to_rerun(z_offset=GRAPH_Z_LIFT, radii=0.01)


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
        render_surface_map,
        voxel_size=voxel_size,
        wall_clearance_m=wall_clearance_m,
        clearance_clamp_m=clearance_clamp_m,
    )
    return {
        "world/surface_map": surface if on else None,
        "world/nodes": render_nodes if on else None,
        "world/node_edges": render_node_edges if on else None,
    }
