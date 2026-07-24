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

"""Rendering for :class:`MLSPlannerNative`'s introspection layers.

The planner can publish what it actually searched over: the traversable surface it
extracted, the graph nodes it sampled on that surface, and the weighted edges between
them. Its ``viz_publish_hz`` config decides whether it emits them (0.0 = not at all,
which is the default — the geometry is rebuilt from scratch every tick).
:func:`planner_visual_override` reads that same number so a blueprint only has to set it
in one place; drawing and publishing cannot drift apart.
"""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

# Small lift so graph artifacts render visibly above the surface points instead of z-fighting.
_GRAPH_Z_LIFT = 0.05


def render_surface_map(
    msg: PointCloud2,
    voxel_size: float = 0.1,
    wall_clearance_m: float = 0.0,
    clearance_clamp_m: float = 1.0,
) -> Archetype:
    """Floor cells colored by wall clearance: dark navy where tight, pale blue in the open.

    Clearance rides the cloud's intensity channel; cells below ``wall_clearance_m`` are
    untraversable and dropped. Falls back to a flat color when the channel is absent.
    """
    import rerun as rr

    pts = msg.points_f32()
    clearance = msg.intensities_f32()
    if clearance is None or len(clearance) != len(pts):
        return msg.to_rerun(voxel_size=voxel_size, colors=[40, 75, 130])
    passable = clearance >= wall_clearance_m
    pts, clearance = pts[passable], clearance[passable]
    norm = np.clip(np.nan_to_num(clearance / clearance_clamp_m, nan=1.0, posinf=1.0), 0.0, 1.0)
    tight = np.array([4.0, 8.0, 48.0])
    open_ = np.array([150.0, 200.0, 255.0])
    colors = (tight + norm[:, None] * (open_ - tight)).astype(np.uint8)
    return rr.Points3D(
        positions=pts,
        colors=colors,
        radii=voxel_size * 0.5,
    )


def render_nodes(msg: PointCloud2) -> Archetype:
    import rerun as rr

    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    pts = pts.copy()
    pts[:, 2] += _GRAPH_Z_LIFT
    return rr.Points3D(positions=pts, colors=[[75, 156, 211]], radii=[0.15])


def render_node_edges(msg: LineSegments3D) -> Archetype:
    """Color each segment by its safe-adj weight on a log-scale green->red gradient."""
    import rerun as rr

    if not msg._segments:
        return rr.LineStrips3D([])
    weights = np.asarray(msg._traversability, dtype=np.float64)
    log_w = np.log10(np.maximum(weights, 1e-6))
    lo, hi = float(log_w.min()), float(log_w.max())
    norm = (log_w - lo) / (hi - lo) if hi > lo else np.zeros_like(log_w)
    r = (255 * norm).astype(np.uint8)
    g = (255 * (1.0 - norm)).astype(np.uint8)
    b = np.full_like(r, 60)
    a = np.full_like(r, 220)
    colors = np.column_stack([r, g, b, a])
    strips = [
        [
            [p1[0], p1[1], p1[2] + _GRAPH_Z_LIFT],
            [p2[0], p2[1], p2[2] + _GRAPH_Z_LIFT],
        ]
        for p1, p2 in msg._segments
    ]
    return rr.LineStrips3D(strips, colors=colors, radii=[0.01] * len(strips))


def planner_visual_override(
    viz_publish_hz: float,
    voxel_size: float = 0.1,
    wall_clearance_m: float = 0.0,
    clearance_clamp_m: float = 1.0,
) -> dict[str, Any]:
    """rerun overrides for the planner's debug entities, keyed off its own publish rate.

    Pass the same ``viz_publish_hz``, ``voxel_size`` and ``wall_clearance_m`` given to
    ``MLSPlannerNative.blueprint(...)``.
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
