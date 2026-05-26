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

"""Blueprint + entrypoint for the path-planner evaluator.

Wires the Evaluator and MLSPlanner together and bridges all streams to rerun.
Run with::

    python -m dimos.navigation.nav_stack.evaluator.main
"""

from __future__ import annotations

from functools import partial
from typing import Any

import numpy as np
from scipy.sparse.csgraph import connected_components

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.nav_stack.evaluator.evaluator import Evaluator
from dimos.navigation.nav_stack.modules.mls_planner.mls_planner import (
    NODE_STEP_THRESHOLD_M,
    MLSPlanner,
    MLSPlannerConfig,
    build_surface_adjacency,
    build_surface_lookup,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_POSE_MARKER_RADIUS = 0.4
# Small lift so graph artifacts render visibly above the surface points instead of z-fighting.
_GRAPH_Z_LIFT = 0.05
_SURFACE_COMPONENT_PALETTE = np.array(
    [
        [245, 140, 150],
        [245, 185, 120],
        [245, 225, 125],
        [170, 220, 135],
        [125, 220, 195],
        [130, 195, 230],
        [170, 160, 230],
        [210, 160, 230],
        [230, 160, 195],
        [225, 200, 145],
    ],
    dtype=np.uint8,
)


def _render_start_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[0, 255, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_goal_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[255, 0, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.03, colors=[128, 128, 128])


def _render_surface_map(voxel_size: float, msg: Any) -> Any:
    """Render surface points colored by connected component."""
    import rerun as rr

    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    pts = pts.astype(np.float32)
    indices = np.floor(pts / voxel_size).astype(np.int64)
    ix, iy, iz = indices[:, 0], indices[:, 1], indices[:, 2]
    surface_lookup = build_surface_lookup(ix, iy, iz)
    step_cells = max(0, int(NODE_STEP_THRESHOLD_M / voxel_size))
    adj, cell_to_idx, _ = build_surface_adjacency(surface_lookup, voxel_size, step_cells)
    _, labels = connected_components(adj, directed=False)
    point_labels = np.array(
        [
            labels[cell_to_idx[cell]]
            for cell in zip(ix.tolist(), iy.tolist(), iz.tolist(), strict=True)
        ],
        dtype=np.int64,
    )
    colors = _SURFACE_COMPONENT_PALETTE[point_labels % len(_SURFACE_COMPONENT_PALETTE)]
    return rr.Points3D(positions=pts, colors=colors, radii=[0.05])


def _render_nodes(msg: Any) -> Any:
    import rerun as rr

    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    pts = pts.copy()
    pts[:, 2] += _GRAPH_Z_LIFT
    return rr.Points3D(positions=pts, colors=[[75, 156, 211]], radii=[0.15])


def _render_node_edges(msg: Any) -> Any:
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
    return rr.LineStrips3D(strips, colors=colors, radii=[0.04] * len(strips))


def create_evaluator_blueprint() -> Blueprint:
    planner_voxel = MLSPlannerConfig().voxel_size
    return autoconnect(
        Evaluator.blueprint(),
        MLSPlanner.blueprint(),
        RerunWebSocketServer.blueprint(),
        RerunBridgeModule.blueprint(
            visual_override={
                "world/start_pose": _render_start_pose,
                "world/goal_pose": _render_goal_pose,
                "world/global_map": _render_global_map,
                "world/surface_map": partial(_render_surface_map, planner_voxel),
                "world/nodes": _render_nodes,
                "world/node_edges": _render_node_edges,
            }
        ),
    )


if __name__ == "__main__":
    ModuleCoordinator.build(create_evaluator_blueprint()).loop()
