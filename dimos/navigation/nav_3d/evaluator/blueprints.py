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

"""Blueprint for the path-planner evaluator.

Wires the Evaluator and MLSPlannerNative together and bridges all streams to rerun.
Run with::

    dimos run path-planner-eval
"""

from __future__ import annotations

import numpy as np
import rerun as rr
from rerun._baseclasses import Archetype

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.modules.click_start_goal_router.click_start_goal_router import (
    ClickStartGoalRouter,
)
from dimos.navigation.nav_3d.evaluator.evaluator import Evaluator
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_POSE_MARKER_RADIUS = 0.4
# Small lift so graph artifacts render visibly above the surface points instead of z-fighting.
_GRAPH_Z_LIFT = 0.05


def _render_start_pose(msg: PoseStamped) -> Archetype:
    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[0, 255, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_goal_pose(msg: PoseStamped) -> Archetype:
    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[255, 0, 0]],
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_global_map(msg: PointCloud2) -> Archetype:
    return msg.to_rerun(voxel_size=0.03, colors=[128, 128, 128])


def _render_surface_map(msg: PointCloud2) -> Archetype:
    return msg.to_rerun(voxel_size=0.1, colors=[40, 75, 130])


def _render_nodes(msg: PointCloud2) -> Archetype:
    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    pts = pts.copy()
    pts[:, 2] += _GRAPH_Z_LIFT
    return rr.Points3D(positions=pts, colors=[[75, 156, 211]], radii=[0.15])


def _render_node_edges(msg: LineSegments3D) -> Archetype:
    """Color each segment by its safe-adj weight on a log-scale green->red gradient."""
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


path_planner_eval = autoconnect(
    Evaluator.blueprint(),
    MLSPlannerNative.blueprint(),
    ClickStartGoalRouter.blueprint(),
    RerunWebSocketServer.blueprint(),
    RerunBridgeModule.blueprint(
        visual_override={
            "world/start_pose": _render_start_pose,
            "world/goal_pose": _render_goal_pose,
            "world/global_map": _render_global_map,
            "world/surface_map": _render_surface_map,
            "world/nodes": _render_nodes,
            "world/node_edges": _render_node_edges,
        }
    ),
)
