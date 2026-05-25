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

Wires the Evaluator and StraightLinePlanner together and bridges all
streams to rerun. Run with::

    python -m dimos.navigation.nav_stack.evaluator.main
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.sparse.csgraph import connected_components

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.navigation.nav_stack.evaluator.evaluator import Evaluator
from dimos.navigation.nav_stack.modules.mls_planner.mls_planner import (
    NODE_STEP_THRESHOLD_M,
    MLSPlanner,
    _build_surface_adjacency,
    _build_surface_lookup,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule

_POSE_MARKER_RADIUS = 0.4
_SURFACE_VOXEL = 0.1
_SURFACE_COMPONENT_PALETTE = np.array(
    [
        [220, 50, 50],
        [50, 180, 50],
        [50, 100, 220],
        [240, 180, 50],
        [180, 50, 220],
        [50, 200, 200],
        [240, 130, 50],
        [120, 200, 100],
        [200, 60, 130],
        [80, 80, 200],
    ],
    dtype=np.uint8,
)


def _render_start_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[0, 255, 0]],  # green
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_goal_pose(msg: Any) -> Any:
    import rerun as rr

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z]],
        colors=[[255, 0, 0]],  # red
        radii=[_POSE_MARKER_RADIUS],
    )


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.03, colors=[128, 128, 128])  # gray


def _render_surface_map(msg: Any) -> Any:
    import rerun as rr

    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    indices = np.floor(pts / _SURFACE_VOXEL).astype(np.int64)
    ix, iy, iz = indices[:, 0], indices[:, 1], indices[:, 2]
    surface_lookup = _build_surface_lookup(ix, iy, iz)
    step_cells = max(0, int(NODE_STEP_THRESHOLD_M / _SURFACE_VOXEL))
    adj, cell_to_idx, _ = _build_surface_adjacency(surface_lookup, _SURFACE_VOXEL, step_cells)
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


# raise the path and way points out of the surface
_GRAPH_Z_LIFT = 0.1


def _render_nodes(msg: Any) -> Any:
    import rerun as rr

    pts, _ = msg.as_numpy()
    if pts is None or len(pts) == 0:
        return rr.Points3D([])
    pts = pts.copy()
    pts[:, 2] += _GRAPH_Z_LIFT
    return rr.Points3D(positions=pts, colors=[[75, 156, 211]], radii=[0.15])  # Carolina Blue


def _render_node_edges(msg: Any) -> Any:
    return msg.to_rerun(z_offset=_GRAPH_Z_LIFT, radii=0.04)


def create_evaluator_blueprint() -> Blueprint:
    return autoconnect(
        Evaluator.blueprint(),
        MLSPlanner.blueprint(),
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


if __name__ == "__main__":
    ModuleCoordinator.build(create_evaluator_blueprint()).loop()
