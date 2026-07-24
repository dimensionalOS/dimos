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

import rerun as rr
from rerun._baseclasses import Archetype

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.modules.click_start_goal_router.click_start_goal_router import (
    ClickStartGoalRouter,
)
from dimos.navigation.nav_3d.evaluator.evaluator import Evaluator
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import (
    render_node_edges,
    render_nodes,
    render_surface_map,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_POSE_MARKER_RADIUS = 0.4


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
            # The evaluator exists to inspect the planner, so these are always on.
            "world/surface_map": render_surface_map,
            "world/nodes": render_nodes,
            "world/node_edges": render_node_edges,
        }
    ),
)
