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

"""unitree_go2_mid360_pgo: run the jnav PGO live on a Go2's Livox Mid-360 +
Point-LIO and visualize the optimized pose graph in Rerun.

Point-LIO reads the Mid-360 and publishes a registered `lidar` (PointCloud2) plus
`odometry` (Odometry); the PGO consumes both and emits a loop-closed `pose_graph`
(Graph3D). The Rerun bridge renders that graph as nodes (keyframes) + edges (odom
backbone in green, loop closures in yellow) via `Graph3D.to_rerun_multi`.

This is a passive observer rig — drive the dog however you like (Go2 app / a
teleop blueprint) and watch the graph build and snap on loop closure. It needs
only the Mid-360 + Point-LIO, so it deliberately does NOT pull in GO2Connection
(whose own `lidar` Out would collide with Point-LIO's registered `lidar`). For a
mount-free rig that runs on the Go2's onboard lidar instead, see
`unitree_go2_pgo`.

Run on the dog:
    dimos run unitree-go2-mid360-pgo
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import PGO
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.visualization.rerun.bridge import RerunMulti
from dimos.visualization.vis_module import vis_module

# Rerun entity path for the pose graph. The bridge maps the `pose_graph` stream to
# `<entity_prefix>/pose_graph` = `world/pose_graph`; matching that here lets the
# override draw nodes + edges instead of the default nodes-only Points3D.
_POSE_GRAPH_PATH = "world/pose_graph"


def _render_pose_graph(graph: Graph3D) -> RerunMulti:
    """Nodes (keyframes) + edges (odom backbone / loop closures) for the graph."""
    return graph.to_rerun_multi(base_path=_POSE_GRAPH_PATH)


unitree_go2_mid360_pgo = autoconnect(
    PointLio.blueprint(),
    PGO.blueprint(),
    vis_module(
        "rerun",
        rerun_config={"visual_override": {_POSE_GRAPH_PATH: _render_pose_graph}},
    ),
).global_config(n_workers=3, robot_model="unitree_go2")
