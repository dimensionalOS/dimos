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

"""unitree_go2_pgo: run the jnav PGO live on the Go2's onboard lidar (no Mid-360
mount) and visualize the optimized pose graph in Rerun.

GO2Connection streams the onboard lidar already accumulated in the odom/world
frame plus the robot pose as a PoseStamped. The PGO instead wants a raw
sensor-frame scan plus a nav_msgs Odometry, so SensorFrameLidarBridge sits in
between: it subtracts the pose from every point (world -> base_link) and re-emits
the scan as `lidar` alongside an `odometry`. The PGO consumes both and emits a
loop-closed `pose_graph` (Graph3D), which the Rerun bridge renders as nodes
(keyframes) + edges (odom backbone in green, loop closures in yellow).

The onboard `lidar` Out is remapped to `world_lidar` so it feeds the bridge
rather than colliding with the bridge's own base_link `lidar` Out into the PGO.
For the Mid-360 + Point-LIO rig instead, see `unitree_go2_mid360_pgo`.

Run on the dog:
    dimos run unitree-go2-pgo
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import PGO
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.sensor_frame_lidar_bridge import SensorFrameLidarBridge
from dimos.visualization.rerun.bridge import RerunMulti
from dimos.visualization.vis_module import vis_module

# Rerun entity path for the pose graph. The bridge maps the `pose_graph` stream to
# `<entity_prefix>/pose_graph` = `world/pose_graph`; matching that here lets the
# override draw nodes + edges instead of the default nodes-only Points3D.
_POSE_GRAPH_PATH = "world/pose_graph"


def _render_pose_graph(graph: Graph3D) -> RerunMulti:
    """Nodes (keyframes) + edges (odom backbone / loop closures) for the graph."""
    return graph.to_rerun_multi(base_path=_POSE_GRAPH_PATH)


unitree_go2_pgo = autoconnect(
    GO2Connection.blueprint().remappings([(GO2Connection, "lidar", "world_lidar")]),
    SensorFrameLidarBridge.blueprint(),
    PGO.blueprint(),
    vis_module(
        "rerun",
        rerun_config={"visual_override": {_POSE_GRAPH_PATH: _render_pose_graph}},
    ),
).global_config(n_workers=4, robot_model="unitree_go2")
