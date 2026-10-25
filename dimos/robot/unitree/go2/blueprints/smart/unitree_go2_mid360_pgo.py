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
`odometry` (Odometry); its `lidar` is remapped to the PGO's `cloud` In. The PGO
consumes both and emits a loop-closed `pose_graph` (one packed record per keyframe,
see gsc_pgo/records.py). The Rerun bridge renders it as the keyframe positions;
`closure_edges` likewise shows each closure candidate's source keyframe.

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
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import GscPGO
from dimos.visualization.vis_module import vis_module

unitree_go2_mid360_pgo = autoconnect(
    PointLio.blueprint().remappings([(PointLio, "lidar", "cloud")]),
    GscPGO.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="unitree_go2")
