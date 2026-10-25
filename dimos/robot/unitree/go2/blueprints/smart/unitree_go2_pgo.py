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
sensor-frame scan plus a nav_msgs Odometry, so NormalizeGo2Lidar sits in
between: it takes the onboard `lidar`, subtracts the pose from every point
(world -> base_link) and re-emits the scan as `cloud` alongside an `odometry`.
The PGO consumes both and emits a loop-closed `pose_graph` (one packed record per
keyframe, see gsc_pgo/records.py), which the Rerun bridge renders as the keyframe
positions; `closure_edges` likewise shows each closure candidate's source keyframe.

For the Mid-360 + Point-LIO rig instead, see `unitree_go2_mid360_pgo`.

Run on the dog:
    dimos run unitree-go2-pgo
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import GscPGO
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.normalize_go2_lidar import NormalizeGo2Lidar
from dimos.visualization.vis_module import vis_module

unitree_go2_pgo = autoconnect(
    GO2Connection.blueprint(),
    NormalizeGo2Lidar.blueprint(),
    GscPGO.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=4, robot_model="unitree_go2")
