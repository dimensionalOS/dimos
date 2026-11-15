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

"""Live GSC PGO on the Go2's onboard lidar (world-frame scans, posed by GO2Connection's world -> base_link tf), with the pose graph in Rerun."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import GscPGO
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.visualization.vis_module import vis_module

unitree_go2_pgo = autoconnect(
    GO2Connection.blueprint(),
    GscPGO.blueprint(odom_frame=ConnectionConfig.model_fields["odom_frame_id"].default).remappings(
        [(GscPGO, "cloud", "lidar")]
    ),
    vis_module("rerun"),
).global_config(n_workers=4, robot_model="unitree_go2")
