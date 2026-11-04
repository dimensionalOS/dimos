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

"""Live GSC PGO on a Go2-mounted Mid-360 through Point-LIO, with the pose graph in Rerun (no GO2Connection: its `lidar` would collide with Point-LIO's)."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import GscPGO
from dimos.visualization.vis_module import vis_module

unitree_go2_mid360_pgo = autoconnect(
    mid360_for_pointlio(),
    PointLio.blueprint().remappings([(PointLio, "lidar", "cloud")]),
    GscPGO.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="unitree_go2")
