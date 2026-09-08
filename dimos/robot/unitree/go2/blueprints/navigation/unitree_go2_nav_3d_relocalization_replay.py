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

"""The relocalized nav_3d stack driven by a recording instead of a robot.

dimos run unitree-go2-nav-3d-relocalization-replay --map-file=<premap stem>
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.relocalization.blueprints import RecordingPlayer
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nav_3d import (
    mls_planner_config,
    nav_rerun_config,
    unitree_go2_nav_3d_relocalization,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import Go2Mid360StaticTf
from dimos.visualization.vis_module import vis_module

# The viewer bridge renders the seeded surface in Python, so a higher rate starves it.
planner_viz_hz = 0.2

# The recording carries the mount tf chain, so the static publisher would
# write base_link a second time.
unitree_go2_nav_3d_relocalization_replay = autoconnect(
    unitree_go2_nav_3d_relocalization.disabled_modules(
        GO2Connection, PointLio, Go2Mid360StaticTf, BasicPathFollower
    ),
    RecordingPlayer.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=nav_rerun_config(planner_viz_hz)),
    MLSPlannerNative.blueprint(
        **{**mls_planner_config.model_dump(exclude_unset=True), "viz_publish_hz": planner_viz_hz}
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
).global_config(n_workers=8)
