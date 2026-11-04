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

"""Go2 GSC PGO feeding the ray tracer and MLS planner, which warp their map and graph in place through each loop closure."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.reform import ReformConfig
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import GscPGO, PGOConfig
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config, ray_tracing_config
from dimos.visualization.vis_module import vis_module

# Both follow GscPGO's corrected frame, so new scans land where the last warp moved the map.
map_frame = PGOConfig.model_fields["map_frame"].default

unitree_go2_mls_reform = autoconnect(
    GO2Connection.blueprint(),
    GscPGO.blueprint(odom_frame=ConnectionConfig.model_fields["odom_frame_id"].default).remappings(
        [(GscPGO, "cloud", "lidar")]
    ),
    RayTracingVoxelMap.blueprint(
        **ray_tracing_config.model_dump(exclude_unset=True),
        world_frame=map_frame,
        reform=ReformConfig(enabled=True),
    ),
    MLSPlannerNative.blueprint(
        **mls_planner_config.model_dump(exclude_unset=True),
        world_frame=map_frame,
        reform=ReformConfig(enabled=True),
        require_full_route=True,
        viz_publish_hz=2.0,
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    vis_module("rerun"),
).global_config(n_workers=4, robot_model="unitree_go2")
