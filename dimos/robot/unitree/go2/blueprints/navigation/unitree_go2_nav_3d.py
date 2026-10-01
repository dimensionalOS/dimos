#!/usr/bin/env python3
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

"""3d navigation on Go2 with ray tracing and MLS planning"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.viz import nav_static, nav_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.basic.module import BasicPathFollower
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.go2_mid360_static_transforms import Go2Mid360StaticTf
from dimos.robot.unitree.go2.nav_3d_config import (
    mls_planner_config,
    ray_tracing_config,
    relocalization,
    voxel_size,
    wall_clearance_m,
)
from dimos.visualization.vis_module import vis_module

# What the planner searched over (surface, nodes, weighted edges), by changed cell.
planner_viz_hz = 2.0


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        # Rate-limited at the source by global_emit_every, roughly every 5s.
        "world/global_map": 0,
        "world/local_map": 0.5,
    },
    # Ring buffer replayed to a connecting viewer. Small so connect catches up fast.
    "memory_limit": "64MB",
    # The robot box hangs off base_link on its own entity: a static transform
    # under world/tf would override the live one.
    "static": nav_static(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT, wall_clearance_m),
    "visual_override": {
        **rerun_config["visual_override"],
        # The raw premap is millions of points. The seeded voxels arrive on seed_map.
        "world/loaded_map": None,
        "world/camera_info": None,
        "world/color_image": None,
        "world/lidar": None,
        **nav_visual_override(planner_viz_hz, voxel_size, wall_clearance_m),
    },
}

unitree_go2_nav_3d = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    # "mcf" for stair traversal
    GO2Connection.blueprint(
        lidar=False,
        camera=False,
        motion_mode="mcf",
        odom_frame_id="go2_odom",
        publish_tf=False,
    ).remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    PointLio.blueprint(),
    Go2Mid360StaticTf.blueprint(),
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    MLSPlannerNative.blueprint(
        **mls_planner_config.model_copy(update={"viz_publish_hz": planner_viz_hz}).model_dump(
            exclude_unset=True
        )
    ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
    BasicPathFollower.blueprint(heading_gain=1.0, lookahead_time_s=2.5, min_lookahead_m=1.2),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)

# LCM can lose the one-shot loaded_map publish, so this stack republishes it.
unitree_go2_nav_3d_relocalization = autoconnect(
    unitree_go2_nav_3d,
    relocalization(republish_loaded_map=30.0),
).global_config(n_workers=11)
