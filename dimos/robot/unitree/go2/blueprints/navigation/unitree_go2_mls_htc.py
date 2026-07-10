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

"""3D navigation on Go2 with voxel-grid mapping, MLS planning, and holonomic
trajectory control over WebRTC.

"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05
# Height of the head-mounted lidar above the ground while standing.
# While in case of Go2 it's ~ .3m, but in this blueprint
# MLSPlanner works better on Go2 lidar when the value is 0.5
go2_lidar_height = 0.5


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        "world/global_map": 0,
    },
    "memory_limit": "8192MB",
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/planner_path": None,
        "world/path": _render_path,
        "world/surface_map": None,
        "world/nodes": None,
        "world/node_edges": None,
    },
}

unitree_go2_mls_htc = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
    GO2Connection.blueprint(motion_mode="mcf"),
    VoxelGridMapper.blueprint(
        voxel_size=voxel_size,
        frame_id="world",
        emit_every=1,
    ),
    MLSPlannerNative.blueprint(
        world_frame="world",
        voxel_size=voxel_size,
        robot_height=go2_lidar_height,
        wall_clearance_m=0.2,
        wall_buffer_m=0.75,
        wall_buffer_weight=100.0,
        step_threshold_m=0.16,
        step_penalty_weight=1.0,
        viz_publish_hz=0.0,
    ).remappings(
        [
            (MLSPlannerNative, "path", "planner_path"),
            # The planner's start pose is the robot's odom pose
            (MLSPlannerNative, "start_pose", "odom"),
        ]
    ),
    GoalRelay.blueprint(),
    # Setting resample_spacing_m to > 0.0 will smooth out jagged paths retunned my MLSP
    DanLocalPlanner.blueprint(resample_spacing_m=0.1),
    DanHolonomicTC.blueprint(run_profile="walk"),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False)
