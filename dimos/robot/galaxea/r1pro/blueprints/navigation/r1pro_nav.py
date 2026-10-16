# Copyright 2025-2026 Dimensional Inc.
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

"""R1 Pro 3D navigation on Point-LIO (lidar + lidar-anchored head depth into one ray-traced map) with arm planning: ``dimos run r1pro-nav``."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.experimental.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.experimental.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_lidar_odometry,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_WIDTH_M,
    R1PRO_MAX_STEP_HEIGHT_M,
    R1PRO_OVERHEAD_CLEARANCE_M,
    R1PRO_ROTATION_DIAMETER_M,
    R1PRO_URDF_MANAGER_CONFIG,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.head_depth import r1pro_head_depth
from dimos.robot.galaxea.r1pro.lio import R1ProPointLio
from dimos.robot.galaxea.r1pro.rerun import r1pro_nav_rerun_config
from dimos.robot.urdf_manager import UrdfManager
from dimos.visualization.vis_module import vis_module

VOXEL_SIZE_M = 0.07
# A hard limit, so the chassis can turn in place anywhere on a route.
WALL_CLEARANCE_M = R1PRO_ROTATION_DIAMETER_M / 2
PLANNER_VIZ_HZ = 0.0
# How far apart a sensor sample and the tf it is placed with may be, for every module here that matches them: the lidar
# mount tf comes at 5 Hz, Point-LIO's at 10 with jitter, and the Livox cloud arrives ~0.11 s behind its stamp (0.1 s
# dropped clouds and head-depth frames).
TF_TOLERANCE_S = 0.5


r1pro_nav = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=r1pro_nav_rerun_config(VOXEL_SIZE_M, WALL_CLEARANCE_M, PLANNER_VIZ_HZ),
    ),
    # Point-LIO owns odom -> base_link, so the connection's wheel odometry stays off tf.
    r1pro_control(publish_odom_tf=False, enable_wrist_color=False, enable_imu=False),
    r1pro_lidar_odometry().remappings([(R1ProPointLio, "lidar", "lidar_with_robot")]),
    # Every joint on tf (the head cameras ride the torso), and the robot's own body cut out of the Mid-360's scans.
    UrdfManager.blueprint(
        **R1PRO_URDF_MANAGER_CONFIG,
        tf_tolerance_s=TF_TOLERANCE_S,
        tf_forward_tolerance_s=TF_TOLERANCE_S,
    ).remappings(
        [
            (UrdfManager, "pointcloud", "lidar_with_robot"),
            (UrdfManager, "filtered_pointcloud", "lidar"),
        ]
    ),
    r1pro_head_depth(tf_tolerance_s=TF_TOLERANCE_S).remappings(
        [(Depth2DepthCloud, "depth_cloud", "lidar")]
    ),
    RayTracingVoxelMap.blueprint(
        voxel_size=VOXEL_SIZE_M,
        max_range=10.0,
        # Live, heavier head depth fell ~10 s behind; skipping stale clouds means it never works through a queue.
        max_cloud_age_s=1.0,
        emit_every=1,
        global_emit_every=50,
        support_min=4,
        worker_threads=3,
        tf_match_tolerance_s=TF_TOLERANCE_S,
    ),
    MLSPlannerNative.blueprint(
        voxel_size=VOXEL_SIZE_M,
        robot_height=R1PRO_OVERHEAD_CLEARANCE_M,
        start_z_offset_m=0.0,
        wall_clearance_m=WALL_CLEARANCE_M,
        wall_buffer_m=R1PRO_CHASSIS_WIDTH_M,
        wall_buffer_weight=100.0,
        step_threshold_m=R1PRO_MAX_STEP_HEIGHT_M,
        step_penalty_weight=4.0,
        viz_publish_hz=PLANNER_VIZ_HZ,
        worker_threads=2,
    ).remappings(
        [
            (MLSPlannerNative, "global_map", "global_map_unused"),
            (MLSPlannerNative, "path", "planner_path"),
        ]
    ),
    DanLocalPlanner.blueprint(
        # No commit window: MLS's obstacle truncations must reach the follower at once.
        lock_replan=0.0,
        # Keep MLS's 3D waypoints; the 2D resampler zeroes every Z.
        resample_spacing_m=0.0,
    ).remappings([(DanLocalPlanner, "odom", "chassis_odom")]),
    DanHolonomicTC.blueprint(control_frequency=10.0).remappings(
        [(DanHolonomicTC, "odom", "chassis_odom")]
    ),
    MovementManager.blueprint(),
    # Arm planning (viser, or a controller's arm mode) on r1pro_control's whole-body trajectory task. Its tip frames
    # would give links UrdfManager already publishes a second parent on tf, so they go to a topic of their own.
    ManipulationModule.blueprint(
        model=make_r1pro_model_config(),
        planning_timeout=10.0,
        visualization=ViserVisualizationConfig(host=global_config.listen_host),
    ).remappings([(ManipulationModule, "tf", "manipulation_tf")]),
).global_config(
    # One per module, so no two share a Python process (and its GIL).
    n_workers=16,
    robot_width=R1PRO_CHASSIS_WIDTH_M,
    robot_rotation_diameter=R1PRO_ROTATION_DIAMETER_M,
)
