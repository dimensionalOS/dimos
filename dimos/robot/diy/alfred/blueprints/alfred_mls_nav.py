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

"""Alfred running MLS planning off the D455 alone, with no Mid-360.

    dimos run alfred-mls-nav

``alfred_nav`` needs the lidar twice over: FastLIO2 for odometry and registered scans
for the costmap. This drops it entirely. cuVSLAM tracks the D455's infrared stereo pair
for odometry, and the same camera's depth feeds a ray-traced voxel map that MLS plans
over.

The mapper is ``RayTracingVoxelMap`` rather than ``VoxelGridMapper`` on purpose. The
latter carves whole (X, Y) columns on every insert, which is last-write-wins: measured
against a point-lio reference on drive_2026-08-18_23-05-04.db it kept 26% as many voxels
as ray tracing and ate the walls MLS needs to plan around (IoU 0.076 vs 0.107, recall
0.200 vs 0.292).
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

VOXEL_SIZE_METERS = 0.05
DEPTH_MAX_RANGE_METERS = 6.0
"""Beyond this the D455's stereo error grows past a voxel, so returns stop being
evidence. Its quadratic error model puts ~5 cm at 6 m for the 95 mm baseline."""

ALFRED_BODY_HEIGHT_METERS = 0.5

alfred_mls_nav = (
    autoconnect(
        RealSenseCamera.blueprint(
            width=848,
            height=480,
            fps=30,
            # cuVSLAM tracks the IR pair; they arrive rectified and are not perturbed by
            # the colour sensor's auto-exposure.
            enable_infrared=True,
            # The projector's dot pattern moves with the camera, so feature trackers latch
            # onto it and bias motion toward zero.
            emitter_enabled=False,
            enable_depth=True,
            # The camera only assembles a pointcloud when colour is also streaming.
            enable_color=True,
            enable_pointcloud=True,
            # Feeding the D455's own IMU made cuVSLAM worse on every handheld recording
            # benchmarked, 4x at jogging pace.
            enable_imu=False,
        ),
        CuvslamOdometry.blueprint(
            enable_imu=False,
            # Alfred's computer has no GPU; the fork-built libcuvslam carries the CPU path.
            use_gpu=False,
            base_frame="base_link",
            odom_frame="odom",
            map_frame="map",
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=DEPTH_MAX_RANGE_METERS,
        ),
        MLSPlannerNative.blueprint(
            # Nothing closes loops here, so map -> odom stays identity and odom is the
            # only consistent frame the voxel map and the planner share.
            world_frame="odom",
            voxel_size=VOXEL_SIZE_METERS,
            robot_height=ALFRED_BODY_HEIGHT_METERS,
            wall_clearance_m=0.2,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=0.16,
            step_penalty_weight=1.0,
        ).remappings(
            [
                (MLSPlannerNative, "path", "planner_path"),
                (MLSPlannerNative, "start_pose", "odom"),
            ]
        ),
        # On Go2 the base pose comes off the robot connection. Alfred has no such module,
        # so GoalRelay's odometry-to-pose conversion is what feeds every consumer of odom.
        GoalRelay.blueprint().remappings([(GoalRelay, "start_pose", "odom")]),
        DanLocalPlanner.blueprint(resample_spacing_m=0.1),
        DanHolonomicTC.blueprint(run_profile="walk"),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
        vis_module(global_config.viewer),
    )
    .remappings(
        [
            # Both imagers onto the one stream; the tracker tells them apart by frame_id.
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
            # Colour is on only to unlock the pointcloud, so keep its info off the stream
            # cuVSLAM reads its rig from.
            (RealSenseCamera, "camera_info", "color_camera_info"),
            # The depth pointcloud stands in for the lidar the mapper normally consumes.
            (RealSenseCamera, "pointcloud", "lidar"),
        ]
    )
    .global_config(n_workers=10, robot_model="alfred", obstacle_avoidance=False)
)
