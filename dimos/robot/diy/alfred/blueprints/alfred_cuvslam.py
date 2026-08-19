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

"""Alfred localising from its D455 alone, with cuVSLAM instead of the lidar.

    dimos run alfred-cuvslam

``alfred_nav`` gets its pose from FastLIO2 and a Mid-360. This runs the same robot
with no lidar: the D455's infrared stereo pair drives cuVSLAM, which publishes
odometry and ``odom -> base_link``. Loop closure is no longer cuVSLAM's job; the
correction layer moved downstream, so ``map -> odom`` stays identity here.

``enable_imu`` is off: on every handheld D455 recording benchmarked, feeding the
D455's IMU made cuVSLAM worse -- mildly at walking pace, 4x at jogging pace.
``use_gpu`` is off because Alfred's computer has no GPU; the fork-built libcuvslam
carries the CPU path.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

alfred_cuvslam = (
    autoconnect(
        RealSenseCamera.blueprint(
            width=848,
            height=480,
            fps=30,
            # cuVSLAM tracks on the IR pair: delivered already rectified, which is what
            # lets the module run a pinhole model, and unaffected by the colour
            # sensor's auto-exposure.
            enable_infrared=True,
            # The projector's dot pattern moves with the camera, so feature trackers
            # latch onto it and bias motion toward zero.
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
        ),
        CuvslamOdometry.blueprint(
            enable_imu=False,
            use_gpu=False,
            base_frame="base_link",
            odom_frame="odom",
            map_frame="map",
        ),
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
        ]
    )
    .global_config(n_workers=6)
)
