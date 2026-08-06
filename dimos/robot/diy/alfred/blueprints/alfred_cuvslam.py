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

"""Alfred localising from a D455 alone, with cuVSLAM instead of the lidar.

    dimos run alfred-cuvslam

``alfred_nav`` gets its pose from FastLIO2 and a Mid-360. This runs the same robot with
no lidar at all: the D455's infrared stereo pair drives cuVSLAM, which publishes odometry
and ``map -> odom -> base_link``.

Two things carry over from benchmarking six handheld D455 recordings, because both change
the answer rather than the tuning:

``enable_imu`` is **off**. On every recording measured, feeding the D455's IMU made
cuVSLAM worse -- mildly at walking pace and by 4x at jogging pace -- and no gravity,
excitation or time-offset correction recovered it. The module's own default is on, so this
is a deliberate override, and it should be revisited on a rig whose IMU has been validated
end to end rather than assumed.

``async_sba`` is **off**. cuVSLAM's asynchronous bundle adjustment thread races: the
resulting ``std::out_of_range`` is thrown from that thread, so no caller-side handler sees
it, and the process aborts. Reproduced in NVIDIA's own ``cuvslam_api_launcher``, which
already defaults the flag off. Turning it off also makes cuVSLAM deterministic; with it on,
ATE varied 0.406-3.418 m across identical runs.

Loop closure (``enable_slam``) is left on: it is what makes the difference between drifting
odometry and a pose that survives a revisit, and it costs about 2.5x wall clock while still
running roughly 25x faster than real time.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

CAMERA_NAME = "d455"

alfred_cuvslam = (
    autoconnect(
        RealSenseCamera.blueprint(
            camera_name=CAMERA_NAME,
            width=848,
            height=480,
            fps=30,
            # cuVSLAM tracks on the IR pair. It is delivered already rectified, which is
            # what lets the module run a pinhole model with an identity inter-camera
            # rotation, and it is unaffected by the colour sensor's auto-exposure.
            enable_infrared=True,
            # The projector's dot pattern moves with the camera, so feature trackers latch
            # onto it and bias motion toward zero.
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
            # The mount belongs to the URDF; left at its default the camera also publishes
            # base_link -> camera_link and camera_link ends up with two parents.
            base_transform=None,
        ),
        CuvslamOdometry.blueprint(
            enable_imu=False,
            async_sba=False,
            enable_slam=True,
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
            (RealSenseCamera, "infrared_left", "image_left"),
            (RealSenseCamera, "infrared_right", "image_right"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=6)
)
