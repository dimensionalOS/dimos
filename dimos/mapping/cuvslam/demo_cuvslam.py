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

"""cuVSLAM on a RealSense D455 and nothing else.

    dimos run demo-cuvslam --viewer rerun --rerun-host 0.0.0.0

The smallest thing that shows whether cuVSLAM is tracking: a camera, the tracker, and a
viewer. ``alfred_cuvslam`` is the same pair wired into a robot, so when that misbehaves
this narrows down whether the problem is the tracker or everything around it. It also
pulls in no robot-specific dependencies, which matters because the Alfred blueprint
cannot even be imported without ``portal`` installed.

The camera settings are the ones that survived benchmarking six handheld recordings, and
each changes the answer rather than the tuning:

``emitter_enabled`` is **off**. The projector's dot pattern is fixed to the camera, so it
moves exactly with it and feature trackers latch onto it and bias motion toward zero.

``enable_imu`` is **off**. On every recording measured, feeding the D455's IMU made
cuVSLAM worse -- mildly at walking pace, 4x at jogging pace -- and no gravity, excitation
or time-offset correction recovered it.

``async_sba`` is **off**. cuVSLAM's asynchronous bundle adjustment thread races and throws
``std::out_of_range`` from that thread, where no caller-side handler can catch it, so the
process aborts. NVIDIA's own launcher already defaults it off. Turning it off also makes
cuVSLAM deterministic; with it on, ATE varied 0.406-3.418 m across identical runs.

What to look for: ``odometry`` advancing pose after pose, and restarts staying rare.
Frames arriving in the viewer only proves the camera works -- cuVSLAM restarting its world
frame constantly still publishes odometry and still draws.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.visualization.vis_module import vis_module

CAMERA_NAME = "d455"

demo_cuvslam = (
    autoconnect(
        RealSenseCamera.blueprint(
            camera_name=CAMERA_NAME,
            width=848,
            height=480,
            fps=30,
            # cuVSLAM tracks on the IR pair. The device delivers it already rectified,
            # which is what lets the module run a pinhole model with an identity
            # inter-camera rotation.
            enable_infrared=True,
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
        ),
        CuvslamOdometry.blueprint(
            enable_imu=False,
            async_sba=False,
            enable_slam=True,
            base_frame="base_link",
            odom_frame="odom",
            map_frame="map",
        ),
        vis_module(global_config.viewer),
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image_left"),
            (RealSenseCamera, "infrared_right", "image_right"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=3)
)
