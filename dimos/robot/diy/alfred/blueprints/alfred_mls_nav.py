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
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.diy.alfred.blueprints.vis_nav import _vis_nav
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel

D455_MOUNT = Transform(
    translation=Vector3(-0.2518, -0.2736, 0.4292),
    rotation=Quaternion(0.079967, -0.023978, -0.996361, 0.017193),
)
"""base_link -> camera_link, mirroring the calibration recorded in alfred.urdf."""

alfred_mls_nav = (
    autoconnect(
        RealSenseCamera.blueprint(
            fps=30,
            enable_infrared=True,
            emitter_enabled=False,
            enable_imu=True,
            base_transform=D455_MOUNT,
        ),
        AlfredHighLevel.blueprint(),
        _vis_nav,
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
            # Keep the colour info off the stream cuVSLAM reads its rig from.
            (RealSenseCamera, "camera_info", "color_camera_info"),
            (AlfredHighLevel, "wheel_odometry", "source_odometry"),
        ]
    )
    .transports(
        {
            # Raw colour is 19.6 MB/s of the Jetson's LCM traffic and only ever gets
            # looked at, so pay a JPEG encode to buy back the bus and the CPU.
            ("color_image", Image): JpegLcmTransport("/color_image", Image),
        }
    )
    .global_config(n_workers=10, robot_model="alfred")
)
