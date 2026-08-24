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
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel

D455_MOUNT = Transform(
    translation=Vector3(-0.2518, -0.2736, 0.42),
    rotation=Quaternion(0.078360, 0.006348, -0.996712, 0.019616),
)
"""base_link -> camera_link, fit against drive_2026-08-18_23-05-04.db.

The camera module publishes this edge itself and never reads ``alfred.urdf``, so
correcting the URDF would not reach this blueprint.
"""

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
        vis_nav,
    )
    .remappings(
        [
            # The tracker tells the imagers apart by frame_id.
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
            # Keep the colour info off the stream cuVSLAM reads its rig from.
            (RealSenseCamera, "camera_info", "color_camera_info"),
            (AlfredHighLevel, "wheel_odometry", "source_odometry"),
        ]
    )
    .global_config(n_workers=10, robot_model="alfred")
)
