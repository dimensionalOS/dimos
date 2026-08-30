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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel

# librealsense leaks usbfs fds (~8/s) in this worker; keep socket-accepting modules
# out of its fd table so EMFILE cannot take down rerun/keyboard controls.
RealSenseCamera.dedicated_worker = True

D455_SERIAL = "260922302422"
"""The mast D455. The rear D435i stays plugged in, so the device is pinned by serial."""

D455_MOUNT = Transform(
    translation=Vector3(0.2146, -0.0008, 1.5435),
    rotation=Quaternion(0.0009818, 0.0723326, -0.0053295, 0.9973658),
)
"""base_link -> d455_link, mirroring the d455_joint origin in alfred.urdf. Solved from
drive_2026-08-18_23-05-04 by registering D455 depth against the point-lio lidar map.
Only z is tightly resolved; x and y are bounded to about +/-3 cm."""

D455_REMAPPINGS = [
    (RealSenseCamera, "infrared_left", "image"),
    (RealSenseCamera, "infrared_right", "image"),
    (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
    (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
    # Keep the colour info off the stream cuVSLAM reads its rig from.
    (RealSenseCamera, "camera_info", "color_camera_info"),
]


def jpeg_color() -> dict[tuple[str, type[Image]], JpegLcmTransport]:
    """Raw colour is 19.6 MB/s of the Jetson's LCM traffic and only ever gets looked at, so
    pay a JPEG encode to buy back the bus and the CPU."""
    return {("color_image", Image): JpegLcmTransport("/color_image", Image)}


def d455_stereo() -> Any:
    """The mast D455 as cuVSLAM's stereo rig. The projector is off: its dots ride with the
    robot, so they track as stationary features and eat the motion estimate."""
    return RealSenseCamera.blueprint(
        fps=30,
        enable_infrared=True,
        emitter_enabled=False,
        enable_imu=True,
        camera_name="d455",
        serial_number=D455_SERIAL,
        base_transform=D455_MOUNT,
    )


alfred_mls_nav = (
    autoconnect(
        d455_stereo(),
        AlfredHighLevel.blueprint(),
        vis_nav(),
    )
    .remappings([*D455_REMAPPINGS, (AlfredHighLevel, "wheel_odometry", "source_odometry")])
    .transports(jpeg_color())
    .global_config(n_workers=10, robot_model="alfred")
)
