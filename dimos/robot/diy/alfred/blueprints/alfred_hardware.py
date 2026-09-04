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

"""Alfred's physical layer: the mast D455, the mount transforms, and the base controller.

Every Alfred blueprint composes this and then adds what it does with the sensors. Private, so
it stays out of the runnable blueprint registry: sensors with nothing consuming them.

The rear D435i stays plugged in, so with both attached librealsense may hand back the wrong
one. Pick the mast camera per machine rather than in here:

    dimos run alfred-mls-nav --serial-number=<mast d455 serial>

or set `serial_number` once under the module in ~/.config/dimos/config.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf

# librealsense leaks usbfs fds (~8/s) in this worker; keep socket-accepting modules
# out of its fd table so EMFILE cannot take down rerun/keyboard controls.
RealSenseCamera.dedicated_worker = True

_alfred_hardware = autoconnect(
    RealSenseCamera.blueprint(
        fps=30,
        enable_infrared=True,
        emitter_enabled=False,
        enable_imu=True,
        # Nothing consumes colour, and raw colour is 19.6 MB/s of wire traffic. The JPEG
        # transport that used to pay for it only binds on a python module, and this one is
        # native now. Depth is unaligned without colour, so it lands in its own frame.
        enable_color=False,
        align_depth_to_color=False,
        frame_id="d455_link",
    ).remappings(
        [
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
            # Keep the colour info off the stream dimSLAM reads its rig from.
            (RealSenseCamera, "camera_info", "color_camera_info"),
        ]
    ),
    AlfredMountTf.blueprint(),
    AlfredHighLevel.blueprint().remappings([(AlfredHighLevel, "wheel_odometry", "odom_sources")]),
).global_config(robot_model="alfred")
