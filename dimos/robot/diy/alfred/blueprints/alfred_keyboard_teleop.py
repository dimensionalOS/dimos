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

"""Alfred teleop with the Mid-360 (Point-LIO) and the mast D455, no navigation.

Anything publishing tele_cmd_vel (web_ctrl's keyboard, dimos topic send) drives
the base through MovementManager's teleop/nav mux.

    dimos run alfred-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.blueprints.alfred_mls_nav import D455_MOUNT, D455_SERIAL
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel

# librealsense leaks usbfs fds (~8/s) in this worker; keep socket-accepting modules
# out of its fd table so EMFILE cannot take down the teleop controls.
RealSenseCamera.dedicated_worker = True

MID360_IP = "192.168.1.189"
"""Alfred's Mid-360, on the Jetson's wired 192.168.1.100/24 link."""

alfred_keyboard_teleop = (
    autoconnect(
        RealSenseCamera.blueprint(
            fps=30,
            # Nothing in this stack consumes depth, and raw depth is ~24 MB/s of
            # multicast that every worker's kernel socket has to swallow.
            enable_depth=False,
            enable_imu=True,
            camera_name="d455",
            serial_number=D455_SERIAL,
            base_transform=D455_MOUNT,
        ),
        # No raw Mid360 module: the lidar streams to a single host endpoint, and the
        # Point-LIO native owns it, so a second Livox SDK connection gets no data.
        PointLio.blueprint(lidar_ip=MID360_IP).remappings(
            [
                (PointLio, "lidar", "pointlio_lidar"),
                (PointLio, "odometry", "pointlio_odometry"),
            ]
        ),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
    )
    .transports(
        {
            # Raw colour is ~37 MB/s (~27k UDP fragments/s) that lagged the whole
            # Jetson; pay a JPEG encode to buy back the bus and the CPU.
            ("color_image", Image): JpegLcmTransport("/color_image", Image),
        }
    )
    .global_config(n_workers=6, robot_model="alfred")
)
