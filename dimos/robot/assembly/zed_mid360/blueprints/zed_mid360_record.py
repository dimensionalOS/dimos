#!/usr/bin/env python3
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

"""Record blueprint for the zed_mid360 rig (SDK-free ZED + Mid-360 via Point-LIO).

Both ZED eyes (60 fps color, no ZED SDK) and Point-LIO odom+lidar are recorded into
a memory2 db. Point-LIO publishes the moving ``world -> lidar_link`` edge onto tf
and the rig's urdf mount frames are republished continuously, so every recorded
stream resolves against ``world``. The lidar IP comes from Point-LIO's own config
(``DIMOS_POINTLIO_LIDAR_IP``)::

    export DIMOS_POINTLIO_LIDAR_IP=192.168.1.107
    dimos run zed-mid360-record
"""

from datetime import datetime

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.zed.sdkless_camera import ZedUvcCamera
from dimos.hardware.sensors.camera.zed.sdkless_imu import ZedImu
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.assembly.zed_mid360.assembly import ZedMid360StaticTf
from dimos.robot.assembly.zed_mid360.record import ZedMid360Recorder

run_start = datetime.now().astimezone()

zed_mid360_record = autoconnect(
    ZedUvcCamera.blueprint(encoder="h264_nvenc"),
    # ZED-M onboard IMU at ~800 Hz (SDK-free HID; name matches the recorder In).
    ZedImu.blueprint(),
    # world -> lidar_link is the moving odometry edge; lidar_link is the mid360
    # point-cloud origin in zed_mid360.urdf, tying odometry into the rig tree.
    PointLio.blueprint(frame_id="world", sensor_frame_id="lidar_link"),
    ZedMid360Recorder.blueprint(
        db_path=str(
            RECORDINGS_DIR
            / (
                "zed_mid360__"
                + run_start.strftime("%Y-%m-%d")
                + "_"
                + run_start.strftime("%I-%M%p").lower()
                + "-"
                + run_start.strftime("%Z")
                + ".db"
            )
        )
    ),
    # Continuously republishes the rig's urdf mount frames onto tf (no latched static tf).
    ZedMid360StaticTf.blueprint(),
).global_config(n_workers=4)
