# Copyright 2025 Dimensional Inc.
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

import logging
import time

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core import LCMTransport, start
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.perception.detection2d import Detection2DArrayFix
from dimos.perception.detection2d.module import Detection3DModule
from dimos.protocol.pubsub import lcm
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)


def detection_unitree():
    dimos = start(6)

    connection = deploy_connection(dimos, loop=False, speed=0.2)
    # connection.record("unitree_go2_lidar_corrected")
    # mapper = deploy_navigation(dimos, connection)

    detection = dimos.deploy(Detection3DModule)
    detection.image.connect(connection.video)
    detection.camera_info.connect(connection.camera_info)
    detection.pointcloud.connect(connection.lidar)

    detection.detections.transport = LCMTransport("/detections", Detection2DArrayFix)
    detection.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    detection.filtered_pointcloud.transport = LCMTransport("/filtered_pointcloud", PointCloud2)

    # detection.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        connection.stop()
        mapper.stop()
        detection.stop()
        logger.info("Shutting down...")


def main():
    lcm.autoconf()
    detection_unitree()


if __name__ == "__main__":
    main()
