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
import os
import time
from typing import Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.sensor_msgs import CameraInfo

from dimos.core import LCMTransport, start
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d import Detect2DModule, Detection2DArrayFix
from dimos.protocol.pubsub import lcm
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.connectionModule import ConnectionModule, FakeRTC
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)


class UnitreeGo2:
    def __init__(
        self,
        ip: str,
        connection_type: Optional[str] = "fake",
    ):
        dimos = start(3)

        foxglove_bridge = dimos.deploy(FoxgloveBridge)
        foxglove_bridge.start()

        connection = dimos.deploy(ConnectionModule, ip, connection_type)
        connection.lidar.transport = LCMTransport("/lidar", LidarMessage)
        connection.odom.transport = LCMTransport("/odom", PoseStamped)
        connection.video.transport = LCMTransport("/image", Image)
        connection.movecmd.transport = LCMTransport("/cmd_vel", Vector3)
        connection.camera_info.transport = LCMTransport("/camera_info", CameraInfo)

        detection = dimos.deploy(Detect2DModule)
        detection.image.connect(connection.video)
        # detection.detections.transport = LCMTransport("/detections", Detection2DArrayFix)
        detection.detections.transport = LCMTransport("/detections", ImageAnnotations)

        connection.start()
        detection.start()

    def stop(): ...


def main():
    lcm.autoconf()
    robot = UnitreeGo2(
        ip=os.getenv("ROBOT_IP"), connection_type=os.getenv("CONNECTION_TYPE", "fake")
    )

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.stop()
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()
