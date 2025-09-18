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

# from dimos.msgs.detection2d import Detection2DArray
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d import Detection3DModule
from dimos.protocol.pubsub import lcm
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)


def detection_unitree():
    dimos = start(6)

    connection = deploy_connection(dimos)
    connection.start()
    # connection.record("unitree_go2_office_walk2")
    # mapper = deploy_navigation(dimos, connection)

    module3D = dimos.deploy(Detection3DModule, camera_info=ConnectionModule._camera_info())

    module3D.image.connect(connection.video)
    module3D.pointcloud.connect(connection.lidar)

    module3D.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    module3D.detections.transport = LCMTransport("/detections", Detection2DArray)

    module3D.detected_pointcloud_0.transport = LCMTransport("/detected/pointcloud/0", PointCloud2)
    module3D.detected_pointcloud_1.transport = LCMTransport("/detected/pointcloud/1", PointCloud2)
    module3D.detected_pointcloud_2.transport = LCMTransport("/detected/pointcloud/2", PointCloud2)

    module3D.detected_image_0.transport = LCMTransport("/detected/image/0", Image)
    module3D.detected_image_1.transport = LCMTransport("/detected/image/1", Image)
    module3D.detected_image_2.transport = LCMTransport("/detected/image/2", Image)
    module3D.start()
    # detection.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        connection.stop()
        # mapper.stop()
        # detection.stop()
        logger.info("Shutting down...")


def main():
    lcm.autoconf()
    detection_unitree()


if __name__ == "__main__":
    main()
