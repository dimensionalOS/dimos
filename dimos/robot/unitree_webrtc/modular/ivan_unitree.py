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

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations

from dimos.core import LCMTransport, start
from dimos.perception.detection2d import Detect2DModule, Detection2DArrayFix
from dimos.protocol.pubsub import lcm
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)


def detection_unitree():
    dimos = start(6)

    connection = deploy_connection(dimos, seek=12, duration=2, loop=True)
    # navigation = deploy_navigation(dimos, connection)

    detection = dimos.deploy(Detect2DModule)
    detection.image.connect(connection.video)
    detection.detections.transport = LCMTransport("/detections", Detection2DArrayFix)
    detection.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    detection.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        connection.stop()
        # navigation.stop()
        detection.stop()
        logger.info("Shutting down...")


def main():
    lcm.autoconf()
    detection_unitree()


if __name__ == "__main__":
    main()
