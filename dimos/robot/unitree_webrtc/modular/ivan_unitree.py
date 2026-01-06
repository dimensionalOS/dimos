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

from dimos.agents.spec import Model, Provider
from dimos.core import LCMTransport, start
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.reid import ReidModule
from dimos.protocol.pubsub import lcm  # type: ignore[attr-defined]
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.unitree_webrtc.modular import deploy_connection  # type: ignore[attr-defined]
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


def detection_unitree() -> None:
    dimos = start(8)
    connection = deploy_connection(dimos)

    def goto(pose) -> bool:  # type: ignore[no-untyped-def]
        print("NAVIGATION REQUESTED:", pose)
        return True

    detector = dimos.deploy(  # type: ignore[attr-defined]
        Detection2DModule,
        camera_info=ConnectionModule._camera_info(),
    )

    detector.image.connect(connection.video)

    detector.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    detector.detections.transport = LCMTransport("/detections", Detection2DArray)

    detector.detected_image_0.transport = LCMTransport("/detected/image/0", Image)
    detector.detected_image_1.transport = LCMTransport("/detected/image/1", Image)
    detector.detected_image_2.transport = LCMTransport("/detected/image/2", Image)

    reid = dimos.deploy(ReidModule)  # type: ignore[attr-defined]

    reid.image.connect(connection.video)
    reid.detections.connect(detector.detections)
    reid.annotations.transport = LCMTransport("/reid/annotations", ImageAnnotations)

    detector.start()
    connection.start()
    reid.start()

    from dimos.agents import Agent  # type: ignore[attr-defined]
    from dimos.agents.cli.human import HumanInput

    agent = Agent(
        system_prompt="You are a helpful assistant for controlling a Unitree Go2 robot.",
        model=Model.GPT_4O,  # Could add CLAUDE models to enum
        provider=Provider.OPENAI,  # type: ignore[attr-defined]  # Would need ANTHROPIC provider
    )

    human_input = dimos.deploy(HumanInput)  # type: ignore[attr-defined]
    agent.register_skills(human_input)
    agent.register_skills(detector)

    bridge = FoxgloveBridge(
        shm_channels=[
            "/image#sensor_msgs.Image",
            "/lidar#sensor_msgs.PointCloud2",
        ]
    )
    time.sleep(1)
    bridge.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        connection.stop()
        logger.info("Shutting down...")


if __name__ == "__main__":
    lcm.autoconf()
    detection_unitree()
