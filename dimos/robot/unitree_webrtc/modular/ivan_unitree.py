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
from lcm_msgs.foxglove_msgs import SceneUpdate

from dimos.agents2.spec import Model, Provider
from dimos.core import LCMTransport, start

# from dimos.msgs.detection2d import Detection2DArray
from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule
from dimos.protocol.pubsub import lcm
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)


def detection_unitree():
    dimos = start(6)
    connection = deploy_connection(dimos)
    # mapper = deploy_navigation(dimos, connection)
    # mapper.start()

    def goto(pose):
        print("NAVIGATION REQUESTED:", pose)
        return True

    module3D = dimos.deploy(
        ObjectDBModule,
        goto=goto,
        camera_info=ConnectionModule._camera_info(),
    )

    module3D.image.connect(connection.video)
    # module3D.pointcloud.connect(mapper.global_map)
    module3D.pointcloud.connect(connection.lidar)

    module3D.annotations.transport = LCMTransport("/annotations", ImageAnnotations)
    module3D.detections.transport = LCMTransport("/detections", Detection2DArray)

    module3D.detected_pointcloud_0.transport = LCMTransport("/detected/pointcloud/0", PointCloud2)
    module3D.detected_pointcloud_1.transport = LCMTransport("/detected/pointcloud/1", PointCloud2)
    module3D.detected_pointcloud_2.transport = LCMTransport("/detected/pointcloud/2", PointCloud2)

    module3D.detected_image_0.transport = LCMTransport("/detected/image/0", Image)
    module3D.detected_image_1.transport = LCMTransport("/detected/image/1", Image)
    module3D.detected_image_2.transport = LCMTransport("/detected/image/2", Image)

    module3D.scene_update.transport = LCMTransport("/scene_update", SceneUpdate)

    module3D.start()
    connection.start()

    from dimos.agents2 import Agent, Output, Reducer, Stream, skill
    from dimos.agents2.cli.human import HumanInput

    agent = Agent(
        system_prompt="You are a helpful assistant for controlling a Unitree Go2 robot. ",
        model=Model.GPT_4O,  # Could add CLAUDE models to enum
        provider=Provider.OPENAI,  # Would need ANTHROPIC provider
    )

    human_input = dimos.deploy(HumanInput)
    agent.register_skills(human_input)
    # agent.register_skills(connection)
    agent.register_skills(module3D)

    # agent.run_implicit_skill("video_stream_tool")
    agent.run_implicit_skill("human")

    agent.start()
    agent.loop_thread()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        connection.stop()
        logger.info("Shutting down...")


def main():
    lcm.autoconf()
    detection_unitree()


if __name__ == "__main__":
    main()
