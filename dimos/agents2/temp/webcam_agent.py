#!/usr/bin/env python3
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

"""
Run script for Unitree Go2 robot with agents2 framework.
This is the migrated version using the new LangChain-based agent system.
"""

import time

from dimos.agents2 import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.spec import Model, Provider
from dimos.core import LCMTransport, start
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.protocol.skill.test_coordinator import SkillContainerTest


def main() -> None:
    dimos = start(4)
    # Create agent
    agent = Agent(
        system_prompt="You are a helpful assistant for controlling a Unitree Go2 robot. ",
        model=Model.GPT_4O,  # Could add CLAUDE models to enum
        provider=Provider.OPENAI,  # Would need ANTHROPIC provider
    )

    testcontainer = dimos.deploy(SkillContainerTest)
    webcam = dimos.deploy(
        CameraModule,
        transform=Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
        ),
        hardware=lambda: Webcam(
            camera_index=0,
            frequency=15,
            stereo_slice="left",
            camera_info=zed.CameraInfo.SingleWebcam,
        ),
    )

    webcam.camera_info.transport = LCMTransport("/camera_info", CameraInfo)

    webcam.image.transport = LCMTransport("/image", Image)

    webcam.start()

    human_input = dimos.deploy(HumanInput)

    time.sleep(1)

    agent.register_skills(human_input)
    agent.register_skills(webcam)
    agent.register_skills(testcontainer)

    agent.run_implicit_skill("video_stream")
    agent.run_implicit_skill("human")

    agent.start()
    agent.loop_thread()

    while True:
        time.sleep(1)

    # webcam.stop()


if __name__ == "__main__":
    main()
