#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

import sys
import time
from pathlib import Path

from dotenv import load_dotenv

from dimos.agents2.cli.human import HumanInput

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from threading import Thread

import reactivex as rx
import reactivex.operators as ops

from dimos.agents2 import Agent, Output, Reducer, Stream, skill
from dimos.agents2.cli.human import HumanInput
from dimos.agents2.spec import Model, Provider
from dimos.core import LCMTransport, Module, start
from dimos.hardware.webcam import ColorCameraModule, Webcam
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.test_coordinator import SkillContainerTest
from dimos.stream.audio2 import SpeechModule
from dimos.web.robot_web_interface import RobotWebInterface


def main():
    dimos = start(4)
    # Create agent
    agent = Agent(
        system_prompt="You are a helpful assistant for controlling a Unitree Go2 robot. ",
        model=Model.GPT_4O,  # Could add CLAUDE models to enum
        provider=Provider.OPENAI,  # Would need ANTHROPIC provider
    )

    webcam = dimos.deploy(ColorCameraModule, hardware=lambda: Webcam(camera_index=0))
    webcam.image.transport = LCMTransport("/image", Image)

    webcam.start()

    human_input = dimos.deploy(HumanInput)

    speech = dimos.deploy(SpeechModule)

    time.sleep(1)

    agent.register_skills(human_input)
    agent.register_skills(webcam)
    agent.register_skills(speech)

    agent.run_implicit_skill("video_stream")
    agent.run_implicit_skill("human")

    agent.start()
    agent.loop_thread()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
