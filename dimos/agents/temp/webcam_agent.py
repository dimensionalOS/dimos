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
Run script for Unitree Go2 robot with agents framework.
This is the migrated version using the new LangChain-based agent system.
"""

from threading import Thread
import time

import reactivex as rx
import reactivex.operators as ops

from dimos.agents import Agent, Output, Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.agents.cli.human import HumanInput
from dimos.agents.spec import Model, Provider
from dimos.core import LCMTransport, Module, rpc, start
from dimos.hardware.sensors.camera import zed
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.protocol.skill.test_coordinator import SkillContainerTest
from dimos.web.robot_web_interface import RobotWebInterface


class WebModule(Module):
    web_interface: RobotWebInterface = None  # type: ignore[assignment]
    human_query: rx.subject.Subject = None  # type: ignore[assignment, type-arg]
    agent_response: rx.subject.Subject = None  # type: ignore[assignment, type-arg]

    thread: Thread = None  # type: ignore[assignment]

    _human_messages_running = False

    def __init__(self) -> None:
        super().__init__()
        self.agent_response = rx.subject.Subject()
        self.human_query = rx.subject.Subject()

    @rpc
    def start(self) -> None:
        super().start()

        text_streams = {
            "agent_responses": self.agent_response,
        }

        self.web_interface = RobotWebInterface(
            port=5555,
            text_streams=text_streams,
            audio_subject=rx.subject.Subject(),
        )

        unsub = self.web_interface.query_stream.subscribe(self.human_query.on_next)
        self._disposables.add(unsub)

        self.thread = Thread(target=self.web_interface.run, daemon=True)
        self.thread.start()

    @rpc
    def stop(self) -> None:
        if self.web_interface:
            self.web_interface.stop()  # type: ignore[attr-defined]
        if self.thread:
            # TODO, you can't just wait for a server to close, you have to signal it to end.
            self.thread.join(timeout=1.0)

        super().stop()

    @skill(stream=Stream.call_agent, reducer=Reducer.all, output=Output.human)  # type: ignore[arg-type]
    def human_messages(self):  # type: ignore[no-untyped-def]
        """Provide human messages from web interface. Don't use this tool, it's running implicitly already"""
        if self._human_messages_running:
            print("human_messages already running, not starting another")
            return "already running"
        self._human_messages_running = True
        while True:
            print("Waiting for human message...")
            message = self.human_query.pipe(ops.first()).run()
            print(f"Got human message: {message}")
            yield message


def main() -> None:
    dimos = start(4)
    # Create agent
    agent = Agent(
        system_prompt="You are a helpful assistant for controlling a Unitree Go2 robot. ",
        model=Model.GPT_4O,  # Could add CLAUDE models to enum
        provider=Provider.OPENAI,  # type: ignore[attr-defined]  # Would need ANTHROPIC provider
    )

    testcontainer = dimos.deploy(SkillContainerTest)  # type: ignore[attr-defined]
    webcam = dimos.deploy(  # type: ignore[attr-defined]
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

    human_input = dimos.deploy(HumanInput)  # type: ignore[attr-defined]

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
