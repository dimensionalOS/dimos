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

import os

from dimos.agents.agent import OpenAIAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.stream.audio.pipelines import stt, tts
from dimos.stream.audio.utils import keepalive
from dimos.utils.threadpool import get_scheduler


def main():
    stt_node = stt()
    tts_node = tts()

    robot = UnitreeGo2(
        ip=os.getenv("ROBOT_IP"),
        ros_control=UnitreeROSControl(),
        skills=MyUnitreeSkills(),
    )

    # Initialize agent with main thread pool scheduler
    agent = OpenAIAgent(
        dev_name="UnitreeExecutionAgent",
        input_query_stream=stt_node.emit_text(),
        system_query="You are a helpful robot named daneel that does my bidding",
        pool_scheduler=get_scheduler(),
        skills=robot.get_skills(),
    )

    tts_node.consume_text(agent.get_response_observable())

    # Keep the main thread alive
    keepalive()


if __name__ == "__main__":
    main()
