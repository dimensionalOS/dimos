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

import tests.test_header

import os
from dotenv import load_dotenv
from dimos.agents.claude_agent import ClaudeAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
import time
import reactivex as rx
import reactivex.operators as ops
# Load API key from environment
load_dotenv()

robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                    ros_control=UnitreeROSControl(),
                    skills=MyUnitreeSkills())

# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())

text_streams = {
    "agent_responses": agent_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams)

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    input_query_stream=web_interface.query_stream,
    skills=robot.get_skills(),
    system_query="You are a one-off agent sending commands to a virtual robot given a simple input query. EXECUTE TOOL CALLS TO THE BEST OF YOUR ABILITY AND ONLY RESPOND WITH TOOL CALLS. YOU CANNOT RECEIVE FOLLOW UP QUERIES.",
    model_name="claude-3-7-sonnet-latest",
    #thinking_budget_tokens=500
)

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(
    lambda x: agent_response_subject.on_next(x)
)

web_interface.run()