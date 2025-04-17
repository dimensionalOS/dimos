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
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
from dotenv import load_dotenv
from dimos.agents.claude_agent import ClaudeAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.skills.monitor_skill import MonitorSkill
from dimos.skills.kill_skill import KillSkill
import reactivex as rx
import reactivex.operators as ops
# Load API key from environment
load_dotenv()

robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                    ros_control=UnitreeROSControl(),
                    skills=MyUnitreeSkills(),
                    mock_connection=True)

# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())

streams = {"unitree_video": robot.get_ros_video_stream()}
text_streams = {
    "agent_responses": agent_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams, **streams)

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    input_query_stream=web_interface.query_stream,
    skills=robot.get_skills(),
    system_query="""You are an agent controlling a virtual robot. When given a query, respond ONLY by using the appropriate tool calls to execute commands on the robot.

IMPORTANT INSTRUCTIONS:
1. ALWAYS use the proper tool call mechanism - NEVER output JSON in your response text
2. Each tool call must include the exact function name and appropriate parameters
3. If a function needs parameters like 'distance' or 'angle', be sure to include them
4. Respond ONLY with tool calls - do not include explanations or additional text
5. If you're unsure which tool to use, choose the most appropriate one based on the user's query
6. Parse the user's instructions carefully to determine correct parameter values

Example: If the user asks to move forward 1 meter, call the Move function with distance=1""",
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=10000
)

robot_skills = robot.get_skills()
robot_skills.add_skill(MonitorSkill)
robot_skills.add_skill(KillSkill)

robot_skills.create_instance("MonitorSkill", robot=robot, claude_agent=agent)
robot_skills.create_instance("KillSkill")

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(
    lambda x: agent_response_subject.on_next(x)
)

# Create a memory.txt file to indicate the test was run
with open("memory.txt", "w") as f:
    f.write(f"Test run at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write("Monitor and Kill skills registered and ready for use\n")

print("Monitor and Kill skills registered and ready for use")
print("Created memory.txt file")

web_interface.run()
