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

import time
from dotenv import load_dotenv
from dimos.agents.claude_agent import ClaudeAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.skills.observe_stream import ObserveStream
from dimos.skills.kill_skill import KillSkill
from dimos.skills.navigation import Navigate, BuildSemanticMap, GetPose, NavigateToGoal
from dimos.skills.visual_navigation_skills import NavigateToObject, FollowHuman
import reactivex as rx
import reactivex.operators as ops
from dimos.stream.audio.pipelines import tts, stt
from dimos.web.websocket_vis.server import WebsocketVis
import threading
from dimos.types.vector import Vector
from dimos.skills.speak import Speak

# Load API key from environment
load_dotenv()

robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                    ros_control=UnitreeROSControl(),
                    skills=MyUnitreeSkills(),
                    mock_connection=False)

# Create a subject for agent responses
agent_response_subject = rx.subject.Subject()
agent_response_stream = agent_response_subject.pipe(ops.share())
local_planner_viz_stream = robot.local_planner_viz_stream.pipe(ops.share())

streams = {"unitree_video": robot.get_ros_video_stream(),
           "local_planner_viz": local_planner_viz_stream}
text_streams = {
    "agent_responses": agent_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams, **streams)

stt_node = stt()

# Create a ClaudeAgent instance
agent = ClaudeAgent(
    dev_name="test_agent",
    input_query_stream=stt_node.emit_text(),
    # input_query_stream=web_interface.query_stream,
    skills=robot.get_skills(),
    system_query="""You are an agent controlling a virtual robot. When given a query, respond by using the appropriate tool calls if needed to execute commands on the robot.

IMPORTANT INSTRUCTIONS:
1. Each tool call must include the exact function name and appropriate parameters
2. If a function needs parameters like 'distance' or 'angle', be sure to include them
3. If you're unsure which tool to use, choose the most appropriate one based on the user's query
4. Parse the user's instructions carefully to determine correct parameter values

Example: If the user asks to move forward 1 meter, call the Move function with distance=1""",
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=2000
)

tts_node = tts()
# tts_node.consume_text(agent.get_response_observable())

robot_skills = robot.get_skills()
robot_skills.add(ObserveStream)
robot_skills.add(KillSkill)
robot_skills.add(Navigate)
robot_skills.add(BuildSemanticMap)
robot_skills.add(NavigateToObject)
robot_skills.add(FollowHuman)
robot_skills.add(GetPose)
robot_skills.add(Speak)
robot_skills.add(NavigateToGoal)
robot_skills.create_instance("ObserveStream", robot=robot, agent=agent)
robot_skills.create_instance("KillSkill", robot=robot, skill_library=robot_skills)
robot_skills.create_instance("Navigate", robot=robot)
robot_skills.create_instance("BuildSemanticMap", robot=robot)
robot_skills.create_instance("NavigateToObject", robot=robot)
robot_skills.create_instance("FollowHuman", robot=robot)
robot_skills.create_instance("GetPose", robot=robot)
robot_skills.create_instance("NavigateToGoal", robot=robot)
robot_skills.create_instance("Speak", tts_node=tts_node)

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(
    lambda x: agent_response_subject.on_next(x)
)

print("ObserveStream and Kill skills registered and ready for use")
print("Created memory.txt file")

websocket_vis = WebsocketVis()
websocket_vis.start()
websocket_vis.connect(robot.global_planner.vis_stream())


def msg_handler(msgtype, data):
    if msgtype == "click":
        target = Vector(data["position"])
        try:
            robot.global_planner.set_goal(target)
        except Exception as e:
            print(f"Error setting goal: {e}")
            return
def threaded_msg_handler(msgtype, data):
    thread = threading.Thread(target=msg_handler, args=(msgtype, data))
    thread.daemon = True
    thread.start()

websocket_vis.msg_handler = threaded_msg_handler

web_interface.run()
