#!/usr/bin/env python3

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
    # skills=robot.get_skills()
)

# Subscribe to agent responses and send them to the subject
agent.get_response_observable().subscribe(
    lambda x: agent_response_subject.on_next(x)
)

web_interface.run()



# Use the stream_query method to get a response
# response = agent.stream_query("What is the capital of France?").run()

# print(f"Response from Claude Agent: {response}") 