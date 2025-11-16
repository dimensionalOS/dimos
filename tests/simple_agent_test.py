import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.agents.agent import OpenAIAgent
import os

# Initialize robot
robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                  ros_control=UnitreeROSControl(),
                  skills=MyUnitreeSkills())

# Initialize agent
agent = OpenAIAgent(
            dev_name="UnitreeExecutionAgent",
            input_video_stream=robot.get_ros_video_stream(),
            skills=robot.get_skills(),
            system_query="Wiggle when you see a person! Jump when you see a person waving!"
        )

try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")