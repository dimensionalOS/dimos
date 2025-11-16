import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
import os
import time
# Initialize robot
robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                  ros_control=UnitreeROSControl(),
                  skills=MyUnitreeSkills())

# Move the robot forward
robot.move_vel(x=0.5, y=0, yaw=0, duration=5)

while True:
    time.sleep(1)