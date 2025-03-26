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
robot.move_vel(0.5, 0, 0, duration=5)

# Wait for 5 seconds
time.sleep(5)



try:
    input("Press ESC to exit...")
except KeyboardInterrupt:
    print("\nExiting...")