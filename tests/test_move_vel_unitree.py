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
import time

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills

# Initialize robot
robot = UnitreeGo2(
    ip=os.getenv("ROBOT_IP"), ros_control=UnitreeROSControl(), skills=MyUnitreeSkills()
)

# Move the robot forward
robot.move_vel(x=0.5, y=0, yaw=0, duration=5)

while True:
    time.sleep(1)
