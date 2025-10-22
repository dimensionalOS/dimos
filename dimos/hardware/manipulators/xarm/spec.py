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

from typing import Protocol

from dimos.core import In, Out
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import JointState  #Missing in our msgs

# Import a IK solver here


class ArmDriverSpec(Protocol):
    joint_cmd: In[]                           # Desired joint positions we need a vector/list of floats here
    velocity_cmd: In[]                        # Desired joint velocities we need a vector/list of floats here 
    joint_state: Out[JointState]              # Contains current joint positions and velocities both
    robot_state: Out[CustomRobotStateMsg]     # Custom message containing full robot state (errors, modes, etc.)

    def set_joint_angle(self, tpos_cmd: VectorOfJointAngles) -> None: ...

    def set_joint_velocity(self, vel_cmd: VectorOfJointVelocities ) -> None: ...

    def get_joint_state(self) -> JointState: ...

    def get_robot_state(self) -> CustomRobotStateMsg: ...