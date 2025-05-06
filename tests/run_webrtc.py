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
from dotenv import load_dotenv
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2, Color
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.types.vector import Vector

load_dotenv()

robot = UnitreeGo2(mode="ai", ip=os.getenv("ROBOT_IP"))
robot.color(Color.RED)
robot.move_vel(Vector(0.5, 0.5, 0.5))

show3d_stream(robot.lidar_stream())
# show3d_stream(robot.map_stream())
