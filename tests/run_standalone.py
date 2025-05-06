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

load_dotenv()

robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"))
robot.lidar_stream.subscribe(print)
