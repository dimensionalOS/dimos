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
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.module_utils import robot_capability
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.robot_clean import Robot
from dimos.perception.spatial_perception import SpatialMemory
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.skills.skills import SkillLibrary
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill
from dimos.perception.object_tracker import ObjectTrackingStream
import time
from dimos.agents.claude_agent import ClaudeAgent
from dimos.web.robot_web_interface import RobotWebInterface
import numpy as np
import threading
from dimos.skills.navigation import Explore


@robot_capability(
    SpatialMemory,
    PersonTrackingStream,
    AstarPlanner,
    WavefrontFrontierExplorer,
    VFHPurePursuitPlanner,
    ObjectTrackingStream,
)
class UnitreeGo2(Robot):
    def __init__(self):
        conn = UnitreeWebRTCConnection(ip=os.getenv("ROBOT_IP"))
        super().__init__(conn)
        self.skill_library = MyUnitreeSkills()

        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

        # Camera configuration
        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        self.camera_height = 0.44  # meters


robot = UnitreeGo2()
robot.skill_library.add(Explore)
robot.skill_library.create_instance("Explore", robot=robot)
streams = {
    "unitree_video": robot.video_stream(),
}
web_interface = RobotWebInterface(port=5555, **streams)

agent = ClaudeAgent(
    dev_name="test_agent",
    # input_query_stream=stt_node.emit_text(),
    input_query_stream=web_interface.query_stream,
    skills=robot.skill_library,
    system_query="You are a helpful robot",
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=0,
)

web_thread = threading.Thread(target=web_interface.run)
web_thread.daemon = True
web_thread.start()

while True:
    time.sleep(0.01)
