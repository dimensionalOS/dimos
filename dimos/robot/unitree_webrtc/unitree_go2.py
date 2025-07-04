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
import threading
import time
from typing import List, Optional, Union

import numpy as np
from go2_webrtc_driver.constants import VUI_COLOR
from go2_webrtc_driver.webrtc_driver import WebRTCConnectionMethod

from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.robot.local_planner.local_planner import navigate_path_local
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.unitree_webrtc.connection import WebRTCRobot
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.types.vector import Vector
from dimos.utils.reactive import getter_streaming


class Color(VUI_COLOR): ...


class UnitreeGo2(WebRTCRobot):
    def __init__(
        self,
        ip: str,
        mode: str = "ai",
    ):
        super().__init__(ip, mode)

        print("standing up")
        self.standup()

        self.odom = getter_streaming(self.odom_stream())
        self.map = Map(voxel_size=0.2)
        # self.map_stream = self.map.consume(self.lidar_stream)
        # self.lidar_message = getter_streaming(self.lidar_stream)

        self.global_planner = AstarPlanner(
            set_local_nav=lambda path, stop_event=None, goal_theta=None: navigate_path_local(
                self, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
            ),
            get_costmap=lambda: self.map.costmap,
            get_robot_pos=lambda: self.odom().pos,
        )

        # Initialize the local planner using WebRTC-specific methods
        self.local_planner = VFHPurePursuitPlanner(
            get_costmap=lambda: self.lidar_message().costmap(),
            get_robot_pose=lambda: self.odom(),
            move=self.move,  # Use the robot's move method directly
            robot_width=0.36,  # Unitree Go2 width in meters
            robot_length=0.6,  # Unitree Go2 length in meters
            max_linear_vel=0.7,
            max_angular_vel=0.65,
            lookahead_distance=1.5,
            visualization_size=500,  # 500x500 pixel visualization
            global_planner_plan=self.global_planner.plan,
        )

        # Initialize frontier exploration
        self.frontier_explorer = WavefrontFrontierExplorer(
            set_goal=self.global_planner.set_goal,
            get_costmap=lambda: self.map.costmap,
            get_robot_pos=lambda: self.odom().pos,
        )

        # Create the visualization stream at 5Hz
        self.local_planner_viz_stream = self.local_planner.create_stream(frequency_hz=5.0)

    def get_pose(self) -> dict:
        """
        Get the current pose (position and rotation) of the robot in the map frame.

        Returns:
            Dictionary containing:
                - position: Vector (x, y, z)
                - rotation: Vector (roll, pitch, yaw) in radians
        """
        position = Vector(self.odom().pos.x, self.odom().pos.y, self.odom().pos.z)
        orientation = Vector(self.odom().rot.x, self.odom().rot.y, self.odom().rot.z)
        return {"position": position, "rotation": orientation}

    def explore(self, stop_event: Optional[threading.Event] = None) -> bool:
        """
        Start autonomous frontier exploration.

        Args:
            stop_event: Optional threading.Event to signal when exploration should stop

        Returns:
            bool: True if exploration completed successfully, False if stopped or failed
        """
        return self.frontier_explorer.explore(stop_event=stop_event)

    @property
    def costmap(self):
        """Access to the costmap for navigation."""
        return self.map.costmap
