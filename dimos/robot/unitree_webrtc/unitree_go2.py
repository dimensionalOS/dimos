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

from typing import Union, Optional, List
import time
import numpy as np
import os
from dimos.robot.robot import Robot
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.connection import WebRTCRobot
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.utils.reactive import getter_streaming
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from go2_webrtc_driver.constants import VUI_COLOR
from go2_webrtc_driver.webrtc_driver import WebRTCConnectionMethod
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.local_planner.local_planner import navigate_path_local
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.types.robot_capabilities import RobotCapability
from dimos.types.vector import Vector


class Color(VUI_COLOR): ...


class UnitreeGo2(Robot):
    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
        skill_library: SkillLibrary = None,
        robot_capabilities: List[RobotCapability] = None,
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = True,
    ):
        """Initialize Unitree Go2 robot with WebRTC control interface.

        Args:
            ip: IP address of the robot
            mode: Robot mode (ai, etc.)
            output_dir: Directory for output files
            skill_library: Skill library instance
            robot_capabilities: List of robot capabilities
            spatial_memory_collection: Collection name for spatial memory
            new_memory: Whether to create new spatial memory
        """
        # Create WebRTC connection interface
        webrtc_connection = WebRTCRobot(
            ip=ip,
            mode=mode,
        )

        # Store the WebRTC connection for direct access to WebRTC-specific features
        self.webrtc_connection = webrtc_connection

        # Initialize WebRTC-specific features
        self.odom = getter_streaming(self.webrtc_connection.odom_stream())
        self.map = Map(voxel_size=0.2)
        self.map_stream = self.map.consume(self.webrtc_connection.lidar_stream())

        # Initialize base robot with connection interface
        super().__init__(
            connection_interface=webrtc_connection,
            output_dir=output_dir,
            skill_library=skill_library,
            capabilities=robot_capabilities
            or [
                RobotCapability.LOCOMOTION,
                RobotCapability.VISION,
                RobotCapability.AUDIO,
            ],
            spatial_memory_collection=spatial_memory_collection,
            new_memory=new_memory,
        )

        # Initialize skills with robot reference
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
        else:
            self.skill_library = SkillLibrary()

        # Camera configuration
        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        self.camera_height = 0.44  # meters

        # Initialize visual servoing using connection interface
        video_stream = self.get_video_stream()
        if video_stream is not None:
            self.person_tracker = PersonTrackingStream(
                camera_intrinsics=self.camera_intrinsics,
                camera_pitch=self.camera_pitch,
                camera_height=self.camera_height,
            )
            self.object_tracker = ObjectTrackingStream(
                camera_intrinsics=self.camera_intrinsics,
                camera_pitch=self.camera_pitch,
                camera_height=self.camera_height,
            )
            person_tracking_stream = self.person_tracker.create_stream(video_stream)
            object_tracking_stream = self.object_tracker.create_stream(video_stream)

            self.person_tracking_stream = person_tracking_stream
            self.object_tracking_stream = object_tracking_stream
        else:
            self.person_tracker = None
            self.object_tracker = None
            self.person_tracking_stream = None
            self.object_tracking_stream = None

        # Initialize the local planner using WebRTC-specific methods
        self.local_planner = VFHPurePursuitPlanner(
            get_costmap=lambda: self.map.local_costmap,
            get_robot_pose=lambda: self.odom(),
            move=self.move,  # Use the robot's move method directly
            robot_width=0.36,  # Unitree Go2 width in meters
            robot_length=0.6,  # Unitree Go2 length in meters
            max_linear_vel=0.7,
            max_angular_vel=0.8,
            lookahead_distance=1.5,
            visualization_size=500,  # 500x500 pixel visualization
        )

        self.global_planner = AstarPlanner(
            set_local_nav=lambda path, stop_event=None, goal_theta=None: navigate_path_local(
                self, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
            ),
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

    def odom_stream(self):
        """Get the odometry stream from the robot.

        Returns:
            Observable stream of robot odometry data containing position and orientation.
        """
        return self.webrtc_connection.odom_stream()

    def standup(self):
        """Make the robot stand up.

        Uses AI mode standup if robot is in AI mode, otherwise uses normal standup.
        """
        return self.webrtc_connection.standup()

    def liedown(self):
        """Make the robot lie down.

        Commands the robot to lie down on the ground.
        """
        return self.webrtc_connection.liedown()

    @property
    def costmap(self):
        """Access to the costmap for navigation."""
        return self.map.costmap
