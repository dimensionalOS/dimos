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

import multiprocessing
from typing import Optional, Union, Tuple
import numpy as np
from dimos.robot.robot import Robot
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, AbstractSkill, SkillLibrary
from dimos.stream.video_providers.unitree import UnitreeVideoProvider
from reactivex.disposable import CompositeDisposable
import logging
import time
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import WebRTCConnectionMethod
import os
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from reactivex.scheduler import ThreadPoolScheduler
import threading
from dimos.utils.logging_config import setup_logger
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.local_planner.local_planner import navigate_path_local
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.utils.reactive import backpressure

# Set up logging
logger = setup_logger("dimos.robot.unitree.unitree_go2", level=logging.DEBUG)

# UnitreeGo2 Print Colors (Magenta)
UNITREE_GO2_PRINT_COLOR = "\033[35m"
UNITREE_GO2_RESET_COLOR = "\033[0m"


class UnitreeGo2(Robot):
    def __init__(
        self,
        ros_control: Optional[UnitreeROSControl] = None,
        ip=None,
        connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA,
        serial_number: str = None,
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
        use_ros: bool = True,
        use_webrtc: bool = False,
        disable_video_stream: bool = False,
        mock_connection: bool = False,
        skills: Optional[Union[MyUnitreeSkills, AbstractSkill]] = None,
        spatial_memory_dir: str = None,
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = False,
    ):
        """Initialize the UnitreeGo2 robot.

        Args:
            ros_control: ROS control interface, if None a new one will be created
            ip: IP address of the robot (for LocalSTA connection)
            connection_method: WebRTC connection method (LocalSTA or LocalAP)
            serial_number: Serial number of the robot (for LocalSTA with serial)
            output_dir: Directory for output files
            use_ros: Whether to use ROSControl and ROS video provider
            use_webrtc: Whether to use WebRTC video provider ONLY
            disable_video_stream: Whether to disable the video stream
            mock_connection: Whether to mock the connection to the robot
            skills: Skills library or custom skill implementation. Default is MyUnitreeSkills() if None.
            spatial_memory_dir: Directory for storing spatial memory data. If None, uses output_dir/spatial_memory.
            spatial_memory_collection: Name of the collection in the ChromaDB database.
            new_memory: If True, creates a new spatial memory from scratch.
        """
        print(f"Initializing UnitreeGo2 with use_ros: {use_ros} and use_webrtc: {use_webrtc}")
        if not (use_ros ^ use_webrtc):  # XOR operator ensures exactly one is True
            raise ValueError("Exactly one video/control provider (ROS or WebRTC) must be enabled")

        # Initialize ros_control if it is not provided and use_ros is True
        if ros_control is None and use_ros:
            ros_control = UnitreeROSControl(
                node_name="unitree_go2", disable_video_stream=disable_video_stream, mock_connection=mock_connection
            )

        # Initialize skill library
        if skills is None:
            skills = MyUnitreeSkills(robot=self)

        super().__init__(
            ros_control=ros_control,
            output_dir=output_dir,
            skill_library=skills,
            spatial_memory_dir=spatial_memory_dir,
            spatial_memory_collection=spatial_memory_collection,
            new_memory=new_memory,
        )

        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

        # Camera stuff
        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        self.camera_height = 0.44  # meters

        # Initialize UnitreeGo2-specific attributes
        self.ip = ip
        self.disposables = CompositeDisposable()
        self.main_stream_obs = None

        # Initialize thread pool scheduler
        self.optimal_thread_count = multiprocessing.cpu_count()
        self.thread_pool_scheduler = ThreadPoolScheduler(self.optimal_thread_count // 2)

        if (connection_method == WebRTCConnectionMethod.LocalSTA) and (ip is None):
            raise ValueError("IP address is required for LocalSTA connection")

        # Choose data provider based on configuration
        if use_ros and not disable_video_stream:
            # Use ROS video provider from ROSControl
            self.video_stream = self.ros_control.video_provider
        elif use_webrtc and not disable_video_stream:
            # Use WebRTC ONLY video provider
            self.video_stream = UnitreeVideoProvider(
                dev_name="UnitreeGo2",
                connection_method=connection_method,
                serial_number=serial_number,
                ip=self.ip if connection_method == WebRTCConnectionMethod.LocalSTA else None,
            )
        else:
            self.video_stream = None

        # Initialize visual servoing if enabled
        if self.video_stream is not None:
            self.video_stream_ros = self.get_ros_video_stream(fps=8)
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
            person_tracking_stream = self.person_tracker.create_stream(self.video_stream_ros)
            object_tracking_stream = self.object_tracker.create_stream(self.video_stream_ros)

            self.person_tracking_stream = person_tracking_stream
            self.object_tracking_stream = object_tracking_stream

        # Initialize the local planner and create BEV visualization stream
        self.local_planner = VFHPurePursuitPlanner(
            get_costmap=self.ros_control.topic_latest("/local_costmap/costmap", Costmap),
            transform=self.ros_control,
            move_vel_control=self.ros_control.move_vel_control,
            robot_width=0.36,  # Unitree Go2 width in meters
            robot_length=0.6,  # Unitree Go2 length in meters
            max_linear_vel=0.5,
            lookahead_distance=1.0,
            visualization_size=500,  # 500x500 pixel visualization
        )

        self.global_planner = AstarPlanner(
            conservativism=20,  # how close to obstacles robot is allowed to path plan
            set_local_nav=lambda path, stop_event=None, goal_theta=None: navigate_path_local(self, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event),
            get_costmap=self.ros_control.topic_latest("map", Costmap),
            get_robot_pos=lambda: self.ros_control.transform_euler_pos("base_link"),
        )

        # Create the visualization stream at 5Hz
        self.local_planner_viz_stream = self.local_planner.create_stream(frequency_hz=5.0)

    def get_skills(self) -> Optional[SkillLibrary]:
        return self.skill_library

    def get_pose(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Get the current pose (position and rotation) of the robot in the map frame.
        
        Returns:
            Tuple containing:
                - position: Tuple[float, float, float] (x, y, z)
                - rotation: Tuple[float, float, float] (roll, pitch, yaw) in radians
        """
        [position, rotation] = self.ros_control.transform_euler("base_link")

        return position, rotation
