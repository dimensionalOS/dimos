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
from typing import Optional, Union, List
import numpy as np
from dimos.robot.robot import Robot
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, AbstractSkill, SkillLibrary
from reactivex.disposable import CompositeDisposable
import logging
import os
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from reactivex.scheduler import ThreadPoolScheduler
from dimos.utils.logging_config import setup_logger
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.local_planner.local_planner import navigate_path_local
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.types.costmap import Costmap
from dimos.types.robot_capabilities import RobotCapability
from dimos.types.vector import Vector

# Set up logging
logger = setup_logger("dimos.robot.unitree.unitree_go2", level=logging.DEBUG)

# UnitreeGo2 Print Colors (Magenta)
UNITREE_GO2_PRINT_COLOR = "\033[35m"
UNITREE_GO2_RESET_COLOR = "\033[0m"


class UnitreeGo2(Robot):
    """Unitree Go2 robot implementation using ROS2 control interface.

    This class extends the base Robot class to provide specific functionality
    for the Unitree Go2 quadruped robot using ROS2 for communication and control.
    """

    def __init__(
        self,
        video_provider=None,
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
        skill_library: SkillLibrary = None,
        robot_capabilities: List[RobotCapability] = None,
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = False,
        disable_video_stream: bool = False,
        mock_connection: bool = False,
        enable_perception: bool = True,
    ):
        """Initialize UnitreeGo2 robot with ROS control interface.

        Args:
            video_provider: Provider for video streams
            output_dir: Directory for output files
            skill_library: Library of robot skills
            robot_capabilities: List of robot capabilities
            spatial_memory_collection: Collection name for spatial memory
            new_memory: Whether to create new memory collection
            disable_video_stream: Whether to disable video streaming
            mock_connection: Whether to use mock connection for testing
            enable_perception: Whether to enable perception streams and spatial memory
        """
        # Create ROS control interface
        ros_control = UnitreeROSControl(
            node_name="unitree_go2",
            video_provider=video_provider,
            disable_video_stream=disable_video_stream,
            mock_connection=mock_connection,
        )

        # Initialize skill library if not provided
        if skill_library is None:
            skill_library = MyUnitreeSkills()

        # Initialize base robot with connection interface
        super().__init__(
            connection_interface=ros_control,
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
            enable_perception=enable_perception,
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
        self.disposables = CompositeDisposable()
        self.main_stream_obs = None

        # Initialize thread pool scheduler
        self.optimal_thread_count = multiprocessing.cpu_count()
        self.thread_pool_scheduler = ThreadPoolScheduler(self.optimal_thread_count // 2)

        # Initialize visual servoing if enabled
        if not disable_video_stream:
            self.video_stream_ros = self.get_video_stream(fps=8)
            if enable_perception:
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
            else:
                # Video stream is available but perception tracking is disabled
                self.person_tracker = None
                self.object_tracker = None
                self.person_tracking_stream = None
                self.object_tracking_stream = None
        else:
            # Video stream is disabled
            self.video_stream_ros = None
            self.person_tracker = None
            self.object_tracker = None
            self.person_tracking_stream = None
            self.object_tracking_stream = None

        # Initialize the local planner and create BEV visualization stream
        # Note: These features require ROS-specific methods that may not be available on all connection interfaces
        if hasattr(self.connection_interface, "topic_latest") and hasattr(
            self.connection_interface, "transform_euler"
        ):
            self.local_planner = VFHPurePursuitPlanner(
                get_costmap=self.connection_interface.topic_latest(
                    "/local_costmap/costmap", Costmap
                ),
                transform=self.connection_interface,
                move_vel_control=self.connection_interface.move_vel_control,
                robot_width=0.36,  # Unitree Go2 width in meters
                robot_length=0.6,  # Unitree Go2 length in meters
                max_linear_vel=0.5,
                lookahead_distance=2.0,
                visualization_size=500,  # 500x500 pixel visualization
            )

            self.global_planner = AstarPlanner(
                conservativism=20,  # how close to obstacles robot is allowed to path plan
                set_local_nav=lambda path, stop_event=None, goal_theta=None: navigate_path_local(
                    self, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
                ),
                get_costmap=self.connection_interface.topic_latest("map", Costmap),
                get_robot_pos=lambda: self.connection_interface.transform_euler_pos("base_link"),
            )

            # Create the visualization stream at 5Hz
            self.local_planner_viz_stream = self.local_planner.create_stream(frequency_hz=5.0)
        else:
            self.local_planner = None
            self.global_planner = None
            self.local_planner_viz_stream = None

    def get_skills(self) -> Optional[SkillLibrary]:
        return self.skill_library

    def get_pose(self) -> dict:
        """
        Get the current pose (position and rotation) of the robot in the map frame.

        Returns:
            Dictionary containing:
                - position: Vector (x, y, z)
                - rotation: Vector (roll, pitch, yaw) in radians
        """
        position_tuple, orientation_tuple = self.connection_interface.get_pose_odom_transform()
        position = Vector(position_tuple[0], position_tuple[1], position_tuple[2])
        rotation = Vector(orientation_tuple[0], orientation_tuple[1], orientation_tuple[2])
        return {"position": position, "rotation": rotation}
