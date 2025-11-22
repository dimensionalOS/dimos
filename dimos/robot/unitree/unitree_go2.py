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
from dimos.stream.videostream import VideoStream
from dimos.stream.video_provider import AbstractVideoProvider
from dimos.stream.video_operators import VideoOperators as vops
from dimos.models.qwen.video_query import get_bbox_from_qwen
from reactivex import operators as RxOps
import json
from reactivex import Observable, create
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
import logging
import time
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import WebRTCConnectionMethod
import os
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from reactivex.scheduler import ThreadPoolScheduler
import threading
from dimos.utils.logging_config import setup_logger
from dimos.perception.visual_servoing import VisualServoing
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.local_planner import VFHPurePursuitPlanner
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.utils.ros_utils import distance_angle_to_goal_xy
from dimos.utils.generic_subscriber import GenericSubscriber
from nav_msgs import msg
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.skills.visual_navigation_skills import FollowHuman, NavigateToObject

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
        output_dir: str = os.getcwd(),  # TODO: Pull from ENV variable to handle docker and local development
        use_ros: bool = True,
        use_webrtc: bool = False,
        disable_video_stream: bool = False,
        mock_connection: bool = False,
        skills: Optional[Union[MyUnitreeSkills, AbstractSkill]] = None,
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

        super().__init__(ros_control=ros_control, output_dir=output_dir, skill_library=skills)

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
        self.output_dir = output_dir
        self.ip = ip
        self.disposables = CompositeDisposable()
        self.main_stream_obs = None

        # Initialize thread pool scheduler
        self.optimal_thread_count = multiprocessing.cpu_count()
        self.thread_pool_scheduler = ThreadPoolScheduler(self.optimal_thread_count // 2)

        if (connection_method == WebRTCConnectionMethod.LocalSTA) and (ip is None):
            raise ValueError("IP address is required for LocalSTA connection")

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        print(f"Agent outputs will be saved to: {os.path.join(self.output_dir, 'memory.txt')}")

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
            robot=self,
            robot_width=0.36,  # Unitree Go2 width in meters
            robot_length=0.6,  # Unitree Go2 length in meters
            max_linear_vel=0.5,
            lookahead_distance=1.0,
            visualization_size=500  # 500x500 pixel visualization
        )

        self.global_planner = AstarPlanner(
            costmap=lambda: Costmap.from_msg(self.ros_control.topic_latest("map", msg.OccupancyGrid)()),
            base_link=lambda: self.ros_control.transform_euler("base_link"),
            local_nav=self.navigate_path_local
        )

        # Create the visualization stream at 5Hz
        self.local_planner_viz_stream = self.local_planner.create_stream(frequency_hz=5.0)

    def navigate_to_goal_local(
        self, goal_xy_robot: Tuple[float, float], is_robot_frame=True, timeout: float = 60.0
    ) -> bool:
        """
        Navigates the robot to a goal specified in the robot's local frame
        using the VFHPurePursuitPlanner.

        Args:
            goal_xy_robot: Tuple (x, y) representing the goal position relative
                           to the robot's current position and orientation.
            timeout: Maximum time (in seconds) allowed to reach the goal.

        Returns:
            bool: True if the goal was reached within the timeout, False otherwise.
        """
        logger.info(f"Starting navigation to local goal {goal_xy_robot} with timeout {timeout}s.")

        # Set the single goal in the robot's frame. Adjustment will happen internally.
        self.local_planner.set_goal(goal_xy_robot, is_robot_frame=is_robot_frame)

        start_time = time.time()
        goal_reached = False

        try:
            while time.time() - start_time < timeout:
                # Check if goal has been reached
                if self.local_planner.is_goal_reached():
                    logger.info("Goal reached successfully.")
                    goal_reached = True
                    break

                # Get planned velocity towards the goal
                vel_command = self.local_planner.plan()
                x_vel = vel_command.get("x_vel", 0.0)
                angular_vel = vel_command.get("angular_vel", 0.0)

                # Send velocity command
                self.ros_control.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

                # Control loop frequency
                time.sleep(0.1)

            if not goal_reached:
                logger.warning(f"Navigation timed out after {timeout} seconds before reaching goal.")

        except KeyboardInterrupt:
            logger.info("Navigation to local goal interrupted by user.")
            goal_reached = False  # Consider interruption as failure
        except Exception as e:
            logger.error(f"Error during navigation to local goal: {e}")
            goal_reached = False  # Consider error as failure
        finally:
            logger.info("Stopping robot after navigation attempt.")
            self.ros_control.stop()

        return goal_reached

    def navigate_path_local(self, path: Path, timeout: float = 120.0, goal_theta: Optional[float] = None, stop_event: Optional[threading.Event] = None) -> bool:
        """
        Navigates the robot along a path of waypoints using the waypoint following capability
        of the VFHPurePursuitPlanner.

        Args:
            path: Path object containing waypoints in odom/map frame
            timeout: Maximum time (in seconds) allowed to follow the complete path
            stop_event: Optional threading.Event to signal when navigation should stop

        Returns:
            bool: True if the entire path was successfully followed, False otherwise
        """
        logger.info(f"Starting navigation along path with {len(path)} waypoints and timeout {timeout}s.")

        # Set the path in the local planner
        self.local_planner.set_goal_waypoints(path, goal_theta=goal_theta)

        start_time = time.time()
        path_completed = False

        try:
            while time.time() - start_time < timeout and not (stop_event and stop_event.is_set()):
                # Check if the entire path has been traversed
                if self.local_planner.is_goal_reached():
                    logger.info("Path traversed successfully.")
                    path_completed = True
                    break 

                # Get planned velocity towards the current waypoint target
                vel_command = self.local_planner.plan()
                x_vel = vel_command.get('x_vel', 0.0)
                angular_vel = vel_command.get('angular_vel', 0.0)

                # Send velocity command
                self.ros_control.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

                # Control loop frequency
                time.sleep(0.1)
            
            if not path_completed:
                logger.warning(f"Path following timed out after {timeout} seconds before completing the path.")

        except KeyboardInterrupt:
            logger.info("Path navigation interrupted by user.")
            path_completed = False
        except Exception as e:
            logger.error(f"Error during path navigation: {e}")
            path_completed = False
        finally:
            logger.info("Stopping robot after path navigation attempt.")
            self.ros_control.stop()
            
        return path_completed

    def get_skills(self) -> Optional[SkillLibrary]:
        return self.skill_library
