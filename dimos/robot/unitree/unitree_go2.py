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
from dimos.hardware.interface import HardwareInterface
from dimos.agents.agent import Agent, OpenAIAgent, OpenAIAgent
from dimos.robot.skills import AbstractSkill
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_provider import VideoProvider
from dimos.stream.video_providers.unitree import UnitreeVideoProvider
from dimos.stream.videostream import VideoStream
from dimos.stream.video_provider import AbstractVideoProvider
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import Observable, create
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
import asyncio
import logging
import threading
import time
from queue import Queue
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc import MediaStreamTrack
import os
from datetime import timedelta
from dotenv import load_dotenv, find_dotenv
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from reactivex.scheduler import ThreadPoolScheduler
from dimos.utils.logging_config import setup_logger
from dimos.perception.visual_servoing import VisualServoing
from dimos.perception.person_tracker import PersonTrackingStream

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
            enable_visual_servoing: bool = False,
            skills: Optional[Union[MyUnitreeSkills, AbstractSkill]] = None):

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
            skills: Skills instance. Can be MyUnitreeSkills for full functionality or any AbstractSkill for custom development..
        """
        print(f"Initializing UnitreeGo2 with use_ros: {use_ros} and use_webrtc: {use_webrtc}")
        if not (use_ros ^ use_webrtc):  # XOR operator ensures exactly one is True
            raise ValueError("Exactly one video/control provider (ROS or WebRTC) must be enabled")

        # Initialize ros_control if it is not provided and use_ros is True
        if ros_control is None and use_ros:
            ros_control = UnitreeROSControl(
                node_name="unitree_go2",
                disable_video_stream=disable_video_stream,
                mock_connection=mock_connection)

        super().__init__(ros_control=ros_control, output_dir=output_dir, skills=skills)

        # Unitree specific skill initialization
        if skills is not None:
            self.initialize_skills(skills)

        # camera stuff
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
        self.thread_pool_scheduler = ThreadPoolScheduler(
            self.optimal_thread_count // 2)

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
                ip=self.ip if connection_method == WebRTCConnectionMethod.LocalSTA else None)
        else:
            self.video_stream = None

        self.enable_visual_servoing = enable_visual_servoing
        # Initialize visual servoing if enabled
        if enable_visual_servoing and self.video_stream is not None:
            video_stream = self.get_ros_video_stream(fps=10)
            person_tracker = PersonTrackingStream(
                camera_intrinsics=self.camera_intrinsics,
                camera_pitch=self.camera_pitch,
                camera_height=self.camera_height
            )
            person_tracking_stream = person_tracker.create_stream(video_stream)
            self.visual_servoing = VisualServoing(tracking_stream=person_tracking_stream)
            self.person_tracking_stream = person_tracking_stream

    def follow_human(self, distance: int = 1.5, timeout: float = 20.0, point: Tuple[int, int] = None):
        if self.enable_visual_servoing:
            logger.warning(f"Following human for {timeout} seconds...")
            start_time = time.time()
            success = self.visual_servoing.start_tracking(point=point, desired_distance=distance)
            while self.visual_servoing.running and time.time() - start_time < timeout:
                output = self.visual_servoing.updateTracking()
                x_vel = output.get("linear_vel")
                z_vel = output.get("angular_vel")
                logger.debug(f"Following human: x_vel: {x_vel}, z_vel: {z_vel}")
                self.ros_control.move_vel_control(x=x_vel, y=0, yaw=z_vel)
                time.sleep(0.05)
            self.visual_servoing.stop_tracking()
            return success
        else:
            logger.warning("Visual servoing is disabled, cannot follow human")
            return False

    def do(self, *args, **kwargs):
        pass