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

"""Base module for all DIMOS robots.

This module provides the foundation for all DIMOS robots, including both physical
and simulated implementations, with common functionality for movement, control,
and video streaming.
"""

from abc import ABC, abstractmethod
import os
from typing import Optional, List, Union, Dict, Any

from dimos.hardware.interface import HardwareInterface
from dimos.perception.spatial_perception import SpatialMemory
from dimos.manipulation.manipulation_interface import ManipulationInterface
from dimos.types.robot_capabilities import RobotCapability
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger
from dimos.robot.connection_interface import ConnectionInterface

from dimos.skills.skills import SkillLibrary
from reactivex import Observable, operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.utils.threadpool import get_scheduler
from dimos.utils.reactive import backpressure
from dimos.stream.video_provider import VideoProvider

logger = setup_logger("dimos.robot.robot")


class Robot(ABC):
    """Base class for all DIMOS robots.

    This abstract base class defines the common interface and functionality for all
    DIMOS robots, whether physical or simulated. It provides methods for movement,
    rotation, video streaming, and hardware configuration management.

    Attributes:
        agent_config: Configuration for the robot's agent.
        hardware_interface: Interface to the robot's hardware components.
        ros_control: ROS-based control system for the robot.
        output_dir: Directory for storing output files.
        disposables: Collection of disposable resources for cleanup.
        pool_scheduler: Thread pool scheduler for managing concurrent operations.
    """

    def __init__(
        self,
        hardware_interface: HardwareInterface = None,
        connection_interface: ConnectionInterface = None,
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
        pool_scheduler: ThreadPoolScheduler = None,
        skill_library: SkillLibrary = None,
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = False,
        capabilities: List[RobotCapability] = None,
        video_stream: Optional[Observable] = None,
        enable_perception: bool = True,
    ):
        """Initialize a Robot instance.

        Args:
            hardware_interface: Interface to the robot's hardware. Defaults to None.
            connection_interface: Connection interface for robot control and communication.
            output_dir: Directory for storing output files. Defaults to "./assets/output".
            pool_scheduler: Thread pool scheduler. If None, one will be created.
            skill_library: Skill library instance. If None, one will be created.
            spatial_memory_collection: Name of the collection in the ChromaDB database.
            new_memory: If True, creates a new spatial memory from scratch. Defaults to False.
            capabilities: List of robot capabilities. Defaults to None.
            video_stream: Optional video stream. Defaults to None.
            enable_perception: If True, enables perception streams and spatial memory. Defaults to True.
        """
        self.hardware_interface = hardware_interface
        self.connection_interface = connection_interface
        self.output_dir = output_dir
        self.disposables = CompositeDisposable()
        self.pool_scheduler = pool_scheduler if pool_scheduler else get_scheduler()
        self.skill_library = skill_library if skill_library else SkillLibrary()
        self.enable_perception = enable_perception

        # Initialize robot capabilities
        self.capabilities = capabilities or []

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

        # Initialize memory properties
        self.memory_dir = os.path.join(self.output_dir, "memory")
        os.makedirs(self.memory_dir, exist_ok=True)

        # Initialize spatial memory properties
        self.spatial_memory_dir = os.path.join(self.memory_dir, "spatial_memory")
        self.spatial_memory_collection = spatial_memory_collection
        self.db_path = os.path.join(self.spatial_memory_dir, "chromadb_data")
        self.visual_memory_path = os.path.join(self.spatial_memory_dir, "visual_memory.pkl")

        # Create spatial memory directory
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

        # Initialize spatial memory properties
        self._video_stream = video_stream

        # Only create video stream if connection interface is available
        if self.connection_interface is not None:
            # Get video stream - always create this, regardless of enable_perception
            self._video_stream = self.get_video_stream(fps=10)  # Lower FPS for processing

        # Create SpatialMemory instance only if perception is enabled
        if self.enable_perception:
            self._spatial_memory = SpatialMemory(
                collection_name=self.spatial_memory_collection,
                db_path=self.db_path,
                visual_memory_path=self.visual_memory_path,
                new_memory=new_memory,
                output_dir=self.spatial_memory_dir,
                video_stream=self._video_stream,
                get_pose=self.get_pose,
            )
            logger.info("Spatial memory initialized")
        else:
            self._spatial_memory = None
            logger.info("Spatial memory disabled (enable_perception=False)")

        # Initialize manipulation interface if the robot has manipulation capability
        self._manipulation_interface = None
        if RobotCapability.MANIPULATION in self.capabilities:
            # Initialize manipulation memory properties if the robot has manipulation capability
            self.manipulation_memory_dir = os.path.join(self.memory_dir, "manipulation_memory")

            # Create manipulation memory directory
            os.makedirs(self.manipulation_memory_dir, exist_ok=True)

            self._manipulation_interface = ManipulationInterface(
                output_dir=self.output_dir,  # Use the main output directory
                new_memory=new_memory,
            )
            logger.info("Manipulation interface initialized")

    def get_video_stream(self, fps: int = 30) -> Observable:
        """Get the video stream with rate limiting and frame processing.

        Args:
            fps: Frames per second for the video stream. Defaults to 30.

        Returns:
            Observable: An observable stream of video frames.

        Raises:
            RuntimeError: If no connection interface is available for video streaming.
        """
        if self.connection_interface is None:
            raise RuntimeError("No connection interface available for video streaming")

        stream = self.connection_interface.get_video_stream(fps)
        if stream is None:
            raise RuntimeError("No video stream available from connection interface")

        return stream.pipe(
            ops.observe_on(self.pool_scheduler),
        )

    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        """Move the robot using velocity commands.

        Args:
            velocity: Velocity vector [x, y, yaw] where:
                     x: Linear velocity in x direction (m/s)
                     y: Linear velocity in y direction (m/s)
                     yaw: Angular velocity (rad/s)
            duration: Duration to apply command (seconds). If 0, apply once.

        Returns:
            bool: True if movement succeeded.

        Raises:
            RuntimeError: If no connection interface is available.
        """
        if self.connection_interface is None:
            raise RuntimeError("No connection interface available for movement")

        return self.connection_interface.move(velocity, duration)

    def spin(self, degrees: float, speed: float = 45.0) -> bool:
        """Rotate the robot by a specified angle.

        Args:
            degrees: Angle to rotate in degrees (positive for counter-clockwise,
                negative for clockwise).
            speed: Angular speed in degrees/second. Defaults to 45.0.

        Returns:
            bool: True if rotation succeeded.

        Raises:
            RuntimeError: If no connection interface is available.
        """
        if self.connection_interface is None:
            raise RuntimeError("No connection interface available for rotation")

        # Convert degrees to radians
        import math

        angular_velocity = math.radians(speed)
        duration = abs(degrees) / speed if speed > 0 else 0

        # Set direction based on sign of degrees
        if degrees < 0:
            angular_velocity = -angular_velocity

        velocity = Vector(0.0, 0.0, angular_velocity)
        return self.connection_interface.move(velocity, duration)

    @abstractmethod
    def get_pose(self) -> dict:
        """
        Get the current pose (position and rotation) of the robot.

        Returns:
            Dictionary containing:
                - position: Tuple[float, float, float] (x, y, z)
                - rotation: Tuple[float, float, float] (roll, pitch, yaw) in radians
        """
        pass

    def webrtc_req(
        self,
        api_id: int,
        topic: str = None,
        parameter: str = "",
        priority: int = 0,
        request_id: str = None,
        data=None,
        timeout: float = 1000.0,
    ):
        """Send a WebRTC request command to the robot.

        Args:
            api_id: The API ID for the command.
            topic: The API topic to publish to. Defaults to ROSControl.webrtc_api_topic.
            parameter: Additional parameter data. Defaults to "".
            priority: Priority of the request. Defaults to 0.
            request_id: Unique identifier for the request. If None, one will be generated.
            data: Additional data to include with the request. Defaults to None.
            timeout: Timeout for the request in milliseconds. Defaults to 1000.0.

        Returns:
            The result of the WebRTC request.

        Raises:
            RuntimeError: If no connection interface with WebRTC capability is available.
        """
        if self.connection_interface is None:
            raise RuntimeError("No connection interface available for WebRTC commands")

        # WebRTC requests are only available on ROS control interfaces
        if hasattr(self.connection_interface, "queue_webrtc_req"):
            return self.connection_interface.queue_webrtc_req(
                api_id=api_id,
                topic=topic,
                parameter=parameter,
                priority=priority,
                request_id=request_id,
                data=data,
                timeout=timeout,
            )
        else:
            raise RuntimeError("WebRTC requests not supported by this connection interface")

    def pose_command(self, roll: float, pitch: float, yaw: float) -> bool:
        """Send a pose command to the robot.

        Args:
            roll: Roll angle in radians.
            pitch: Pitch angle in radians.
            yaw: Yaw angle in radians.

        Returns:
            bool: True if command was sent successfully.

        Raises:
            RuntimeError: If no connection interface with pose command capability is available.
        """
        # Pose commands are only available on ROS control interfaces
        if hasattr(self.connection_interface, "pose_command"):
            return self.connection_interface.pose_command(roll, pitch, yaw)
        else:
            raise RuntimeError("Pose commands not supported by this connection interface")

    def update_hardware_interface(self, new_hardware_interface: HardwareInterface):
        """Update the hardware interface with a new configuration.

        Args:
            new_hardware_interface: New hardware interface to use for the robot.
        """
        self.hardware_interface = new_hardware_interface

    def get_hardware_configuration(self):
        """Retrieve the current hardware configuration.

        Returns:
            The current hardware configuration from the hardware interface.

        Raises:
            AttributeError: If hardware_interface is None.
        """
        return self.hardware_interface.get_configuration()

    def set_hardware_configuration(self, configuration):
        """Set a new hardware configuration.

        Args:
            configuration: The new hardware configuration to set.

        Raises:
            AttributeError: If hardware_interface is None.
        """
        self.hardware_interface.set_configuration(configuration)

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory.

        Returns:
            SpatialMemory: The robot's spatial memory system, or None if perception is disabled.
        """
        return self._spatial_memory

    @property
    def manipulation_interface(self) -> Optional[ManipulationInterface]:
        """Get the robot's manipulation interface.

        Returns:
            ManipulationInterface: The robot's manipulation interface or None if not available.
        """
        return self._manipulation_interface

    def has_capability(self, capability: RobotCapability) -> bool:
        """Check if the robot has a specific capability.

        Args:
            capability: The capability to check for

        Returns:
            bool: True if the robot has the capability, False otherwise
        """
        return capability in self.capabilities

    def get_spatial_memory(self) -> Optional[SpatialMemory]:
        """Simple getter for the spatial memory instance.
        (For backwards compatibility)

        Returns:
            The spatial memory instance or None if not set.
        """
        return self._spatial_memory if self._spatial_memory else None

    @property
    def video_stream(self) -> Optional[Observable]:
        """Get the robot's video stream.

        Returns:
            Observable: The robot's video stream or None if not available.
        """
        return self._video_stream

    def get_skills(self):
        """Get the robot's skill library.

        Returns:
            The robot's skill library for adding/managing skills.
        """
        return self.skill_library

    def cleanup(self):
        """Clean up resources used by the robot.

        This method should be called when the robot is no longer needed to
        ensure proper release of resources such as ROS connections and
        subscriptions.
        """
        # Dispose of resources
        if self.disposables:
            self.disposables.dispose()

        # Clean up connection interface
        if self.connection_interface:
            self.connection_interface.disconnect()

        self.disposables.dispose()


class MockRobot(Robot):
    def __init__(self):
        super().__init__()
        self.ros_control = None
        self.hardware_interface = None
        self.skill_library = SkillLibrary()

    def my_print(self):
        print("Hello, world!")


class MockManipulationRobot(Robot):
    def __init__(self, skill_library: Optional[SkillLibrary] = None):
        video_provider = VideoProvider("webcam", video_source=0)  # Default camera
        video_stream = backpressure(
            video_provider.capture_video_as_observable(realtime=True, fps=30)
        )

        super().__init__(
            capabilities=[RobotCapability.MANIPULATION],
            video_stream=video_stream,
            skill_library=skill_library,
        )
        self.camera_intrinsics = [489.33, 367.0, 320.0, 240.0]
        self.ros_control = None
        self.hardware_interface = None
