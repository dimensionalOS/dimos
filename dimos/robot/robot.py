"""Base module for all DIMOS robots.

This module provides the foundation for all DIMOS robots, including both physical 
and simulated implementations, with common functionality for movement, control, 
and video streaming.
"""

from abc import ABC, abstractmethod
from typing import Optional, Type, TYPE_CHECKING
from pydantic import Field
from dimos.hardware.interface import HardwareInterface
from dimos.robot.ros_control import ROSControl
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import Observable, operators as ops
import os
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.utils.threadpool import get_scheduler

# Use TYPE_CHECKING to avoid circular imports
if TYPE_CHECKING:
    from dimos.robot.skills import AbstractSkill
else:
    # Use a forward reference for runtime
    AbstractSkill = 'AbstractSkill'


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
        skills: Robot skills instance for executing various robot actions.
    """

    def __init__(self,
                 hardware_interface: HardwareInterface = None,
                 ros_control: ROSControl = None,
                 output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
                 pool_scheduler: ThreadPoolScheduler = None,
                 skills: Optional[AbstractSkill] = None):
        """Initialize a Robot instance.
        
        Args:
            hardware_interface: Interface to the robot's hardware. Defaults to None.
            ros_control: ROS-based control system. Defaults to None.
            output_dir: Directory for storing output files. Defaults to "./assets/output".
            pool_scheduler: Thread pool scheduler. If None, one will be created.
            skills: Robot skills instance. Defaults to None.
        """
        self.hardware_interface = hardware_interface
        self.ros_control = ros_control
        self.output_dir = output_dir
        self.disposables = CompositeDisposable()
        self.pool_scheduler = pool_scheduler if pool_scheduler else get_scheduler()
        self.skills = None

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

        # Handles skills initialization in Robot(..., skills=AbstractSkill()) AND standalone 
        # via skills_instance = AbstractSkill(robot=robot)
        if skills is not None:
            if hasattr(skills, '_robot') and skills._robot is not None:
                # Skills already initialized with robot reference in AbstractSkill constructor
                pass
            else:
                # New skills instance needs robot reference
                self.skills = skills
                self.skills.set_robot(self)

    def get_ros_video_stream(self, fps: int = 30) -> Observable:
        """Get the ROS video stream with rate limiting and frame processing.
        
        Args:
            fps: Frames per second for the video stream. Defaults to 30.
            
        Returns:
            Observable: An observable stream of video frames.
            
        Raises:
            RuntimeError: If no ROS video provider is available.
        """
        if not self.ros_control or not self.ros_control.video_provider:
            raise RuntimeError("No ROS video provider available")

        print(f"Starting ROS video stream at {fps} FPS...")

        # Get base stream from video provider
        video_stream = self.ros_control.video_provider.capture_video_as_observable(
            fps=fps)

        # Add minimal processing pipeline with proper thread handling
        processed_stream = video_stream.pipe(
            ops.subscribe_on(self.pool_scheduler),
            ops.observe_on(self.pool_scheduler),  # Ensure thread safety
            ops.share()  # Share the stream
        )

        return processed_stream

    def move(self, distance: float, speed: float = 0.5) -> bool:
        """Move the robot using velocity commands.
        
        Args:
            distance: Distance to move forward in meters (must be positive).
            speed: Speed to move at in m/s. Defaults to 0.5.
            
        Returns:
            bool: True if movement succeeded.
            
        Raises:
            RuntimeError: If no ROS control interface is available.
        """
        if self.ros_control is None:
            raise RuntimeError(
                "No ROS control interface available for movement")
        return self.ros_control.move(distance, speed)

    def reverse(self, distance: float, speed: float = 0.5) -> bool:
        """Move the robot backward by a specified distance.
        
        Args:
            distance: Distance to move backward in meters (must be positive).
            speed: Speed to move at in m/s. Defaults to 0.5.
            
        Returns:
            bool: True if movement succeeded.
            
        Raises:
            RuntimeError: If no ROS control interface is available.
        """
        if self.ros_control is None:
            raise RuntimeError(
                "No ROS control interface available for movement")
        return self.ros_control.reverse(distance, speed)

    def spin(self, degrees: float, speed: float = 45.0) -> bool:
        """Rotate the robot by a specified angle.

        Args:
            degrees: Angle to rotate in degrees (positive for counter-clockwise, 
                negative for clockwise).
            speed: Angular speed in degrees/second. Defaults to 45.0.
            
        Returns:
            bool: True if rotation succeeded.
            
        Raises:
            RuntimeError: If no ROS control interface is available.
        """
        if self.ros_control is None:
            raise RuntimeError(
                "No ROS control interface available for rotation")
        return self.ros_control.spin(degrees, speed)

    def webrtc_req(self, api_id: int, topic: str = None, parameter: str = '', 
                  priority: int = 0, request_id: str = None, data=None, timeout: float = 1000.0) -> bool:
        """Send a WebRTC request command to the robot.
        
        Args:
            api_id: The API ID for the command.
            topic: The API topic to publish to. Defaults to ROSControl.webrtc_api_topic.
            parameter: Optional parameter string. Defaults to ''.
            priority: Priority level as defined by PriorityQueue(). Defaults to 0 (no priority).
            data: Optional data dictionary.
            timeout: Maximum time to wait for the command to complete.

        Returns:
            bool: True if command was sent successfully.
            
        Raises:
            RuntimeError: If no ROS control interface is available.

        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for WebRTC commands")
        return self.ros_control.queue_webrtc_req(
            api_id=api_id, 
            topic=topic,
            parameter=parameter, 
            priority=priority,
            request_id=request_id,
            data=data,
            timeout=timeout
        )

    @abstractmethod
    def do(self, *args, **kwargs):
        """Executes motion on the robot.
        
        This abstract method must be implemented by concrete robot subclasses
        to provide robot-specific motion functionality.
        
        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
            
        Returns:
            Implementation-dependent.
        """
        pass

    def update_hardware_interface(self,
                                  new_hardware_interface: HardwareInterface):
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

    def initialize_skills(self, skills: Optional[AbstractSkill]):
        """Initialize the robot's skills instance.
        
        Args:
            skills: Skills instance to initialize for this robot.
            
        Returns:
            The initialized skills instance.
        """

        skills.initialize_skills()
    
    def get_skills(self) -> Optional[AbstractSkill]:
        """Get the robot's skills instance.
        
        Returns:
            The robot's skills instance if one is set, None otherwise.
        """
        return None if self.skills is None else self.skills

    def set_hardware_configuration(self, configuration):
        """Set a new hardware configuration.
        
        Args:
            configuration: The new hardware configuration to set.
            
        Raises:
            AttributeError: If hardware_interface is None.
        """
        self.hardware_interface.set_configuration(configuration)

    def cleanup(self):
        """Clean up resources used by the robot.
        
        This method should be called when the robot is no longer needed to
        ensure proper release of resources such as ROS connections and
        subscriptions.
        """
        if self.ros_control:
            self.ros_control.cleanup()
        self.disposables.dispose()
