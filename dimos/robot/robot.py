from abc import ABC, abstractmethod
from typing import Optional
from pydantic import Field
from dimos.hardware.interface import HardwareInterface
from dimos.agents.agent_config import AgentConfig
from dimos.robot.ros_control import ROSControl
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import Observable, operators as ops
from reactivex.scheduler import ThreadPoolScheduler
from dimos.stream.ros_video_provider import pool_scheduler
import os
from reactivex.disposable import CompositeDisposable

'''
Base class for all dimos robots, both physical and simulated.
'''
class Robot(ABC):
    def __init__(self,
                 agent_config: AgentConfig = None,
                 hardware_interface: HardwareInterface = None,
                 ros_control: ROSControl = None,
                 output_dir: str = os.path.join(os.getcwd(), "output")):
        
        self.agent_config = agent_config
        self.hardware_interface = hardware_interface
        self.ros_control = ros_control
        self.output_dir = output_dir
        self.disposables = CompositeDisposable()
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

    def get_ros_video_stream(self, fps: int = 30, save_frames: bool = True) -> Observable:
        """Get the ROS video stream with rate limiting and frame processing."""
        if not self.ros_control or not self.ros_control.video_provider:
            raise RuntimeError("No ROS video provider available")
            
        print(f"Starting ROS video stream at {fps} FPS...")
        
        # Get base stream from video provider
        video_stream = self.ros_control.video_provider.capture_video_as_observable(fps=fps)
        
        # Add minimal processing pipeline with proper thread handling
        processed_stream = video_stream.pipe(
            ops.observe_on(pool_scheduler),  # Ensure thread safety
            ops.share()  # Share the stream
        )
        
        return processed_stream
        
    def move(self, distance: float, speed: float = 0.5) -> bool:
        """Move the robot using velocity commands.
        
        Args:
            distance: Distance to move forward in meters (must be positive)
            speed: Speed to move at in m/s (default 0.5)
        Returns:
            bool: True if movement succeeded
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for movement")
        return self.ros_control.move(distance, speed)
    
    def reverse(self, distance: float, speed: float = 0.5) -> bool:
        """Move the robot backward by a specified distance.
        
        Args:
            distance: Distance to move backward in meters (must be positive)
            speed: Speed to move at in m/s (default 0.5)
        Returns:
            bool: True if movement succeeded
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for movement")
        return self.ros_control.reverse(distance, speed)
    
    def spin(self, degrees: float, speed: float = 45.0) -> bool:
        """Rotate the robot by a specified angle.

        Args:
            degrees: Angle to rotate in degrees (positive for counter-clockwise, negative for clockwise)
            speed: Angular speed in degrees/second (default 45.0)
        Returns:
            bool: True if rotation succeeded
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for rotation")
        return self.ros_control.spin(degrees, speed)
    
    def webrtc_req(self, api_id: int, topic: str = 'rt/api/sport/request', parameter: str = '', priority: int = 0) -> bool:
        """Send a WebRTC request command to the robot.
        
        Args:
            api_id: The API ID for the command
            topic: The topic to publish to (e.g. 'rt/api/sport/request')
            parameter: Optional parameter string
            priority: Priority level (0 or 1)
            
        Returns:
            bool: True if command was sent successfully
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for WebRTC commands")
        return self.ros_control.queue_webrtc_req(api_id, topic, parameter, priority)

    @abstractmethod
    def do(self, *args, **kwargs):
     """Executes motion."""
    pass
    def update_hardware_interface(self, new_hardware_interface: HardwareInterface):
        """Update the hardware interface with a new configuration."""
        self.hardware_interface = new_hardware_interface

    def get_hardware_configuration(self):
        """Retrieve the current hardware configuration."""
        return self.hardware_interface.get_configuration()

    def set_hardware_configuration(self, configuration):
        """Set a new hardware configuration."""
        self.hardware_interface.set_configuration(configuration)


    def cleanup(self):
        """Cleanup resources."""
        if self.ros_control:
            self.ros_control.cleanup()
        self.disposables.dispose()
