from abc import ABC, abstractmethod
from typing import Optional

from pydantic import Field
from dimos.hardware.interface import HardwareInterface
from dimos.agents.agent_config import AgentConfig
from dimos.robot.ros_control import ROSControl
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import operators as ops
from reactivex.scheduler import ThreadPoolScheduler
from dimos.stream.ros_video_provider import pool_scheduler
import os
import time
import logging

import multiprocessing
from dimos.robot.skills import AbstractSkill


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
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

    def start_ros_perception(self, fps: int = 30, save_frames: bool = True):
        """Start ROS-based perception system with rate limiting and frame processing."""
        if not self.ros_control or not self.ros_control.video_provider:
            raise RuntimeError("No ROS video provider available")
            
        print(f"Starting ROS video stream at {fps} FPS...")
        
        # Get base stream from video provider
        video_stream = self.ros_control.video_provider.capture_video_as_observable(fps=fps)
        
        # Add minimal processing pipeline with proper thread handling
        processed_stream = video_stream.pipe(
            ops.observe_on(pool_scheduler),  # Ensure thread safety
            ops.do_action(lambda x: print(f"ROBOT: Processing frame of type {type(x)}")),
            ops.share()  # Share the stream
        )
        
        return processed_stream
        
    @abstractmethod
    def move(self, x: float, y: float, yaw: float, duration: float = 0.0) -> bool:
        """Move the robot using velocity commands.
        
        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous
            
        Returns:
            bool: True if command was sent successfully
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for movement")
        return self.ros_control.move(x, y, yaw, duration)

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

class MyUnitreeSkills(AbstractSkill):
    """My Unitree Skills."""

    _robot: Optional[Robot] = None

    def __init__(self, robot: Optional[Robot] = None, **data):
        super().__init__(**data)
        self._robot: Robot = robot

    class Move(AbstractSkill):
        """Move the robot using velocity commands."""

        _robot: Robot = None
        _MOVE_PRINT_COLOR: str = "\033[32m"
        _MOVE_RESET_COLOR: str = "\033[0m"

        x: float = Field(..., description="Forward/backward velocity (m/s)")
        y: float = Field(..., description="Left/right velocity (m/s)")
        yaw: float = Field(..., description="Rotational velocity (rad/s)")
        duration: float = Field(..., description="How long to move (seconds). If 0, command is continuous")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._MOVE_PRINT_COLOR}Initializing Move Skill{self._MOVE_RESET_COLOR}")
            self._robot = robot
            print(f"{self._MOVE_PRINT_COLOR}Move Skill Initialized with Robot: {self._robot}{self._MOVE_RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to Move Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return self._robot.ros_control.move(self.x, self.y, self.yaw, self.duration)
