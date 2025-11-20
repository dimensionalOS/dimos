#
#
#

"""
Monitor skill for Claude agent.

This module provides a skill that periodically sends images from the robot's
video stream to a Claude agent for analysis.
"""

import logging
import time
import threading
from typing import Optional, Any, Dict
import base64
import cv2
import reactivex as rx
from reactivex import operators as ops
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.agents.claude_agent import ClaudeAgent
from dimos.utils.threadpool import get_scheduler
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.monitor_skill", level=logging.INFO)

class MonitorSkill(AbstractRobotSkill):
    """
    A skill that periodically sends images from the robot's video stream to a Claude agent.
    
    This skill runs in a non-halting manner, allowing other skills to run concurrently.
    It can be used for passive monitoring, such as waiting for a person to enter a room.
    """
    
    timestep: float = Field(60.0, description="Time interval in seconds between monitoring queries")
    query_text: str = Field("What do you see in this image? Alert me if you see any people or important changes.", 
                           description="Query text to send to Claude agent with each image")
    max_duration: float = Field(0.0, description="Maximum duration to run the monitor in seconds (0 for indefinite)")
    
    def __init__(self, robot=None, claude_agent: Optional[ClaudeAgent] = None, **data):
        """
        Initialize the monitor skill.
        
        Args:
            robot: The robot instance
            claude_agent: The Claude agent to send queries to
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
        self._claude_agent = claude_agent
        self._stop_event = threading.Event()
        self._monitor_thread = None
        self._scheduler = get_scheduler()
        self._subscription = None
        
    def __call__(self):
        """
        Start the monitoring process in a separate thread using the threadpool.
        
        Returns:
            A message indicating the monitor has started
        """
        super().__call__()
        
        if self._claude_agent is None:
            error_msg = "No Claude agent provided to MonitorSkill"
            logger.error(error_msg)
            return error_msg
            
        if self._robot is None:
            error_msg = "No robot instance provided to MonitorSkill"
            logger.error(error_msg)
            return error_msg
            
        self.stop()
        
        self._stop_event.clear()
        
        interval_observable = rx.interval(self.timestep, scheduler=self._scheduler).pipe(
            ops.take_while(lambda _: not self._stop_event.is_set())
        )
        
        # Subscribe to the interval observable
        self._subscription = interval_observable.subscribe(
            on_next=self._monitor_iteration,
            on_error=lambda e: logger.error(f"Error in monitor observable: {e}"),
            on_completed=lambda: logger.info("Monitor observable completed")
        )
        
        skill_library = self._robot.get_skills()
        self.register_as_running("monitor", skill_library, self._subscription)
        
        logger.info(f"Monitor started with timestep={self.timestep}s, query='{self.query_text}'")
        return f"Monitor started with timestep={self.timestep}s, query='{self.query_text}'"
    
    def _monitor_iteration(self, iteration):
        """
        Execute a single monitoring iteration.
        
        Args:
            iteration: The iteration number (provided by rx.interval)
        """
        try:
            if self.max_duration > 0:
                elapsed_time = time.time() - self._start_time
                if elapsed_time > self.max_duration:
                    logger.info(f"Monitor reached maximum duration of {self.max_duration}s")
                    self.stop()
                    return
            
            logger.info(f"Monitor iteration {iteration} executing")
            
            # Get the video stream
            video_stream = self._robot.get_ros_video_stream()
            if video_stream is None:
                logger.error("Failed to get video stream from robot")
                return
            
            # Get a frame from the video stream
            frame = self._get_frame_from_stream(video_stream)
            
            if frame is not None:
                self._process_frame(frame)
            else:
                logger.warning("Failed to get frame from video stream")
                
        except Exception as e:
            logger.error(f"Error in monitor iteration {iteration}: {e}")
    
    def _get_frame_from_stream(self, video_stream):
        """
        Get a single frame from the video stream.
        
        Args:
            video_stream: The ROS video stream observable
            
        Returns:
            A single frame from the video stream, or None if no frame is available
        """
        frame = None
        
        frame_subject = rx.subject.Subject()
        
        subscription = video_stream.pipe(
            ops.take(1)  # Take just one frame
        ).subscribe(
            on_next=lambda x: frame_subject.on_next(x),
            on_error=lambda e: logger.error(f"Error getting frame: {e}")
        )
        
        timeout = 5.0  # 5 seconds timeout
        start_time = time.time()
        
        def on_frame(f):
            nonlocal frame
            frame = f
            
        frame_subject.subscribe(on_frame)
        
        while frame is None and time.time() - start_time < timeout:
            time.sleep(0.1)
            
        subscription.dispose()
        
        return frame
    
    def _process_frame(self, frame):
        """
        Process a frame with the Claude agent.
        
        Args:
            frame: The video frame to process
        """
        logger.info("Processing frame with Claude agent")
        
        try:
            _, buffer = cv2.imencode('.jpg', frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            response = self._claude_agent.direct_query(
                f"{self.query_text}\n\nHere is the current camera view from the robot:",
                base64_image=base64_image,
            )
            
            logger.info(f"Claude response: {response}")
            
        except Exception as e:
            logger.error(f"Error processing frame with Claude agent: {e}")
    
    def stop(self):
        """
        Stop the monitoring process.
        
        Returns:
            A message indicating the monitor has stopped
        """
        if self._subscription is not None and not self._subscription.is_disposed:
            logger.info("Stopping monitor")
            self._stop_event.set()
            self._subscription.dispose()
            self._subscription = None
            
            skill_library = self._robot.get_skills()
            self.unregister_as_running("monitor", skill_library)
            
            return "Monitor stopped"
        return "Monitor was not running"
    
    stop_monitoring = stop
