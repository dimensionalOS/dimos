from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import WebRtcReq
from enum import Enum, auto
import threading
import time
from typing import Optional, Tuple, Dict, Any, Type
from abc import ABC, abstractmethod

from dimos.robot.ros_control import ROSControl, RobotMode

class UnitreeROSControl(ROSControl):
    """Hardware interface for Unitree Go2 robot using ROS2"""
    
    # ROS Camera Topics
    CAMERA_TOPICS = {
        'raw': 'camera/image_raw',
        'compressed': 'camera/compressed',
        'info': 'camera/camera_info'
    }
    
    # Hard coded ROS Message types and Topic names for Unitree Go2
    DEFAULT_STATE_MSG_TYPE = Go2State
    DEFAULT_IMU_MSG_TYPE = IMU
    DEFAULT_WEBRTC_MSG_TYPE = WebRtcReq
    DEFAULT_STATE_TOPIC = 'go2_states'
    DEFAULT_IMU_TOPIC = 'imu'
    DEFAULT_WEBRTC_TOPIC = 'webrtc_req'
    DEFAULT_MAX_LINEAR_VELOCITY = 1.0
    DEFAULT_MAX_ANGULAR_VELOCITY = 2.0

    # Hard coded WebRTC API parameters for Unitree Go2
    DEFAULT_WEBRTC_API_TOPIC = 'rt/api/sport/request'
    
    def __init__(self, 
                 node_name: str = "unitree_hardware_interface",
                 state_topic: str = None,
                 imu_topic: str = None,
                 webrtc_topic: str = None,
                 webrtc_api_topic: str = None,
                 state_msg_type: Type = None,
                 imu_msg_type: Type = None,
                 webrtc_msg_type: Type = None,
                 max_linear_velocity: float = None,
                 max_angular_velocity: float = None,
                 use_compressed: bool = False,
                 use_raw: bool = True,
                 debug: bool = False,
                 disable_video_stream: bool = False,
                 mock_connection: bool = False):
        """
        Initialize Unitree ROS control interface with default values for Unitree Go2
        
        Args:
            node_name: Name for the ROS node
            state_topic: ROS Topic name for robot state (defaults to DEFAULT_STATE_TOPIC)
            imu_topic: ROS Topic name for IMU data (defaults to DEFAULT_IMU_TOPIC)
            webrtc_topic: ROS Topic for WebRTC commands (defaults to DEFAULT_WEBRTC_TOPIC)
            state_msg_type: ROS Message type for state data (defaults to DEFAULT_STATE_MSG_TYPE)
            imu_msg_type: ROS message type for IMU data (defaults to DEFAULT_IMU_MSG_TYPE)
            webrtc_msg_type: ROS message type for webrtc data (defaults to DEFAULT_WEBRTC_MSG_TYPE)
            max_linear_velocity: Maximum linear velocity in m/s (defaults to DEFAULT_MAX_LINEAR_VELOCITY)
            max_angular_velocity: Maximum angular velocity in rad/s (defaults to DEFAULT_MAX_ANGULAR_VELOCITY)
            use_compressed: Whether to use compressed video
            use_raw: Whether to use raw camera topics
            debug: Whether to enable debug logging
            disable_video_stream: Whether to run without video stream for testing.
            mock_connection: Whether to run without active ActionClient servers for testing. 
        """
        
        # Select which camera topics to use
        active_camera_topics = None
        if not disable_video_stream:
            active_camera_topics = {
                'main': self.CAMERA_TOPICS['raw' if use_raw else 'compressed']
            }
        
        # Use default values if not provided
        state_topic = state_topic or self.DEFAULT_STATE_TOPIC
        imu_topic = imu_topic or self.DEFAULT_IMU_TOPIC
        webrtc_topic = webrtc_topic or self.DEFAULT_WEBRTC_TOPIC
        webrtc_api_topic = webrtc_api_topic or self.DEFAULT_WEBRTC_API_TOPIC
        state_msg_type = state_msg_type or self.DEFAULT_STATE_MSG_TYPE
        imu_msg_type = imu_msg_type or self.DEFAULT_IMU_MSG_TYPE
        webrtc_msg_type = webrtc_msg_type or self.DEFAULT_WEBRTC_MSG_TYPE
        max_linear_velocity = max_linear_velocity or self.DEFAULT_MAX_LINEAR_VELOCITY
        max_angular_velocity = max_angular_velocity or self.DEFAULT_MAX_ANGULAR_VELOCITY
        
        super().__init__(
            node_name=node_name,
            camera_topics=active_camera_topics,
            use_compressed_video=use_compressed,
            mock_connection=mock_connection,
            state_topic=state_topic,
            imu_topic=imu_topic,
            state_msg_type=state_msg_type,
            imu_msg_type=imu_msg_type,
            webrtc_msg_type=webrtc_msg_type,
            webrtc_topic=webrtc_topic,
            webrtc_api_topic=webrtc_api_topic,
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            debug=debug
        )
    
    # Unitree-specific RobotMode State update conditons
    def _update_mode(self, msg: Go2State):
        """
        Implementation of abstract method to update robot mode
        
        Logic:
        - If progress is 0 and mode is 1, then state is IDLE
        - If progress is 1 OR mode is NOT equal to 1, then state is MOVING
        """
        # Direct access to protected instance variables from the parent class
        mode = msg.mode
        progress = msg.progress
                
        if progress == 0 and mode == 1:
            self._mode = RobotMode.IDLE
            self._logger.debug("Robot mode set to IDLE (progress=0, mode=1)")
        elif progress == 1 or mode != 1:
            self._mode = RobotMode.MOVING
            self._logger.debug(f"Robot mode set to MOVING (progress={progress}, mode={mode})")
        else:
            self._mode = RobotMode.UNKNOWN
            self._logger.debug(f"Robot mode set to UNKNOWN (progress={progress}, mode={mode})")