from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import WebRtcReq
from enum import Enum, auto
import threading
import time
from typing import Optional, Tuple, Dict, Any
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
    
    def __init__(self, 
                 node_name: str = "unitree_hardware_interface",
                 state_topic: str = 'go2_states',
                 imu_topic: str = 'imu',
                 webrtc_topic: str = 'webrtc_req',
                 use_compressed: bool = False,
                 use_raw: bool = True):
        # Select which camera topics to use
        active_camera_topics = {
            'main': self.CAMERA_TOPICS['raw' if use_raw else 'compressed']
        }
        
        super().__init__(
            node_name=node_name,
            camera_topics=active_camera_topics,
            use_compressed_video=use_compressed,
            state_topic=state_topic,
            imu_topic=imu_topic,
            webrtc_topic=webrtc_topic,
            state_msg_type=Go2State,
            imu_msg_type=IMU,
            webrtc_msg_type=WebRtcReq
        )
    
    # Unitree-specific RobotMode update conditons
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