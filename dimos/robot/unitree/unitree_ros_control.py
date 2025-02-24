from go2_interfaces.msg import Go2State, IMU
from enum import Enum, auto
import threading
import time
from typing import Optional, Tuple, Dict, Any
from abc import ABC, abstractmethod

from dimos.robot.ros_control import ROSControl, RobotMode

class UnitreeROSControl(ROSControl):
    """Hardware interface for Unitree Go2 robot using ROS2"""
    
    # Define topics as class constants
    CAMERA_TOPICS = {
        'raw': 'camera/image_raw',
        'compressed': 'camera/compressed',
        'info': 'camera/camera_info'
    }
    
    def __init__(self, 
                 node_name: str = "unitree_hardware_interface",
                 use_compressed: bool = False,
                 use_raw: bool = True):
        # Select which camera topics to use
        active_camera_topics = {
            'main': self.CAMERA_TOPICS['raw' if use_raw else 'compressed']
        }
        
        super().__init__(
            node_name=node_name,
            cmd_vel_topic='cmd_vel_out',
            camera_topics=active_camera_topics,
            use_compressed_video=use_compressed
        )
        
        # Unitree-specific state tracking
        self._robot_state: Optional[Go2State] = None
        self._imu_state: Optional[IMU] = None
        
        # Unitree-specific subscribers
        self._state_sub = self._node.create_subscription(
            Go2State, 'go2_states', self._state_callback, 10)
        self._imu_sub = self._node.create_subscription(
            IMU, 'imu', self._imu_callback, 10)
    
    def _state_callback(self, msg: Go2State):
        """Callback for robot state updates"""
        self._robot_state = msg
        self._update_mode(msg)
    
    def _imu_callback(self, msg: IMU):
        """Callback for IMU data"""
        self._imu_state = msg
    
    def _update_mode(self, state_msg: Go2State):
        """Implementation of abstract method to update robot mode"""
        if state_msg.mode == 0:
            self._mode = RobotMode.IDLE
        elif state_msg.mode == 1:
            self._mode = RobotMode.STANDING
        elif state_msg.mode == 2:
            self._mode = RobotMode.MOVING
    
    def get_state(self) -> Dict:
        """Implementation of abstract method to get robot state"""
        return {
            "mode": self._mode,
            "is_moving": self._is_moving,
            "velocity": self._current_velocity,
            "robot_state": self._robot_state,
            "imu_state": self._imu_state
        }

def main(args=None):
    """Example usage of the UnitreeROSControl class"""
    try:
        # Initialize control interface
        robot = UnitreeROSControl()
        
        # Example movement sequence
        print("Moving forward...")
        robot.move(-0.1, 0.0, 0.0, duration=20.0)  # Move forward for 2 seconds
        time.sleep(0.5)
        
        # print("Moving left...")
        # robot.move(0.0, 0.3, 0.0, duration=1.0)  # Move left for 1 second
        # time.sleep(0.5)
        
        # print("Rotating...")
        # robot.move(0.0, 0.0, 0.5, duration=1.0)  # Rotate for 1 second
        # time.sleep(0.5)
        
        print("Getting robot state...")
        state = robot.get_state()
        print(f"Robot state: {state}")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if 'robot' in locals():
            robot.cleanup()

if __name__ == '__main__':
    main() 