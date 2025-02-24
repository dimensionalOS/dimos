import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from go2_interfaces.msg import Go2State, IMU
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from dimos.stream.video_provider import VideoProvider
from enum import Enum, auto
import threading
import time
from typing import Optional, Tuple, Dict, Any
from abc import ABC, abstractmethod
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)
#from dimos.stream.data_provider import ROSDataProvider
from dimos.stream.ros_video_provider import ROSVideoProvider

from dimos.robot.ros_skill_library import register_skill

__all__ = ['ROSControl', 'RobotMode']

class RobotMode(Enum):
    """Enum for robot modes"""
    UNKNOWN = auto()
    IDLE = auto()
    STANDING = auto()
    MOVING = auto()
    ERROR = auto()

class ROSControl(ABC):
    """Abstract base class for ROS-controlled robots"""
    
    def __init__(self, 
                 node_name: str,
                 cmd_vel_topic: str = 'cmd_vel',
                 camera_topics: Dict[str, str] = None,
                 use_compressed_video: bool = True,
                 max_linear_velocity: float = 1.0,
                 max_angular_velocity: float = 2.0):
        """
        Initialize base ROS control interface
        Args:
            node_name: Name for the ROS node
            cmd_vel_topic: Topic for velocity commands
            camera_topics: Dictionary of camera topics
            use_compressed_video: Whether to use compressed video
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
        """
        # Initialize rclpy and ROS node if not already running
        if not rclpy.ok():
            rclpy.init()
        
        self._node = Node(node_name)
        self._logger = self._node.get_logger()
        
        # Prepare a multi-threaded executor
        self._executor = MultiThreadedExecutor()
        
        # Movement constraints
        self.MAX_LINEAR_VELOCITY = max_linear_velocity
        self.MAX_ANGULAR_VELOCITY = max_angular_velocity
        
        # State tracking
        self._mode = RobotMode.UNKNOWN
        self._is_moving = False
        self._current_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._subscriptions = []


        # Create sensor data QoS profile
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Initialize data handling
        self._video_provider = None
        self._bridge = None
        if camera_topics:
            self._bridge = CvBridge()
            self._video_provider = ROSVideoProvider(dev_name=f"{node_name}_video")
            
            # Create subscribers for each topic with sensor QoS
            msg_type = CompressedImage if use_compressed_video else Image
            for topic in camera_topics.values():
                self._logger.info(f"Subscribing to {topic} with BEST_EFFORT QoS")
                subscription = self._node.create_subscription(
                    msg_type,
                    topic,
                    self._image_callback,
                    sensor_qos
                )
                self._subscriptions.append(subscription)
        
        # Publishers
        self._cmd_vel_pub = self._node.create_publisher(
            Twist, cmd_vel_topic, 10)
            
        # Start ROS spin in a background thread via the executor
        self._spin_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._spin_thread.start()
        
        self._logger.info(f"{node_name} initialized with multi-threaded executor")
    
    def _ros_spin(self):
        """Background thread for spinning the multi-threaded executor."""
        self._executor.add_node(self._node)
        try:
            self._executor.spin()
        finally:
            self._executor.shutdown()
    
    def _clamp_velocity(self, velocity: float, max_velocity: float) -> float:
        """Clamp velocity within safe limits"""
        return max(min(velocity, max_velocity), -max_velocity)
    
    @abstractmethod
    def _update_mode(self, *args, **kwargs):
        """Update robot mode based on state - to be implemented by child classes"""
        pass
    
    def _image_callback(self, msg):
        """Convert ROS image to numpy array and push to data stream"""
        print("Running image callback")
        if self._video_provider and self._bridge:
            try:
                if isinstance(msg, CompressedImage):
                    frame = self._bridge.compressed_imgmsg_to_cv2(msg)
                    print(f"Compressed image")
                else:
                    frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                print(f"Converted frame shape: {frame.shape}")
                
                self._video_provider.push_data(frame)
                print("Successfully pushed frame to data provider")
            except Exception as e:
                self._logger.error(f"Error converting image: {e}")
                print(f"Full conversion error: {str(e)}")
    
    @property
    def video_provider(self) -> Optional[ROSVideoProvider]:
        """Data provider property for streaming data"""
        return self._video_provider
    
    #@register_skill("move_robot")
    def move(self, x: float, y: float, yaw: float, duration: float = 0.0) -> bool:
        """
        Send movement command to the robot
        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous
        Returns:
            bool: True if command was sent successfully
        """
        # Clamp velocities to safe limits
        x = self._clamp_velocity(x, self.MAX_LINEAR_VELOCITY)
        y = self._clamp_velocity(y, self.MAX_LINEAR_VELOCITY)
        yaw = self._clamp_velocity(yaw, self.MAX_ANGULAR_VELOCITY)
        
        # Create and send command
        cmd = Twist()
        cmd.linear.x = float(x)
        cmd.linear.y = float(y)
        cmd.angular.z = float(yaw)
        
        try:
            if duration > 0:
                start_time = time.time()
                while time.time() - start_time < duration:
                    self._cmd_vel_pub.publish(cmd)
                    time.sleep(0.1)  # 10Hz update rate
                # Stop after duration
                self.stop()
            else:
                self._cmd_vel_pub.publish(cmd)
            
            self._current_velocity = {"x": x, "y": y, "z": yaw}
            self._is_moving = any(abs(v) > 0.01 for v in [x, y, yaw])
            return True
            
        except Exception as e:
            self._logger.error(f"Failed to send movement command: {e}")
            return False
    
    def stop(self) -> bool:
        """
        Stop all robot movement
        Returns:
            bool: True if stop command was sent successfully
        """
        try:
            cmd = Twist()
            self._cmd_vel_pub.publish(cmd)
            self._current_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._is_moving = False
            return True
        except Exception as e:
            self._logger.error(f"Failed to send stop command: {e}")
            return False
    
    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """Get current robot state - to be implemented by child classes"""
        pass
    
    def cleanup(self):
        """Cleanup the executor, ROS node, and stop robot."""
        self.stop()

        # Shut down the executor to stop spin loop cleanly
        self._executor.shutdown()

        # Destroy node and shutdown rclpy
        self._node.destroy_node()
        rclpy.shutdown()