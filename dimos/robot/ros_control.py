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

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import Spin

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from enum import Enum, auto
import threading
import time
from typing import Optional, Tuple, Dict, Any, Type
from abc import ABC, abstractmethod
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from dimos.stream.ros_video_provider import ROSVideoProvider
import math
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3, Twist
from dimos.robot.ros_command_queue import ROSCommandQueue
from dimos.utils.logging_config import setup_logger

from nav_msgs.msg import OccupancyGrid

import tf2_ros
from dimos.robot.ros_transform import ROSTransformAbility
from dimos.robot.ros_observable_topic import ROSObservableTopicAbility

from nav_msgs.msg import Odometry

logger = setup_logger("dimos.robot.ros_control")

__all__ = ["ROSControl", "RobotMode"]


class RobotMode(Enum):
    """Enum for robot modes"""

    UNKNOWN = auto()
    INITIALIZING = auto()
    IDLE = auto()
    MOVING = auto()
    ERROR = auto()

class ROSControl(ROSTransformAbility, ROSObservableTopicAbility, ABC):
    """Abstract base class for ROS-controlled robots"""
    def __init__(self, 
                 node_name: str,
                 camera_topics: Dict[str, str] = None,
                 max_linear_velocity: float = 1.0,
                 mock_connection: bool = False,
                 max_angular_velocity: float = 2.0,
                 state_topic: str = None,
                 imu_topic: str = None,
                 state_msg_type: Type = None,
                 imu_msg_type: Type = None,
                 webrtc_topic: str = None,
                 webrtc_api_topic: str = None,
                 webrtc_msg_type: Type = None,
                 move_vel_topic: str = None,
                 pose_topic: str = None,
                 odom_topic: str = '/odom',
                 global_costmap_topic: str = "map",
                 costmap_topic: str = '/local_costmap/costmap',
                 debug: bool = False):
        """
        Initialize base ROS control interface
        Args:
            node_name: Name for the ROS node
            camera_topics: Dictionary of camera topics
            max_linear_velocity: Maximum linear velocity (m/s)
            max_angular_velocity: Maximum angular velocity (rad/s)
            state_topic: Topic name for robot state (optional)
            imu_topic: Topic name for IMU data (optional)
            state_msg_type: The ROS message type for state data
            imu_msg_type: The ROS message type for IMU data
            webrtc_topic: Topic for WebRTC commands
            webrtc_api_topic: Topic for WebRTC API commands
            webrtc_msg_type: The ROS message type for webrtc data
            move_vel_topic: Topic for direct movement commands
            pose_topic: Topic for pose commands
            odom_topic: Topic for odometry data
            costmap_topic: Topic for local costmap data
        """
        # Initialize rclpy and ROS node if not already running
        if not rclpy.ok():
            rclpy.init()

        self._state_topic = state_topic
        self._imu_topic = imu_topic
        self._odom_topic = odom_topic
        self._costmap_topic = costmap_topic
        self._state_msg_type = state_msg_type
        self._imu_msg_type = imu_msg_type
        self._webrtc_msg_type = webrtc_msg_type
        self._webrtc_topic = webrtc_topic
        self._webrtc_api_topic = webrtc_api_topic
        self._node = Node(node_name)
        self._global_costmap_topic = global_costmap_topic
        self._debug = debug

        # Prepare a multi-threaded executor
        self._executor = MultiThreadedExecutor()

        # Movement constraints
        self.MAX_LINEAR_VELOCITY = max_linear_velocity
        self.MAX_ANGULAR_VELOCITY = max_angular_velocity

        self._subscriptions = []

        # Track State variables
        self._robot_state = None  # Full state message
        self._imu_state = None  # Full IMU message
        self._odom_data = None  # Odometry data
        self._costmap_data = None  # Costmap data
        self._mode = RobotMode.INITIALIZING

        # Create sensor data QoS profile
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        command_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,  # Higher depth for commands to ensure delivery
        )

        if self._global_costmap_topic:
            self._global_costmap_data = None
            self._global_costmap_sub = self._node.create_subscription(
                OccupancyGrid,
                self._global_costmap_topic,
                self._global_costmap_callback,
                sensor_qos,
            )
            self._subscriptions.append(self._global_costmap_sub)
        else:
            logger.warning(
                "No costmap topic provided - costmap data tracking will be unavailable"
            )

        # Initialize data handling
        self._video_provider = None
        self._bridge = None
        if camera_topics:
            self._bridge = CvBridge()
            self._video_provider = ROSVideoProvider(dev_name=f"{node_name}_video")

            # Create subscribers for each topic with sensor QoS
            for camera_config in camera_topics.values():
                topic = camera_config["topic"]
                msg_type = camera_config["type"]

                logger.info(
                    f"Subscribing to {topic} with BEST_EFFORT QoS using message type {msg_type.__name__}"
                )
                _camera_subscription = self._node.create_subscription(
                    msg_type, topic, self._image_callback, sensor_qos
                )
                self._subscriptions.append(_camera_subscription)

        # Subscribe to state topic if provided
        if self._state_topic and self._state_msg_type:
            logger.info(f"Subscribing to {state_topic} with BEST_EFFORT QoS")
            self._state_sub = self._node.create_subscription(
                self._state_msg_type,
                self._state_topic,
                self._state_callback,
                qos_profile=sensor_qos,
            )
            self._subscriptions.append(self._state_sub)
        else:
            logger.warning(
                "No state topic andor message type provided - robot state tracking will be unavailable"
            )

        if self._imu_topic and self._imu_msg_type:
            self._imu_sub = self._node.create_subscription(
                self._imu_msg_type, self._imu_topic, self._imu_callback, sensor_qos
            )
            self._subscriptions.append(self._imu_sub)
        else:
            logger.warning("No IMU topic and/or message type provided - IMU data tracking will be unavailable")
            
        if self._odom_topic:
            self._odom_sub = self._node.create_subscription(
                Odometry,
                self._odom_topic,
                self._odom_callback,
                sensor_qos
            )
            self._subscriptions.append(self._odom_sub)
        else:
            logger.warning("No odometry topic provided - odometry data tracking will be unavailable")
        
        if self._costmap_topic:
            self._costmap_sub = self._node.create_subscription(
                OccupancyGrid,
                self._costmap_topic,
                self._costmap_callback,
                sensor_qos
            )
            self._subscriptions.append(self._costmap_sub)
        else:
            logger.warning("No costmap topic provided - costmap data tracking will be unavailable")

        # Nav2 Action Clients
        self._spin_client = ActionClient(self._node, Spin, "spin")

        # Wait for action servers
        if not mock_connection:
            self._spin_client.wait_for_server()

        # Publishers
        self._move_vel_pub = self._node.create_publisher(
            Twist, move_vel_topic, command_qos
        )
        self._pose_pub = self._node.create_publisher(Vector3, pose_topic, command_qos)

        if webrtc_msg_type:
            self._webrtc_pub = self._node.create_publisher(
                webrtc_msg_type, webrtc_topic, qos_profile=command_qos
            )

            # Initialize command queue
            self._command_queue = ROSCommandQueue(
                webrtc_func=self.webrtc_req,
                is_ready_func=lambda: self._mode == RobotMode.IDLE,
                is_busy_func=lambda: self._mode == RobotMode.MOVING,
            )
            # Start the queue processing thread
            self._command_queue.start()
        else:
            logger.warning("No WebRTC message type provided - WebRTC commands will be unavailable")
           
        # Initialize TF Buffer and Listener for transform abilities
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        logger.info(f"TF Buffer and Listener initialized for {node_name}")
        
        # Start ROS spin in a background thread via the executor
        self._spin_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._spin_thread.start()

        logger.info(f"{node_name} initialized with multi-threaded executor")
        print(f"{node_name} initialized with multi-threaded executor")

    def get_global_costmap(self) -> Optional[OccupancyGrid]:
        """
        Get current global_costmap data

        Returns:
            Optional[OccupancyGrid]: Current global_costmap data or None if not available
        """
        if not self._global_costmap_topic:
            logger.warning(
                "No global_costmap topic provided - global_costmap data tracking will be unavailable"
            )
            return None

        if self._global_costmap_data:
            return self._global_costmap_data
        else:
            return None

    def _global_costmap_callback(self, msg):
        """Callback for costmap data"""
        self._global_costmap_data = msg

    def _imu_callback(self, msg):
        """Callback for IMU data"""
        self._imu_state = msg
        # Log IMU state (very verbose)
        #logger.debug(f"IMU state updated: {self._imu_state}")

    def _odom_callback(self, msg):
        """Callback for odometry data"""
        self._odom_data = msg
            
    def _costmap_callback(self, msg):
        """Callback for costmap data"""
        self._costmap_data = msg

    def _state_callback(self, msg):
        """Callback for state messages to track mode and progress"""

        # Call the abstract method to update RobotMode enum based on the received state
        self._robot_state = msg
        self._update_mode(msg)
        # Log state changes (very verbose)
        # logger.debug(f"Robot state updated: {self._robot_state}")

    @property
    def robot_state(self) -> Optional[Any]:
        """Get the full robot state message"""
        return self._robot_state

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

    def get_state(self) -> Optional[Any]:
        """
        Get current robot state

        Base implementation provides common state fields. Child classes should
        extend this method to include their specific state information.

        Returns:
            ROS msg containing the robot state information
        """
        if not self._state_topic:
            logger.warning(
                "No state topic provided - robot state tracking will be unavailable"
            )
            return None

        return self._robot_state

    def get_imu_state(self) -> Optional[Any]:
        """
        Get current IMU state

        Base implementation provides common state fields. Child classes should
        extend this method to include their specific state information.

        Returns:
            ROS msg containing the IMU state information
        """
        if not self._imu_topic:
            logger.warning(
                "No IMU topic provided - IMU data tracking will be unavailable"
            )
            return None
        return self._imu_state

    def get_odometry(self) -> Optional[Odometry]:
        """
        Get current odometry data
        
        Returns:
            Optional[Odometry]: Current odometry data or None if not available
        """
        if not self._odom_topic:
            logger.warning("No odometry topic provided - odometry data tracking will be unavailable")
            return None
        return self._odom_data
        
    def get_costmap(self) -> Optional[OccupancyGrid]:
        """
        Get current costmap data
        
        Returns:
            Optional[OccupancyGrid]: Current costmap data or None if not available
        """
        if not self._costmap_topic:
            logger.warning("No costmap topic provided - costmap data tracking will be unavailable")
            return None
        return self._costmap_data
    
    def _image_callback(self, msg):
        """Convert ROS image to numpy array and push to data stream"""
        if self._video_provider and self._bridge:
            try:
                if isinstance(msg, CompressedImage):
                    frame = self._bridge.compressed_imgmsg_to_cv2(msg)
                elif isinstance(msg, Image):
                    frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                else:
                    logger.error(f"Unsupported image message type: {type(msg)}")
                    return
                self._video_provider.push_data(frame)
            except Exception as e:
                logger.error(f"Error converting image: {e}")
                print(f"Full conversion error: {str(e)}")

    @property
    def video_provider(self) -> Optional[ROSVideoProvider]:
        """Data provider property for streaming data"""
        return self._video_provider

    def _send_action_client_goal(
        self, client, goal_msg, description=None, time_allowance=20.0
    ):
        """
        Generic function to send any action client goal and wait for completion.

        Args:
            client: The action client to use
            goal_msg: The goal message to send
            description: Optional description for logging
            time_allowance: Maximum time to wait for completion

        Returns:
            bool: True if action succeeded, False otherwise
        """
        if description:
            logger.info(description)

        print(f"[ROSControl] Sending action client goal: {description}")
        print(f"[ROSControl] Goal message: {goal_msg}")

        # Reset action result tracking
        self._action_success = None

        # Send the goal
        send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=lambda feedback: None
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        # Wait for completion
        start_time = time.time()
        while (
            self._action_success is None and time.time() - start_time < time_allowance
        ):
            time.sleep(0.1)

        elapsed = time.time() - start_time
        print(
            f"[ROSControl] Action completed in {elapsed:.2f}s with result: {self._action_success}"
        )

        # Check result
        if self._action_success is None:
            logger.error(f"Action timed out after {time_allowance}s")
            return False
        elif self._action_success:
            logger.info(f"Action succeeded")
            return True
        else:
            logger.error(f"Action failed")
            return False

    def move(
        self, distance: float, speed: float = 0.5, time_allowance: float = 120
    ) -> bool:
        """
        Move the robot forward by a specified distance

        Args:
            distance: Distance to move forward in meters (must be positive)
            speed: Speed to move at in m/s (default 0.5)
            time_allowance: Maximum time to wait for the request to complete

        Returns:
            bool: True if movement succeeded
        """
        try:
            if distance <= 0:
                logger.error("Distance must be positive")
                return False

            speed = min(abs(speed), self.MAX_LINEAR_VELOCITY)

            # Define function to execute the move
            def execute_move():
                # Create DriveOnHeading goal
                goal = DriveOnHeading.Goal()
                goal.target.x = distance
                goal.target.y = 0.0
                goal.target.z = 0.0
                goal.speed = speed
                goal.time_allowance = Duration(sec=time_allowance)

                logger.info(f"Moving forward: distance={distance}m, speed={speed}m/s")

                return self._send_action_client_goal(
                    self._drive_client,
                    goal,
                    f"Sending Action Client goal in ROSControl.execute_move for {distance}m at {speed}m/s",
                    time_allowance,
                )

            # Queue the action
            cmd_id = self._command_queue.queue_action_client_request(
                action_name="move",
                execute_func=execute_move,
                priority=0,
                timeout=time_allowance,
                distance=distance,
                speed=speed,
            )
            logger.info(
                f"Queued move command: {cmd_id} - Distance: {distance}m, Speed: {speed}m/s"
            )
            return True

        except Exception as e:
            logger.error(f"Forward movement failed: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def reverse(
        self, distance: float, speed: float = 0.5, time_allowance: float = 120
    ) -> bool:
        """
        Move the robot backward by a specified distance

        Args:
            distance: Distance to move backward in meters (must be positive)
            speed: Speed to move at in m/s (default 0.5)
            time_allowance: Maximum time to wait for the request to complete

        Returns:
            bool: True if movement succeeded
        """
        try:
            if distance <= 0:
                logger.error("Distance must be positive")
                return False

            speed = min(abs(speed), self.MAX_LINEAR_VELOCITY)

            # Define function to execute the reverse
            def execute_reverse():
                # Create BackUp goal
                goal = BackUp.Goal()
                goal.target = Point()
                goal.target.x = -distance  # Negative for backward motion
                goal.target.y = 0.0
                goal.target.z = 0.0
                goal.speed = speed  # BackUp expects positive speed
                goal.time_allowance = Duration(sec=time_allowance)

                print(
                    f"[ROSControl] execute_reverse: Creating BackUp goal with distance={distance}m, speed={speed}m/s"
                )
                print(
                    f"[ROSControl] execute_reverse: Goal details: x={goal.target.x}, y={goal.target.y}, z={goal.target.z}, speed={goal.speed}"
                )

                logger.info(f"Moving backward: distance={distance}m, speed={speed}m/s")

                result = self._send_action_client_goal(
                    self._backup_client,
                    goal,
                    f"Moving backward {distance}m at {speed}m/s",
                    time_allowance,
                )

                print(f"[ROSControl] execute_reverse: BackUp action result: {result}")
                return result

            # Queue the action
            cmd_id = self._command_queue.queue_action_client_request(
                action_name="reverse",
                execute_func=execute_reverse,
                priority=0,
                timeout=time_allowance,
                distance=distance,
                speed=speed,
            )
            logger.info(
                f"Queued reverse command: {cmd_id} - Distance: {distance}m, Speed: {speed}m/s"
            )
            return True

        except Exception as e:
            logger.error(f"Backward movement failed: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def spin(
        self, degrees: float, speed: float = 45.0, time_allowance: float = 120
    ) -> bool:
        """
        Rotate the robot by a specified angle

        Args:
            degrees: Angle to rotate in degrees (positive for counter-clockwise, negative for clockwise)
            speed: Angular speed in degrees/second (default 45.0)
            time_allowance: Maximum time to wait for the request to complete

        Returns:
            bool: True if movement succeeded
        """
        try:
            # Convert degrees to radians
            angle = math.radians(degrees)
            angular_speed = math.radians(abs(speed))

            # Clamp angular speed
            angular_speed = min(angular_speed, self.MAX_ANGULAR_VELOCITY)
            time_allowance = max(
                int(abs(angle) / angular_speed * 2), 20
            )  # At least 20 seconds or double the expected time

            # Define function to execute the spin
            def execute_spin():
                # Create Spin goal
                goal = Spin.Goal()
                goal.target_yaw = angle  # Nav2 Spin action expects radians
                goal.time_allowance = Duration(sec=time_allowance)

                logger.info(f"Spinning: angle={degrees}deg ({angle:.2f}rad)")

                return self._send_action_client_goal(
                    self._spin_client,
                    goal,
                    f"Spinning {degrees} degrees at {speed} deg/s",
                    time_allowance,
                )

            # Queue the action
            cmd_id = self._command_queue.queue_action_client_request(
                action_name="spin",
                execute_func=execute_spin,
                priority=0,
                timeout=time_allowance,
                degrees=degrees,
                speed=speed,
            )
            logger.info(
                f"Queued spin command: {cmd_id} - Degrees: {degrees}, Speed: {speed}deg/s"
            )
            return True

        except Exception as e:
            logger.error(f"Spin movement failed: {e}")
            import traceback

            logger.error(traceback.format_exc())
            return False

    def _goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warn("Goal was rejected!")
            print("[ROSControl] Goal was REJECTED by the action server")
            self._action_success = False
            return

        logger.info("Goal accepted")
        print("[ROSControl] Goal was ACCEPTED by the action server")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle the goal result."""
        try:
            result = future.result().result
            logger.info("Goal completed")
            print(f"[ROSControl] Goal COMPLETED with result: {result}")
            self._action_success = True
        except Exception as e:
            logger.error(f"Goal failed with error: {e}")
            print(f"[ROSControl] Goal FAILED with error: {e}")
            self._action_success = False

    def stop(self) -> bool:
        """Stop all robot movement"""
        try:
            # self.navigator.cancelTask()
            self._current_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._is_moving = False
            return True
        except Exception as e:
            logger.error(f"Failed to stop movement: {e}")
            return False

    def cleanup(self):
        """Cleanup the executor, ROS node, and stop robot."""
        self.stop()

        # Stop the WebRTC queue manager
        if self._command_queue:
            logger.info("Stopping WebRTC queue manager...")
            self._command_queue.stop()

        # Shut down the executor to stop spin loop cleanly
        self._executor.shutdown()

        # Destroy node and shutdown rclpy
        self._node.destroy_node()
        rclpy.shutdown()

    def webrtc_req(
        self,
        api_id: int,
        topic: str = None,
        parameter: str = "",
        priority: int = 0,
        request_id: str = None,
        data=None,
    ) -> bool:
        """
        Send a WebRTC request command to the robot

        Args:
            api_id: The API ID for the command
            topic: The API topic to publish to (defaults to self._webrtc_api_topic)
            parameter: Optional parameter string
            priority: Priority level (0 or 1)
            request_id: Optional request ID for tracking (not used in ROS implementation)
            data: Optional data dictionary (not used in ROS implementation)
            params: Optional params dictionary (not used in ROS implementation)

        Returns:
            bool: True if command was sent successfully
        """
        try:
            # Create and send command
            cmd = self._webrtc_msg_type()
            cmd.api_id = api_id
            cmd.topic = topic if topic is not None else self._webrtc_api_topic
            cmd.parameter = parameter
            cmd.priority = priority

            self._webrtc_pub.publish(cmd)
            logger.info(f"Sent WebRTC request: api_id={api_id}, topic={cmd.topic}")
            return True

        except Exception as e:
            logger.error(f"Failed to send WebRTC request: {e}")
            return False

    def get_robot_mode(self) -> RobotMode:
        """
        Get the current robot mode

        Returns:
            RobotMode: The current robot mode enum value
        """
        return self._mode

    def print_robot_mode(self):
        """Print the current robot mode to the console"""
        mode = self.get_robot_mode()
        print(f"Current RobotMode: {mode.name}")
        print(f"Mode enum: {mode}")

    def queue_webrtc_req(
        self,
        api_id: int,
        topic: str = None,
        parameter: str = "",
        priority: int = 0,
        timeout: float = 90.0,
        request_id: str = None,
        data=None,
    ) -> str:
        """
        Queue a WebRTC request to be sent when the robot is IDLE

        Args:
            api_id: The API ID for the command
            topic: The topic to publish to (defaults to self._webrtc_api_topic)
            parameter: Optional parameter string
            priority: Priority level (0 or 1)
            timeout: Maximum time to wait for the request to complete
            request_id: Optional request ID (if None, one will be generated)
            data: Optional data dictionary (not used in ROS implementation)

        Returns:
            str: Request ID that can be used to track the request
        """
        return self._command_queue.queue_webrtc_request(
            api_id=api_id,
            topic=topic if topic is not None else self._webrtc_api_topic,
            parameter=parameter,
            priority=priority,
            timeout=timeout,
            request_id=request_id,
            data=data,
        )

    def move_vel(self, x: float, y: float, yaw: float, duration: float = 0.0) -> bool:
        """
        Send movement command to the robot using velocity commands

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
                    self._move_vel_pub.publish(cmd)
                    time.sleep(0.1)  # 10Hz update rate
                # Stop after duration
                self.stop()
            else:
                self._move_vel_pub.publish(cmd)
            return True

        except Exception as e:
            self._logger.error(f"Failed to send movement command: {e}")
            return False

    def move_vel_control(self, x: float, y: float, yaw: float) -> bool:
        """
        Send a single velocity command without duration handling.

        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)

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
            self._move_vel_pub.publish(cmd)
            return True
        except Exception as e:
            logger.error(f"Failed to send velocity command: {e}")
            return False

    def pose_command(self, roll: float, pitch: float, yaw: float) -> bool:
        """
        Send a pose command to the robot to adjust its body orientation

        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians

        Returns:
            bool: True if command was sent successfully
        """
        # Create the pose command message
        cmd = Vector3()
        cmd.x = float(roll)  # Roll
        cmd.y = float(pitch)  # Pitch
        cmd.z = float(yaw)  # Yaw

        try:
            self._pose_pub.publish(cmd)
            logger.debug(f"Sent pose command: roll={roll}, pitch={pitch}, yaw={yaw}")
            return True
        except Exception as e:
            logger.error(f"Failed to send pose command: {e}")
            return False
            
    def get_position_stream(self):
        """
        Get a stream of position updates from ROS.
        
        Returns:
            Observable that emits (x, y) tuples representing the robot's position
        """
        from dimos.robot.position_stream import PositionStreamProvider
        
        # Create a position stream provider
        position_provider = PositionStreamProvider(
            ros_node=self._node,
            odometry_topic="/odom",  # Default odometry topic
            use_odometry=True
        )
        
        return position_provider.get_position_stream()
