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

"""
Visual servoing module for Unitree Go2 robot base control.
Handles visual tracking and base positioning for object manipulation.
"""

import time
import threading
from typing import Optional, Dict, Any
from enum import Enum

import numpy as np

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray
from dimos_lcm.std_msgs import String

from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.perception.common.utils import find_clicked_detection
from dimos.manipulation.visual_servoing.utils import (
    create_manipulation_visualization,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.visual_servoing.servo_base")


class ServoBaseState(Enum):
    """Enum for different servo base states."""

    IDLE = "idle"
    SERVOING = "servoing"
    SET_PICK_POSE = "set_pick_pose"
    WAITING_FOR_PICK = "waiting_for_pick"


class ServoBaseModule(Module):
    """
    Visual servoing module for Unitree Go2 base control.

    Subscribes to:
        - RGB images
        - Detection3DArray
        - Detection2DArray
        - Grasp state from manipulation module

    Publishes:
        - Velocity commands for base control
        - Servo state
        - Visualization images
    """

    # LCM inputs
    rgb_image: In[Image] = None
    detection3d_array: In[Detection3DArray] = None
    detection2d_array: In[Detection2DArray] = None
    camera_info: In[CameraInfo] = None
    grasp_state: In[String] = None  # Subscribe to manipulation module's state

    # LCM outputs
    cmd_vel: Out[Twist] = None  # Velocity commands for robot base
    servo_state: Out[String] = None  # Current servo state
    viz_image: Out[Image] = None  # Visualization output

    def __init__(
        self,
        servoing_distance: float = 0.5,  # Target distance from object
        servoing_tolerance: float = 0.05,  # Position tolerance
        max_linear_vel: float = 0.3,  # Max linear velocity (m/s)
        max_angular_vel: float = 0.5,  # Max angular velocity (rad/s)
        set_pick_pose_duration: float = 5.0,  # Duration for SET_PICK_POSE state
        **kwargs,
    ):
        """
        Initialize servo base module.

        Args:
            servoing_distance: Target distance to maintain from object (meters)
            servoing_tolerance: Position tolerance for considering target reached
            max_linear_vel: Maximum linear velocity for base
            max_angular_vel: Maximum angular velocity for base
            set_pick_pose_duration: Time to spend in SET_PICK_POSE state
        """
        super().__init__(**kwargs)

        # Parameters
        self.servoing_distance = servoing_distance
        self.servoing_tolerance = servoing_tolerance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.set_pick_pose_duration = set_pick_pose_duration

        # State machine
        self.state = ServoBaseState.IDLE
        self.state_lock = threading.Lock()

        # PBVS controller
        self.pbvs = PBVS()
        self.camera_intrinsics = None

        # Tracking state
        self.current_target = None
        self.target_click = None
        self.last_valid_target = None

        # Latest sensor data
        self.latest_rgb = None
        self.latest_detection3d = None
        self.latest_detection2d = None
        self.latest_grasp_state = None

        # Control thread
        self.control_thread = None
        self.stop_event = threading.Event()
        self.control_rate = 30.0  # Hz
        self.control_period = 1.0 / self.control_rate

        # State timing
        self.set_pick_pose_start_time = None

        logger.info(
            f"ServoBaseModule initialized with distance={servoing_distance}m, "
            f"tolerance={servoing_tolerance}m"
        )

    @rpc
    def start(self):
        """Start the servo base module."""
        # Subscribe to inputs
        self.rgb_image.subscribe(self._on_rgb_image)
        self.detection3d_array.subscribe(self._on_detection3d_array)
        self.detection2d_array.subscribe(self._on_detection2d_array)
        self.camera_info.subscribe(self._on_camera_info)

        if self.grasp_state:
            self.grasp_state.subscribe(self._on_grasp_state)

        # Start control thread
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        logger.info("ServoBaseModule started")

    @rpc
    def stop(self):
        """Stop the servo base module."""
        # Stop control thread
        self.stop_event.set()
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)

        # Stop robot
        self._stop_robot()

        # Reset state
        self.reset_to_idle()

        logger.info("ServoBaseModule stopped")

    def _on_rgb_image(self, msg: Image):
        """Handle RGB image messages."""
        self.latest_rgb = msg.data

    def _on_detection3d_array(self, msg: Detection3DArray):
        """Handle 3D detection messages."""
        self.latest_detection3d = msg

    def _on_detection2d_array(self, msg: Detection2DArray):
        """Handle 2D detection messages."""
        self.latest_detection2d = msg

    def _on_camera_info(self, msg: CameraInfo):
        """Handle camera info messages."""
        self.camera_intrinsics = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]
        logger.info(f"Camera intrinsics updated: {self.camera_intrinsics}")

    def _on_grasp_state(self, msg: String):
        """Handle grasp state messages from manipulation module."""
        self.latest_grasp_state = msg.data

        # Check if grasp has returned from RETRACT to IDLE
        if self.state == ServoBaseState.WAITING_FOR_PICK:
            if msg.data == "idle":  # GraspStage.IDLE
                logger.info("Grasp completed, returning to IDLE")
                self.set_state(ServoBaseState.IDLE)

    def set_state(self, state: ServoBaseState):
        """Set the servo base state."""
        with self.state_lock:
            self.state = state
            logger.info(f"Servo base state: {state.value}")

            # Publish state change
            if self.servo_state:
                self.servo_state.publish(String(data=state.value))

    def reset_to_idle(self):
        """Reset the system to IDLE state."""
        self.set_state(ServoBaseState.IDLE)
        self.current_target = None
        self.last_valid_target = None
        self.set_pick_pose_start_time = None
        self._stop_robot()

    def _stop_robot(self):
        """Send zero velocity command to stop the robot."""
        if self.cmd_vel:
            stop_cmd = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            self.cmd_vel.publish(stop_cmd)

    @rpc
    def select_target(self, x: int, y: int) -> bool:
        """
        Select a target object at the given pixel coordinates.

        Args:
            x: X coordinate in image
            y: Y coordinate in image

        Returns:
            True if target was selected, False otherwise
        """
        self.target_click = (x, y)
        return True

    def _pick_target(self, x: int, y: int) -> bool:
        """Select a target object for tracking."""
        if not self.latest_detection2d or not self.latest_detection3d:
            logger.warning("No detections available for target selection")
            return False

        clicked_3d = find_clicked_detection(
            (x, y), self.latest_detection2d.detections, self.latest_detection3d.detections
        )

        if clicked_3d:
            self.pbvs.set_target(clicked_3d)
            self.current_target = clicked_3d
            self.last_valid_target = clicked_3d

            position = clicked_3d.bbox.center.position
            logger.info(
                f"Target selected: ID={clicked_3d.id}, "
                f"pos=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )

            # Start servoing
            self.set_state(ServoBaseState.SERVOING)
            return True

        return False

    def _compute_base_velocity(self) -> Optional[Twist]:
        """
        Compute velocity commands for base control using PBVS.

        Returns:
            Twist message with velocity commands or None
        """
        if not self.current_target:
            return None

        # Update tracking with latest detections
        if self.latest_detection3d and self.pbvs:
            target_tracked = self.pbvs.update_tracking(self.latest_detection3d)
            if target_tracked:
                self.current_target = self.pbvs.get_current_target()
                self.last_valid_target = self.current_target

        # Use last valid target for control
        if not self.last_valid_target:
            return None

        # Get target position in camera frame
        target_pos = self.last_valid_target.bbox.center.position

        # Calculate errors
        x_error = target_pos.x  # Lateral error
        z_error = target_pos.z - self.servoing_distance  # Distance error

        # Simple proportional control
        kp_linear = 0.5
        kp_angular = 1.0

        # Calculate velocities
        linear_vel = np.clip(kp_linear * z_error, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(-kp_angular * x_error, -self.max_angular_vel, self.max_angular_vel)

        # Check if target is reached
        if abs(x_error) < self.servoing_tolerance and abs(z_error) < self.servoing_tolerance:
            logger.info("Target position reached")
            return Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))

        # Create velocity command
        cmd = Twist(
            linear=Vector3(linear_vel, 0.0, 0.0),  # Forward/backward
            angular=Vector3(0.0, 0.0, angular_vel),  # Rotation
        )

        return cmd

    def _execute_idle(self):
        """Execute IDLE state."""
        # Check for target selection
        if self.target_click:
            x, y = self.target_click
            if self._pick_target(x, y):
                self.target_click = None

    def _execute_servoing(self):
        """Execute SERVOING state."""
        # Compute and send velocity commands
        cmd = self._compute_base_velocity()

        if cmd:
            self.cmd_vel.publish(cmd)

            # Check if target is reached (zero velocity)
            if cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
                logger.info("Target reached, transitioning to SET_PICK_POSE")
                self.set_state(ServoBaseState.SET_PICK_POSE)
                self.set_pick_pose_start_time = time.time()
        else:
            # Lost target, stop and return to idle
            logger.warning("Lost target, returning to IDLE")
            self._stop_robot()
            self.reset_to_idle()

    def _execute_set_pick_pose(self):
        """Execute SET_PICK_POSE state - position robot for picking."""
        # For now, just wait for the specified duration
        # In the future, this would adjust robot pose (tilt, height, etc.)

        if self.set_pick_pose_start_time:
            elapsed = time.time() - self.set_pick_pose_start_time

            if elapsed >= self.set_pick_pose_duration:
                logger.info("Pick pose set, transitioning to WAITING_FOR_PICK")
                self.set_state(ServoBaseState.WAITING_FOR_PICK)
                self.set_pick_pose_start_time = None

    def _execute_waiting_for_pick(self):
        """Execute WAITING_FOR_PICK state - wait for manipulation to complete."""
        # State change is handled by _on_grasp_state callback
        pass

    def _control_loop(self):
        """Main control loop running in separate thread."""
        while not self.stop_event.is_set():
            with self.state_lock:
                current_state = self.state

            # Execute state-specific behavior
            state_handlers = {
                ServoBaseState.IDLE: self._execute_idle,
                ServoBaseState.SERVOING: self._execute_servoing,
                ServoBaseState.SET_PICK_POSE: self._execute_set_pick_pose,
                ServoBaseState.WAITING_FOR_PICK: self._execute_waiting_for_pick,
            }

            if current_state in state_handlers:
                try:
                    state_handlers[current_state]()
                except Exception as e:
                    logger.error(f"Error in state {current_state.value}: {e}")
                    self.reset_to_idle()

            # Publish visualization if enabled
            if self.latest_rgb is not None and self.viz_image:
                self._publish_visualization()

            time.sleep(self.control_period)

    def _publish_visualization(self):
        """Create and publish visualization image."""
        try:
            # Create simple feedback for visualization
            feedback = {
                "state": self.state.value,
                "target_tracked": self.current_target is not None,
                "target_id": self.current_target.id if self.current_target else None,
            }

            # Create visualization
            viz = create_manipulation_visualization(
                self.latest_rgb, feedback, self.latest_detection3d, self.latest_detection2d
            )

            if viz is not None:
                import cv2

                viz_rgb = cv2.cvtColor(viz, cv2.COLOR_BGR2RGB)
                viz_msg = Image.from_numpy(viz_rgb)
                self.viz_image.publish(viz_msg)
        except Exception as e:
            logger.error(f"Error creating visualization: {e}")

    @rpc
    def get_state(self) -> str:
        """Get the current servo base state."""
        with self.state_lock:
            return self.state.value

    @rpc
    def cleanup(self):
        """Clean up resources."""
        self.stop()
