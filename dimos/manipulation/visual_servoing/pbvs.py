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
Position-Based Visual Servoing (PBVS) system for robotic manipulation.
Supports both eye-in-hand and eye-to-hand configurations.
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any, List
import cv2

from scipy.spatial.transform import Rotation as R
from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion
from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    yaw_towards_point,
    euler_to_quaternion,
)

logger = setup_logger("dimos.manipulation.pbvs")


class PBVS:
    """
    High-level Position-Based Visual Servoing orchestrator.

    Handles:
    - Object tracking and target management
    - Pregrasp distance computation
    - Grasp pose generation
    - Coordination with low-level controller

    Note: This class is agnostic to camera mounting (eye-in-hand vs eye-to-hand).
    The caller is responsible for providing appropriate camera and EE poses.
    """

    def __init__(
        self,
        position_gain: float = 0.5,
        rotation_gain: float = 0.3,
        max_velocity: float = 0.1,  # m/s
        max_angular_velocity: float = 0.5,  # rad/s
        target_tolerance: float = 0.01,  # 1cm
        tracking_distance_threshold: float = 0.05,  # 5cm for target tracking
        pregrasp_distance: float = 0.15,  # 15cm pregrasp distance
        direct_ee_control: bool = False,  # If True, output target poses instead of velocities
    ):
        """
        Initialize PBVS system.

        Args:
            position_gain: Proportional gain for position control
            rotation_gain: Proportional gain for rotation control
            max_velocity: Maximum linear velocity command magnitude (m/s)
            max_angular_velocity: Maximum angular velocity command magnitude (rad/s)
            target_tolerance: Distance threshold for considering target reached (m)
            tracking_distance_threshold: Max distance for target association (m)
            pregrasp_distance: Distance to maintain before grasping (m)
            direct_ee_control: If True, output target poses instead of velocity commands
        """
        # Initialize low-level controller only if not in direct control mode
        if not direct_ee_control:
            self.controller = PBVSController(
                position_gain=position_gain,
                rotation_gain=rotation_gain,
                max_velocity=max_velocity,
                max_angular_velocity=max_angular_velocity,
                target_tolerance=target_tolerance,
            )
        else:
            self.controller = None

        # Store parameters for direct mode error computation
        self.target_tolerance = target_tolerance

        # Target tracking parameters
        self.tracking_distance_threshold = tracking_distance_threshold
        self.pregrasp_distance = pregrasp_distance
        self.direct_ee_control = direct_ee_control

        # Target state
        self.current_target = None
        self.target_grasp_pose = None

        # For direct control mode visualization
        self.last_position_error = None
        self.last_target_reached = False

        logger.info(
            f"Initialized PBVS system with controller gains: pos={position_gain}, rot={rotation_gain}, "
            f"pregrasp_distance={pregrasp_distance}m"
        )

    def set_target(self, target_object: Dict[str, Any]) -> bool:
        """
        Set a new target object for servoing.

        Args:
            target_object: Object dict with at least 'position' field

        Returns:
            True if target was set successfully
        """
        if target_object and "position" in target_object:
            self.current_target = target_object
            self.target_grasp_pose = None  # Will be computed when needed
            logger.info(f"New target set: ID {target_object.get('object_id', 'unknown')}")
            return True
        return False

    def clear_target(self):
        """Clear the current target."""
        self.current_target = None
        self.target_grasp_pose = None
        self.last_position_error = None
        self.last_target_reached = False
        if self.controller:
            self.controller.clear_state()
        logger.info("Target cleared")

    def get_current_target(self):
        """
        Get the current target object.

        Returns:
            Current target ObjectData or None if no target selected
        """
        return self.current_target

    def is_target_reached(self, ee_pose: Pose) -> bool:
        """
        Check if the current target has been reached.

        Args:
            ee_pose: Current end-effector pose

        Returns:
            True if target is reached, False otherwise
        """
        if not self.target_grasp_pose:
            return False

        # Calculate position error
        error_x = self.target_grasp_pose.position.x - ee_pose.position.x
        error_y = self.target_grasp_pose.position.y - ee_pose.position.y
        error_z = self.target_grasp_pose.position.z - ee_pose.position.z

        error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        return error_magnitude < self.target_tolerance

    def update_target_tracking(self, new_detections: List[ObjectData]) -> bool:
        """
        Update target by matching to closest object in new detections.
        If tracking is lost, keeps the old target pose.

        Args:
            new_detections: List of newly detected objects

        Returns:
            True if target was successfully tracked, False if lost (but target is kept)
        """
        if not self.current_target or "position" not in self.current_target:
            return False

        if not new_detections:
            logger.debug("No detections for target tracking - using last known pose")
            return False

        # Get current target position
        target_pos = self.current_target["position"]
        if isinstance(target_pos, Vector3):
            target_xyz = np.array([target_pos.x, target_pos.y, target_pos.z])
        else:
            target_xyz = np.array([target_pos["x"], target_pos["y"], target_pos["z"]])

        best_match = None
        min_distance = float("inf")

        for detection in new_detections:
            if "position" not in detection:
                continue

            det_pos = detection["position"]
            if isinstance(det_pos, Vector3):
                det_xyz = np.array([det_pos.x, det_pos.y, det_pos.z])
            else:
                det_xyz = np.array([det_pos["x"], det_pos["y"], det_pos["z"]])

            distance = np.linalg.norm(target_xyz - det_xyz)

            if distance < self.tracking_distance_threshold:
                best_match = detection

            if distance < min_distance:
                min_distance = distance

        if best_match:
            self.current_target = best_match
            self.target_grasp_pose = None  # Recompute grasp pose
            return True
        logger.info(f"Target tracking lost: closest target distance={min_distance:.3f}m")
        return False

    def _update_target_grasp_pose(self, ee_pose: Pose):
        """
        Update target grasp pose based on current target and EE pose.

        Args:
            ee_pose: Current end-effector pose
        """
        if not self.current_target or "position" not in self.current_target:
            return

        # Get target position
        target_pos = self.current_target["position"]

        # Calculate orientation pointing from target towards EE
        yaw_to_ee = yaw_towards_point(
            Vector3(target_pos.x, target_pos.y, target_pos.z), ee_pose.position
        )

        # Create target pose with proper orientation
        # Convert euler angles to quaternion using utility function
        euler = Vector3(0.0, 1.65, yaw_to_ee)  # roll=0, pitch=90deg, yaw=calculated
        target_orientation = euler_to_quaternion(euler)

        target_pose = Pose(target_pos, target_orientation)

        # Apply pregrasp distance
        self.target_grasp_pose = self._apply_pregrasp_distance(target_pose, ee_pose)

    def _apply_pregrasp_distance(self, target_pose: Pose, ee_pose: Pose) -> Pose:
        """
        Apply pregrasp distance to target pose by moving back towards EE.

        Args:
            target_pose: Target pose
            ee_pose: Current end-effector pose

        Returns:
            Modified target pose with pregrasp distance applied
        """
        # Get approach vector (from target position towards EE)
        target_pos = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        )
        ee_pos = np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z])
        approach_vector = ee_pos - target_pos  # Vector pointing towards EE

        # Normalize approach vector
        approach_magnitude = np.linalg.norm(approach_vector)
        if approach_magnitude > 1e-6:  # Avoid division by zero
            norm_approach_vector = approach_vector / approach_magnitude
        else:
            norm_approach_vector = np.array([0.0, 0.0, 0.0])

        # Move back by pregrasp distance towards EE
        offset_vector = self.pregrasp_distance * norm_approach_vector

        # Apply offset to target position
        new_position = Vector3(
            target_pose.position.x + offset_vector[0],
            target_pose.position.y + offset_vector[1],
            target_pose.position.z + offset_vector[2],
        )

        return Pose(new_position, target_pose.orientation)

    def compute_control(
        self, ee_pose: Pose, new_detections: Optional[List[ObjectData]] = None
    ) -> Tuple[Optional[Vector3], Optional[Vector3], bool, bool, Optional[Pose]]:
        """
        Compute PBVS control with position and orientation servoing.

        Args:
            ee_pose: Current end-effector pose
            new_detections: Optional new detections for target tracking

        Returns:
            Tuple of (velocity_command, angular_velocity_command, target_reached, has_target, target_pose)
            - velocity_command: Linear velocity vector or None if no target (None in direct_ee_control mode)
            - angular_velocity_command: Angular velocity vector or None if no target (None in direct_ee_control mode)
            - target_reached: True if within target tolerance
            - has_target: True if currently tracking a target
            - target_pose: Target EE pose (only in direct_ee_control mode, otherwise None)
        """
        # Check if we have a target
        if not self.current_target or "position" not in self.current_target:
            return None, None, False, False, None

        # Try to update target tracking if new detections provided
        # Continue with last known pose even if tracking is lost
        target_tracked = False
        if new_detections is not None:
            if self.update_target_tracking(new_detections):
                target_tracked = True
            else:
                target_tracked = False

        # Update target grasp pose
        self._update_target_grasp_pose(ee_pose)

        if self.target_grasp_pose is None:
            logger.warning("Failed to compute grasp pose")
            return None, None, False, False, None

        # Check if target reached using our separate function
        target_reached = self.is_target_reached(ee_pose)

        # Return appropriate values based on control mode
        if self.direct_ee_control:
            # Direct control mode - compute errors for visualization only
            self.last_position_error = Vector3(
                self.target_grasp_pose.position.x - ee_pose.position.x,
                self.target_grasp_pose.position.y - ee_pose.position.y,
                self.target_grasp_pose.position.z - ee_pose.position.z,
            )
            self.last_target_reached = target_reached
            return None, None, target_reached, target_tracked, self.target_grasp_pose
        else:
            # Velocity control mode - use controller
            velocity_cmd, angular_velocity_cmd, controller_reached = (
                self.controller.compute_control(ee_pose, self.target_grasp_pose)
            )
            return velocity_cmd, angular_velocity_cmd, target_reached, target_tracked, None

    def get_object_pose_camera_frame(
        self, object_pos: Vector3, camera_pose: Pose
    ) -> Tuple[Vector3, Quaternion]:
        """
        Get object pose in camera frame coordinates with orientation.

        Args:
            object_pos: Object position in camera frame
            camera_pose: Current camera pose

        Returns:
            Tuple of (position, rotation) in camera frame
        """
        # Calculate orientation pointing at camera
        yaw_to_camera = yaw_towards_point(Vector3(object_pos.x, object_pos.y, object_pos.z))

        # Convert euler angles to quaternion using utility function
        euler = Vector3(0.0, 0.0, yaw_to_camera)  # Level grasp
        orientation = euler_to_quaternion(euler)

        return object_pos, orientation

    def create_status_overlay(
        self,
        image: np.ndarray,
    ) -> np.ndarray:
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            camera_intrinsics: Optional [fx, fy, cx, cy] (not used)

        Returns:
            Image with PBVS status overlay
        """
        if self.direct_ee_control:
            # Use our own error data for direct control mode
            return self._create_direct_status_overlay(image, self.current_target)
        else:
            # Use controller's overlay for velocity mode
            return self.controller.create_status_overlay(
                image,
                self.current_target,
                self.direct_ee_control,
            )

    def _create_direct_status_overlay(
        self, image: np.ndarray, current_target: Optional[ObjectData] = None
    ) -> np.ndarray:
        """
        Create status overlay for direct control mode.

        Args:
            image: Input image
            current_target: Current target object

        Returns:
            Image with status overlay
        """
        viz_img = image.copy()
        height, width = image.shape[:2]

        # Status panel
        if current_target is not None:
            panel_height = 160  # Adjusted panel for target, grasp pose, and pregrasp distance info
            panel_y = height - panel_height
            overlay = viz_img.copy()
            cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
            viz_img = cv2.addWeighted(viz_img, 0.7, overlay, 0.3, 0)

            # Status text
            y = panel_y + 20
            cv2.putText(
                viz_img,
                "PBVS Status (Direct EE)",
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            # Add frame info
            cv2.putText(
                viz_img, "Frame: Camera", (250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
            )

            if self.last_position_error:
                error_mag = np.linalg.norm(
                    [
                        self.last_position_error.x,
                        self.last_position_error.y,
                        self.last_position_error.z,
                    ]
                )
                color = (0, 255, 0) if self.last_target_reached else (0, 255, 255)

                cv2.putText(
                    viz_img,
                    f"Pos Error: {error_mag:.3f}m ({error_mag * 100:.1f}cm)",
                    (10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )

                cv2.putText(
                    viz_img,
                    f"XYZ: ({self.last_position_error.x:.3f}, {self.last_position_error.y:.3f}, {self.last_position_error.z:.3f})",
                    (10, y + 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (200, 200, 200),
                    1,
                )

            # Show target and grasp poses
            if current_target and "position" in current_target:
                target_pos = current_target["position"]
                cv2.putText(
                    viz_img,
                    f"Target: ({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f})",
                    (10, y + 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 0),
                    1,
                )

            if self.target_grasp_pose:
                grasp_pos = self.target_grasp_pose.position
                cv2.putText(
                    viz_img,
                    f"Grasp:  ({grasp_pos.x:.3f}, {grasp_pos.y:.3f}, {grasp_pos.z:.3f})",
                    (10, y + 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 255),
                    1,
                )

                # Show pregrasp distance if we have both poses
                if current_target and "position" in current_target:
                    target_pos = current_target["position"]
                    distance = np.sqrt(
                        (grasp_pos.x - target_pos.x) ** 2
                        + (grasp_pos.y - target_pos.y) ** 2
                        + (grasp_pos.z - target_pos.z) ** 2
                    )
                    cv2.putText(
                        viz_img,
                        f"Pregrasp: {distance * 1000:.1f}mm",
                        (10, y + 95),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 200, 0),
                        1,
                    )

            if self.last_target_reached:
                cv2.putText(
                    viz_img,
                    "TARGET REACHED",
                    (width - 150, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        return viz_img


class PBVSController:
    """
    Low-level Position-Based Visual Servoing controller.
    Pure control logic that computes velocity commands from poses.

    Handles:
    - Position and orientation error computation
    - Velocity command generation with gain control
    - Target reached detection
    """

    def __init__(
        self,
        position_gain: float = 0.5,
        rotation_gain: float = 0.3,
        max_velocity: float = 0.1,  # m/s
        max_angular_velocity: float = 0.5,  # rad/s
        target_tolerance: float = 0.01,  # 1cm
    ):
        """
        Initialize PBVS controller.

        Args:
            position_gain: Proportional gain for position control
            rotation_gain: Proportional gain for rotation control
            max_velocity: Maximum linear velocity command magnitude (m/s)
            max_angular_velocity: Maximum angular velocity command magnitude (rad/s)
            target_tolerance: Distance threshold for considering target reached (m)
        """
        self.position_gain = position_gain
        self.rotation_gain = rotation_gain
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.target_tolerance = target_tolerance

        # State variables for visualization
        self.last_position_error = None
        self.last_rotation_error = None
        self.last_velocity_cmd = None
        self.last_angular_velocity_cmd = None
        self.last_target_reached = False

        logger.info(
            f"Initialized PBVS controller: pos_gain={position_gain}, rot_gain={rotation_gain}, "
            f"max_vel={max_velocity}m/s, max_ang_vel={max_angular_velocity}rad/s, "
            f"target_tolerance={target_tolerance}m"
        )

    def clear_state(self):
        """Clear controller state."""
        self.last_position_error = None
        self.last_rotation_error = None
        self.last_velocity_cmd = None
        self.last_angular_velocity_cmd = None
        self.last_target_reached = False

    def compute_control(
        self, ee_pose: Pose, grasp_pose: Pose
    ) -> Tuple[Optional[Vector3], Optional[Vector3], bool]:
        """
        Compute PBVS control with position and orientation servoing.

        Args:
            ee_pose: Current end-effector pose
            grasp_pose: Target grasp pose

        Returns:
            Tuple of (velocity_command, angular_velocity_command, target_reached)
            - velocity_command: Linear velocity vector
            - angular_velocity_command: Angular velocity vector
            - target_reached: True if within target tolerance
        """
        # Calculate position error (target - EE position)
        error = Vector3(
            grasp_pose.position.x - ee_pose.position.x,
            grasp_pose.position.y - ee_pose.position.y,
            grasp_pose.position.z - ee_pose.position.z,
        )
        self.last_position_error = error

        # Compute velocity command with proportional control
        velocity_cmd = Vector3(
            error.x * self.position_gain,
            error.y * self.position_gain,
            error.z * self.position_gain,
        )

        # Limit velocity magnitude
        vel_magnitude = np.linalg.norm([velocity_cmd.x, velocity_cmd.y, velocity_cmd.z])
        if vel_magnitude > self.max_velocity:
            scale = self.max_velocity / vel_magnitude
            velocity_cmd = Vector3(
                float(velocity_cmd.x * scale),
                float(velocity_cmd.y * scale),
                float(velocity_cmd.z * scale),
            )

        self.last_velocity_cmd = velocity_cmd

        # Compute angular velocity for orientation control
        angular_velocity_cmd = self._compute_angular_velocity(grasp_pose.orientation, ee_pose)

        # Check if target reached
        error_magnitude = np.linalg.norm([error.x, error.y, error.z])
        target_reached = bool(error_magnitude < self.target_tolerance)
        self.last_target_reached = target_reached

        return velocity_cmd, angular_velocity_cmd, target_reached

    def _compute_angular_velocity(self, target_rot: Quaternion, current_pose: Pose) -> Vector3:
        """
        Compute angular velocity commands for orientation control.
        Uses quaternion error computation for better numerical stability.

        Args:
            target_rot: Target orientation (quaternion)
            current_pose: Current EE pose

        Returns:
            Angular velocity command as Vector3
        """
        # Use quaternion error for better numerical stability

        # Convert to scipy Rotation objects
        target_rot_scipy = R.from_quat([target_rot.x, target_rot.y, target_rot.z, target_rot.w])
        current_rot_scipy = R.from_quat(
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
        )

        # Compute rotation error: error = target * current^(-1)
        error_rot = target_rot_scipy * current_rot_scipy.inv()

        # Convert to axis-angle representation for control
        error_axis_angle = error_rot.as_rotvec()

        # Use axis-angle directly as angular velocity error (small angle approximation)
        roll_error = error_axis_angle[0]
        pitch_error = error_axis_angle[1]
        yaw_error = error_axis_angle[2]

        self.last_rotation_error = Vector3(roll_error, pitch_error, yaw_error)

        # Apply proportional control
        angular_velocity = Vector3(
            roll_error * self.rotation_gain,
            pitch_error * self.rotation_gain,
            yaw_error * self.rotation_gain,
        )

        # Limit angular velocity magnitude
        ang_vel_magnitude = np.sqrt(
            angular_velocity.x**2 + angular_velocity.y**2 + angular_velocity.z**2
        )
        if ang_vel_magnitude > self.max_angular_velocity:
            scale = self.max_angular_velocity / ang_vel_magnitude
            angular_velocity = Vector3(
                angular_velocity.x * scale, angular_velocity.y * scale, angular_velocity.z * scale
            )

        self.last_angular_velocity_cmd = angular_velocity

        return angular_velocity

    def create_status_overlay(
        self,
        image: np.ndarray,
        current_target: Optional[Dict[str, Any]] = None,
        direct_ee_control: bool = False,
    ) -> np.ndarray:
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            current_target: Current target object (for display)
            direct_ee_control: Whether in direct EE control mode

        Returns:
            Image with PBVS status overlay
        """
        viz_img = image.copy()
        height, width = image.shape[:2]

        # Status panel
        if current_target is not None:
            panel_height = 160  # Adjusted panel height
            panel_y = height - panel_height
            overlay = viz_img.copy()
            cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
            viz_img = cv2.addWeighted(viz_img, 0.7, overlay, 0.3, 0)

            # Status text
            y = panel_y + 20
            mode_text = "Direct EE" if direct_ee_control else "Velocity"
            cv2.putText(
                viz_img,
                f"PBVS Status ({mode_text})",
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            # Add frame info
            cv2.putText(
                viz_img, "Frame: Camera", (250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
            )

            if self.last_position_error:
                error_mag = np.linalg.norm(
                    [
                        self.last_position_error.x,
                        self.last_position_error.y,
                        self.last_position_error.z,
                    ]
                )
                color = (0, 255, 0) if self.last_target_reached else (0, 255, 255)

                cv2.putText(
                    viz_img,
                    f"Pos Error: {error_mag:.3f}m ({error_mag * 100:.1f}cm)",
                    (10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )

                cv2.putText(
                    viz_img,
                    f"XYZ: ({self.last_position_error.x:.3f}, {self.last_position_error.y:.3f}, {self.last_position_error.z:.3f})",
                    (10, y + 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (200, 200, 200),
                    1,
                )

            if self.last_velocity_cmd and not direct_ee_control:
                cv2.putText(
                    viz_img,
                    f"Lin Vel: ({self.last_velocity_cmd.x:.2f}, {self.last_velocity_cmd.y:.2f}, {self.last_velocity_cmd.z:.2f})m/s",
                    (10, y + 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 0),
                    1,
                )

            if self.last_rotation_error:
                cv2.putText(
                    viz_img,
                    f"Rot Error: ({self.last_rotation_error.x:.2f}, {self.last_rotation_error.y:.2f}, {self.last_rotation_error.z:.2f})rad",
                    (10, y + 85),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (200, 200, 200),
                    1,
                )

            if self.last_angular_velocity_cmd and not direct_ee_control:
                cv2.putText(
                    viz_img,
                    f"Ang Vel: ({self.last_angular_velocity_cmd.x:.2f}, {self.last_angular_velocity_cmd.y:.2f}, {self.last_angular_velocity_cmd.z:.2f})rad/s",
                    (10, y + 105),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 0),
                    1,
                )

            if self.last_target_reached:
                cv2.putText(
                    viz_img,
                    "TARGET REACHED",
                    (width - 150, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        return viz_img
