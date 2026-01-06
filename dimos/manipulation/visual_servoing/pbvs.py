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

from collections import deque

from dimos_lcm.vision_msgs import Detection3D  # type: ignore[import-untyped]
import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore[import-untyped]

from dimos.manipulation.visual_servoing.utils import (
    create_pbvs_visualization,
    find_best_object_match,
    is_target_reached,
    update_target_grasp_pose,
)
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.vision_msgs import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
        max_tracking_distance_threshold: float = 0.12,  # Max distance for target tracking (m)
        min_size_similarity: float = 0.6,  # Min size similarity threshold (0.0-1.0)
        direct_ee_control: bool = True,  # If True, output target poses instead of velocities
    ) -> None:
        """
        Initialize PBVS system.

        Args:
            position_gain: Proportional gain for position control
            rotation_gain: Proportional gain for rotation control
            max_velocity: Maximum linear velocity command magnitude (m/s)
            max_angular_velocity: Maximum angular velocity command magnitude (rad/s)
            target_tolerance: Distance threshold for considering target reached (m)
            max_tracking_distance: Maximum distance for valid target tracking (m)
            min_size_similarity: Minimum size similarity for valid target tracking (0.0-1.0)
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
            self.controller = None  # type: ignore[assignment]

        # Store parameters for direct mode error computation
        self.target_tolerance = target_tolerance

        # Target tracking parameters
        self.max_tracking_distance_threshold = max_tracking_distance_threshold
        self.min_size_similarity = min_size_similarity
        self.direct_ee_control = direct_ee_control

        # Target state
        self.current_target = None
        self.target_grasp_pose = None

        # Detection history for robust tracking
        self.detection_history_size = 3
        self.detection_history = deque(maxlen=self.detection_history_size)  # type: ignore[var-annotated]

        # For direct control mode visualization
        self.last_position_error = None
        self.last_target_reached = False

        logger.info(
            f"Initialized PBVS system with controller gains: pos={position_gain}, rot={rotation_gain}, "
            f"tracking_thresholds: distance={max_tracking_distance_threshold}m, size={min_size_similarity:.2f}"
        )

    def set_target(self, target_object: Detection3D) -> bool:
        """
        Set a new target object for servoing.

        Args:
            target_object: Detection3D object

        Returns:
            True if target was set successfully
        """
        if target_object and target_object.bbox and target_object.bbox.center:
            self.current_target = target_object
            self.target_grasp_pose = None  # Will be computed when needed
            logger.info(f"New target set: ID {target_object.id}")
            return True
        return False

    def clear_target(self) -> None:
        """Clear the current target."""
        self.current_target = None
        self.target_grasp_pose = None
        self.last_position_error = None
        self.last_target_reached = False
        self.detection_history.clear()
        if self.controller:
            self.controller.clear_state()
        logger.info("Target cleared")

    def get_current_target(self) -> Detection3D | None:
        """
        Get the current target object.

        Returns:
            Current target Detection3D or None if no target selected
        """
        return self.current_target

    def update_tracking(self, new_detections: Detection3DArray | None = None) -> bool:
        """
        Update target tracking with new detections using a rolling window.
        If tracking is lost, keeps the old target pose.

        Args:
            new_detections: Optional new detections for target tracking

        Returns:
            True if target was successfully tracked, False if lost (but target is kept)
        """
        # Check if we have a current target
        if not self.current_target:
            return False

        # Add new detections to history if provided
        if new_detections is not None and new_detections.detections_length > 0:
            self.detection_history.append(new_detections)

        # If no detection history, can't track
        if not self.detection_history:
            logger.debug("No detection history for target tracking - using last known pose")
            return False

        # Collect all candidates from detection history
        all_candidates = []
        for detection_array in self.detection_history:
            all_candidates.extend(detection_array.detections)

        if not all_candidates:
            logger.debug("No candidates in detection history")
            return False

        # Use stage-dependent distance threshold
        max_distance = self.max_tracking_distance_threshold

        # Find best match across all recent detections
        match_result = find_best_object_match(
            target_obj=self.current_target,
            candidates=all_candidates,
            max_distance=max_distance,
            min_size_similarity=self.min_size_similarity,
        )

        if match_result.is_valid_match:
            self.current_target = match_result.matched_object
            self.target_grasp_pose = None  # Recompute grasp pose
            logger.debug(
                f"Target tracking successful: distance={match_result.distance:.3f}m, "
                f"size_similarity={match_result.size_similarity:.2f}, "
                f"confidence={match_result.confidence:.2f}"
            )
            return True

        logger.debug(
            f"Target tracking lost across {len(self.detection_history)} frames: "
            f"distance={match_result.distance:.3f}m, "
            f"size_similarity={match_result.size_similarity:.2f}, "
            f"thresholds: distance={max_distance:.3f}m, size={self.min_size_similarity:.2f}"
        )
        return False

    def compute_control(
        self,
        ee_pose: Pose,
        grasp_distance: float = 0.15,
        grasp_pitch_degrees: float = 45.0,
    ) -> tuple[Vector3 | None, Vector3 | None, bool, bool, Pose | None]:
        """
        Compute PBVS control with position and orientation servoing.

        Args:
            ee_pose: Current end-effector pose
            grasp_distance: Distance to maintain from target (meters)

        Returns:
            Tuple of (velocity_command, angular_velocity_command, target_reached, has_target, target_pose)
            - velocity_command: Linear velocity vector or None if no target (None in direct_ee_control mode)
            - angular_velocity_command: Angular velocity vector or None if no target (None in direct_ee_control mode)
            - target_reached: True if within target tolerance
            - has_target: True if currently tracking a target
            - target_pose: Target EE pose (only in direct_ee_control mode, otherwise None)
        """
        # Check if we have a target
        if not self.current_target:
            return None, None, False, False, None

        # Update target grasp pose with provided distance and pitch
        self.target_grasp_pose = update_target_grasp_pose(
            self.current_target.bbox.center, ee_pose, grasp_distance, grasp_pitch_degrees
        )

        if self.target_grasp_pose is None:
            logger.warning("Failed to compute grasp pose")
            return None, None, False, False, None

        # Compute errors for visualization before checking if reached (in case pose gets cleared)
        if self.direct_ee_control and self.target_grasp_pose:
            self.last_position_error = Vector3(
                self.target_grasp_pose.position.x - ee_pose.position.x,
                self.target_grasp_pose.position.y - ee_pose.position.y,
                self.target_grasp_pose.position.z - ee_pose.position.z,
            )

        # Check if target reached using our separate function
        target_reached = is_target_reached(self.target_grasp_pose, ee_pose, self.target_tolerance)

        # Return appropriate values based on control mode
        if self.direct_ee_control:
            # Direct control mode
            if self.target_grasp_pose:
                self.last_target_reached = target_reached
                # Return has_target=True since we have a target
                return None, None, target_reached, True, self.target_grasp_pose
            else:
                return None, None, False, True, None
        else:
            # Velocity control mode - use controller
            velocity_cmd, angular_velocity_cmd, _controller_reached = (
                self.controller.compute_control(ee_pose, self.target_grasp_pose)
            )
            # Return has_target=True since we have a target, regardless of tracking status
            return velocity_cmd, angular_velocity_cmd, target_reached, True, None

    def create_status_overlay(  # type: ignore[no-untyped-def]
        self,
        image: np.ndarray,  # type: ignore[type-arg]
        grasp_stage=None,
    ) -> np.ndarray:  # type: ignore[type-arg]
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            grasp_stage: Current grasp stage (optional)

        Returns:
            Image with PBVS status overlay
        """
        stage_value = grasp_stage.value if grasp_stage else "idle"
        return create_pbvs_visualization(
            image,
            self.current_target,
            self.last_position_error,
            self.last_target_reached,
            stage_value,
        )


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
    ) -> None:
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

    def clear_state(self) -> None:
        """Clear controller state."""
        self.last_position_error = None
        self.last_rotation_error = None
        self.last_velocity_cmd = None
        self.last_angular_velocity_cmd = None
        self.last_target_reached = False

    def compute_control(
        self, ee_pose: Pose, grasp_pose: Pose
    ) -> tuple[Vector3 | None, Vector3 | None, bool]:
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
        self.last_position_error = error  # type: ignore[assignment]

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

        self.last_velocity_cmd = velocity_cmd  # type: ignore[assignment]

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

        self.last_rotation_error = Vector3(roll_error, pitch_error, yaw_error)  # type: ignore[assignment]

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

        self.last_angular_velocity_cmd = angular_velocity  # type: ignore[assignment]

        return angular_velocity

    def create_status_overlay(
        self,
        image: np.ndarray,  # type: ignore[type-arg]
        current_target: Detection3D | None = None,
    ) -> np.ndarray:  # type: ignore[type-arg]
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            current_target: Current target object Detection3D (for display)

        Returns:
            Image with PBVS status overlay
        """
        return create_pbvs_visualization(
            image,
            current_target,
            self.last_position_error,
            self.last_target_reached,
            "velocity_control",
        )
