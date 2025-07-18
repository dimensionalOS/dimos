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
from typing import Optional, Tuple
from enum import Enum

from scipy.spatial.transform import Rotation as R
from dimos_lcm.geometry_msgs import Pose, Vector3, Quaternion, Point
from dimos_lcm.vision_msgs import Detection3D, Detection3DArray
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    yaw_towards_point,
    pose_to_matrix,
    euler_to_quaternion,
)
from dimos.manipulation.visual_servoing.utils import (
    find_best_object_match,
    create_pbvs_status_overlay,
    create_pbvs_controller_overlay,
)

logger = setup_logger("dimos.manipulation.pbvs")


class GraspStage(Enum):
    """Enum for different grasp stages."""

    PRE_GRASP = "pre_grasp"
    GRASP = "grasp"


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
        max_tracking_distance_threshold: float = 0.1,  # Max distance for target tracking (m)
        min_size_similarity: float = 0.7,  # Min size similarity threshold (0.0-1.0)
        pregrasp_distance: float = 0.15,  # 15cm pregrasp distance
        grasp_distance: float = 0.05,  # 5cm grasp distance (final approach)
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
            max_tracking_distance: Maximum distance for valid target tracking (m)
            min_size_similarity: Minimum size similarity for valid target tracking (0.0-1.0)
            pregrasp_distance: Distance to maintain before grasping (m)
            grasp_distance: Distance for final grasp approach (m)
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
        self.max_tracking_distance_threshold = max_tracking_distance_threshold
        self.min_size_similarity = min_size_similarity
        self.pregrasp_distance = pregrasp_distance
        self.grasp_distance = grasp_distance
        self.direct_ee_control = direct_ee_control
        self.grasp_pitch_degrees = 45.0  # Default grasp pitch in degrees (45° between level and top-down)

        # Target state
        self.current_target = None
        self.target_grasp_pose = None
        self.grasp_stage = GraspStage.PRE_GRASP

        # For direct control mode visualization
        self.last_position_error = None
        self.last_target_reached = False

        logger.info(
            f"Initialized PBVS system with controller gains: pos={position_gain}, rot={rotation_gain}, "
            f"pregrasp_distance={pregrasp_distance}m, grasp_distance={grasp_distance}m, "
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
            self.grasp_stage = GraspStage.PRE_GRASP  # Reset to pre-grasp stage
            logger.info(f"New target set: ID {target_object.id}")
            return True
        return False

    def clear_target(self):
        """Clear the current target."""
        self.current_target = None
        self.target_grasp_pose = None
        self.grasp_stage = GraspStage.PRE_GRASP
        self.last_position_error = None
        self.last_target_reached = False
        if self.controller:
            self.controller.clear_state()
        logger.info("Target cleared")

    def get_current_target(self) -> Optional[Detection3D]:
        """
        Get the current target object.

        Returns:
            Current target Detection3D or None if no target selected
        """
        return self.current_target

    def set_grasp_stage(self, stage: GraspStage):
        """
        Set the grasp stage.

        Args:
            stage: The new grasp stage
        """
        self.grasp_stage = stage

    def set_grasp_pitch(self, pitch_degrees: float):
        """
        Set the grasp pitch angle in degrees.
        
        Args:
            pitch_degrees: Grasp pitch angle in degrees (0-90)
                          0° = level grasp (horizontal)
                          90° = top-down grasp (vertical)
        """
        # Clamp to valid range
        pitch_degrees = max(0.0, min(90.0, pitch_degrees))
        self.grasp_pitch_degrees = pitch_degrees
        # Reset target grasp pose to recompute with new pitch
        self.target_grasp_pose = None

    def is_target_reached(self, ee_pose: Pose) -> bool:
        """
        Check if the current target stage has been reached.

        Args:
            ee_pose: Current end-effector pose

        Returns:
            True if current stage target is reached, False otherwise
        """
        if not self.target_grasp_pose:
            return False

        # Calculate position error
        error_x = self.target_grasp_pose.position.x - ee_pose.position.x
        error_y = self.target_grasp_pose.position.y - ee_pose.position.y
        error_z = self.target_grasp_pose.position.z - ee_pose.position.z

        error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        stage_reached = error_magnitude < self.target_tolerance

        # Handle stage transitions
        if stage_reached and self.grasp_stage == GraspStage.PRE_GRASP:
            return True  # Signal that pre-grasp target was reached
        elif stage_reached and self.grasp_stage == GraspStage.GRASP:
            # Grasp reached, clear target
            logger.info("Grasp position reached, clearing target")
            self.clear_target()
            return True

        return False

    def update_target_tracking(self, new_detections: Detection3DArray) -> bool:
        """
        Update target by matching to closest object in new detections.
        If tracking is lost, keeps the old target pose.

        Args:
            new_detections: List of newly detected objects

        Returns:
            True if target was successfully tracked, False if lost (but target is kept)
        """
        if not self.current_target or not self.current_target.bbox or not self.current_target.bbox.center:
            return False

        if not new_detections or new_detections.detections_length == 0:
            logger.debug("No detections for target tracking - using last known pose")
            return False

        # Use stage-dependent distance threshold
        max_distance = self.max_tracking_distance_threshold

        # Find best match using standardized utility function
        match_result = find_best_object_match(
            target_obj=self.current_target,
            candidates=new_detections.detections,
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
            f"Target tracking lost: distance={match_result.distance:.3f}m, "
            f"size_similarity={match_result.size_similarity:.2f}, "
            f"thresholds: distance={max_distance:.3f}m, size={self.min_size_similarity:.2f}"
        )
        return False

    def _update_target_grasp_pose(self, ee_pose: Pose):
        """
        Update target grasp pose based on current target and EE pose.

        Args:
            ee_pose: Current end-effector pose
        """
        if not self.current_target or not self.current_target.bbox or not self.current_target.bbox.center:
            return

        # Get target position
        target_pos = self.current_target.bbox.center.position

        # Calculate orientation pointing from target towards EE
        yaw_to_ee = yaw_towards_point(
            Vector3(target_pos.x, target_pos.y, target_pos.z), ee_pose.position
        )

        # Create target pose with proper orientation
        # Convert grasp pitch from degrees to radians with mapping:
        # 0° (level) -> π/2 (1.57 rad), 90° (top-down) -> π (3.14 rad)
        pitch_radians = 1.57 + (self.grasp_pitch_degrees * np.pi / 180.0 / 2.0)
        
        # Convert euler angles to quaternion using utility function
        euler = Vector3(0.0, pitch_radians, yaw_to_ee)  # roll=0, pitch=mapped, yaw=calculated
        target_orientation = euler_to_quaternion(euler)

        target_pose = Pose(target_pos, target_orientation)

        # Apply grasp distance
        distance = (
            self.pregrasp_distance
            if self.grasp_stage == GraspStage.PRE_GRASP
            else self.grasp_distance
        )
        self.target_grasp_pose = self._apply_grasp_distance(target_pose, distance)

    def _apply_grasp_distance(self, target_pose: Pose, distance: float) -> Pose:
        """
        Apply grasp distance offset to target pose along its approach direction.

        Args:
            target_pose: Target grasp pose
            distance: Distance to offset along the approach direction (meters)

        Returns:
            Target pose offset by the specified distance along its approach direction
        """
        # Convert pose to transformation matrix to extract rotation
        T_target = pose_to_matrix(target_pose)
        rotation_matrix = T_target[:3, :3]
        
        # Define the approach vector based on the target pose orientation
        # Assuming the gripper approaches along its local -z axis (common for downward grasps)
        # You can change this to [1, 0, 0] for x-axis or [0, 1, 0] for y-axis based on your gripper
        approach_vector_local = np.array([0, 0, -1])
        
        # Transform approach vector to world coordinates
        approach_vector_world = rotation_matrix @ approach_vector_local
        
        # Apply offset along the approach direction
        offset_position = Point(
            target_pose.position.x + distance * approach_vector_world[0],
            target_pose.position.y + distance * approach_vector_world[1],
            target_pose.position.z + distance * approach_vector_world[2],
        )
        
        return Pose(offset_position, target_pose.orientation)

    def compute_control(
        self, ee_pose: Pose, new_detections: Optional[Detection3DArray] = None
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
        if not self.current_target or not self.current_target.bbox or not self.current_target.bbox.center:
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
        if not self.current_target:
            logger.info("No current target")

        self._update_target_grasp_pose(ee_pose)

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
        target_reached = self.is_target_reached(ee_pose)

        # If stage transitioned, recompute target grasp pose
        if (
            target_reached
            and self.grasp_stage == GraspStage.GRASP
            and self.target_grasp_pose is None
        ):
            self._update_target_grasp_pose(ee_pose)

        # Return appropriate values based on control mode
        if self.direct_ee_control:
            # Direct control mode
            if self.target_grasp_pose:
                self.last_target_reached = target_reached
                return None, None, target_reached, target_tracked, self.target_grasp_pose
            else:
                return None, None, False, target_tracked, None
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

        Returns:
            Image with PBVS status overlay
        """
        if self.direct_ee_control:
            # Use direct control overlay
            return create_pbvs_status_overlay(
                image,
                self.current_target,
                self.last_position_error,
                self.last_target_reached,
                self.target_grasp_pose,
                self.grasp_stage.value,
                is_direct_control=True,
            )
        else:
            # Use controller's overlay for velocity mode
            return self.controller.create_status_overlay(
                image,
                self.current_target,
                self.direct_ee_control,
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
        current_target: Optional[Detection3D] = None,
        direct_ee_control: bool = False,
    ) -> np.ndarray:
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            current_target: Current target object Detection3D (for display)
            direct_ee_control: Whether in direct EE control mode

        Returns:
            Image with PBVS status overlay
        """
        return create_pbvs_controller_overlay(
            image,
            current_target,
            self.last_position_error,
            self.last_rotation_error,
            self.last_velocity_cmd,
            self.last_angular_velocity_cmd,
            self.last_target_reached,
            direct_ee_control,
        )
