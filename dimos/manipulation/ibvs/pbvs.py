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
Position-Based Visual Servoing (PBVS) controller for eye-in-hand configuration.
Works with manipulator frame origin and proper robot arm conventions.
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any, List
import cv2

from scipy.spatial.transform import Rotation as R
from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    pose_to_matrix,
    apply_transform,
    optical_to_robot_frame,
    yaw_towards_point,
)

logger = setup_logger("dimos.manipulation.pbvs")


class PBVSController:
    """
    Position-Based Visual Servoing controller for eye-in-hand cameras.
    Supports manipulator frame origin and robot arm conventions.

    Handles:
    - Position and orientation error computation
    - Velocity command generation with gain control
    - Automatic target tracking across frames
    - Frame transformations from ZED to robot conventions
    - Pregrasp distance functionality
    - 6DOF EE to camera transform handling
    """

    def __init__(
        self,
        position_gain: float = 0.5,
        rotation_gain: float = 0.3,
        max_velocity: float = 0.1,  # m/s
        max_angular_velocity: float = 0.5,  # rad/s
        target_tolerance: float = 0.01,  # 5cm
        tracking_distance_threshold: float = 0.05,  # 5cm for target tracking
        pregrasp_distance: float = 0.15,  # 15cm pregrasp distance
        ee_to_camera_transform: Vector = Vector(
            [0.0, 0.0, -0.06, 0.0, -1.57, 0.0]
        ),  # 6DOF: [x,y,z,rx,ry,rz]
    ):
        """
        Initialize PBVS controller.

        Args:
            position_gain: Proportional gain for position control
            rotation_gain: Proportional gain for rotation control
            max_velocity: Maximum linear velocity command magnitude (m/s)
            max_angular_velocity: Maximum angular velocity command magnitude (rad/s)
            target_tolerance: Distance threshold for considering target reached (m)
            tracking_distance_threshold: Max distance for target association (m)
            pregrasp_distance: Distance to maintain before grasping (m)
            ee_to_camera_transform: 6DOF transform from EE to camera [x,y,z,rx,ry,rz]
        """
        self.position_gain = position_gain
        self.rotation_gain = rotation_gain
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.target_tolerance = target_tolerance
        self.tracking_distance_threshold = tracking_distance_threshold
        self.pregrasp_distance = pregrasp_distance
        self.ee_to_camera_transform_vec = ee_to_camera_transform

        # State variables
        self.current_target = None
        self.last_position_error = None
        self.last_rotation_error = None
        self.last_velocity_cmd = None
        self.last_angular_velocity_cmd = None
        self.last_target_reached = False

        # Manipulator frame origin
        self.manipulator_origin = None  # Transform matrix from world to manipulator frame
        self.manipulator_origin_pose = None  # Original pose for reference

        # Create 6DOF EE to camera transform matrix
        self.ee_to_camera_transform = self._create_ee_to_camera_transform()

        logger.info(
            f"Initialized PBVS controller: pos_gain={position_gain}, rot_gain={rotation_gain}, "
            f"max_vel={max_velocity}m/s, max_ang_vel={max_angular_velocity}rad/s, "
            f"target_tolerance={target_tolerance}m, pregrasp_distance={pregrasp_distance}m, "
            f"ee_to_camera_transform={ee_to_camera_transform.to_list()}"
        )

    def _create_ee_to_camera_transform(self) -> np.ndarray:
        """
        Create 6DOF transform matrix from EE to camera frame.

        Returns:
            4x4 transformation matrix from EE to camera
        """
        # Extract position and rotation from 6DOF vector
        pos = self.ee_to_camera_transform_vec.to_list()[:3]
        rot = self.ee_to_camera_transform_vec.to_list()[3:6]  # euler angles: [rx, ry, rz]

        # Create transformation matrix
        T_ee_to_cam = np.eye(4)
        T_ee_to_cam[0:3, 3] = pos

        # Apply rotation using scipy (treating as euler angles)
        if np.linalg.norm(rot) > 1e-6:
            rotation = R.from_euler("xyz", rot)
            T_ee_to_cam[0:3, 0:3] = rotation.as_matrix()

        return T_ee_to_cam

    def set_manipulator_origin(self, camera_pose: Pose):
        """
        Set the manipulator frame origin based on current camera pose.
        This establishes the robot arm coordinate frame.

        Args:
            camera_pose: Current camera pose in world frame
        """
        self.manipulator_origin_pose = camera_pose

        # Create transform matrix from ZED world to manipulator origin
        # This is the inverse of the camera pose at origin
        T_world_to_origin = pose_to_matrix(camera_pose)
        self.manipulator_origin = np.linalg.inv(T_world_to_origin)

        logger.info(
            f"Set manipulator origin at pose: pos=({camera_pose.position.x:.3f}, "
            f"{camera_pose.position.y:.3f}, {camera_pose.position.z:.3f})"
        )

    def _apply_pregrasp_distance(self, target_pose: Pose) -> Pose:
        """
        Apply pregrasp distance to target pose by moving back towards robot origin.

        Args:
            target_pose: Target pose in robot frame

        Returns:
            Modified target pose with pregrasp distance applied
        """
        # Get approach vector (from target position towards robot origin)
        target_pos = np.array(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        )
        robot_origin = np.array([0.0, 0.0, 0.0])  # Robot origin in robot frame
        approach_vector = robot_origin - target_pos  # Vector pointing towards robot

        # Normalize approach vector
        approach_magnitude = np.linalg.norm(approach_vector)
        if approach_magnitude > 1e-6:  # Avoid division by zero
            norm_approach_vector = approach_vector / approach_magnitude
        else:
            norm_approach_vector = np.array([0.0, 0.0, 0.0])

        # Move back by pregrasp distance towards robot
        offset_vector = self.pregrasp_distance * norm_approach_vector

        # Apply offset to target position
        new_position = Vector3(
            target_pose.position.x + offset_vector[0],
            target_pose.position.y + offset_vector[1],
            target_pose.position.z + offset_vector[2],
        )

        return Pose(new_position, target_pose.orientation)

    def _update_target_robot_frame(self):
        """Update current target with robot frame coordinates."""
        if not self.current_target or "position" not in self.current_target:
            return

        # Get target position in ZED world frame
        target_pos = self.current_target["position"]
        target_pose_zed = Pose(target_pos, Quaternion())  # Identity quaternion

        # Transform to manipulator frame
        target_pose_manip = apply_transform(target_pose_zed, self.manipulator_origin)

        # Calculate orientation pointing at origin (in robot frame)
        yaw_to_origin = yaw_towards_point(
            Vector(
                target_pose_manip.position.x,
                target_pose_manip.position.y,
                target_pose_manip.position.z,
            )
        )

        # Create target pose with proper orientation
        # Convert euler angles to quaternion using scipy
        euler = [0.0, 1.57, yaw_to_origin]  # roll=0, pitch=90deg, yaw=calculated
        quat = R.from_euler("xyz", euler).as_quat()  # [x, y, z, w]
        target_orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        target_pose_robot = Pose(target_pose_manip.position, target_orientation)

        # Apply pregrasp distance
        target_pose_pregrasp = self._apply_pregrasp_distance(target_pose_robot)

        # Update target with robot frame pose
        self.current_target["robot_position"] = target_pose_pregrasp.position
        self.current_target["robot_rotation"] = target_pose_pregrasp.orientation

    def set_target(self, target_object: Dict[str, Any]) -> bool:
        """
        Set a new target object for servoing.
        Requires manipulator origin to be set.

        Args:
            target_object: Object dict with at least 'position' field

        Returns:
            True if target was set successfully, False if no origin set
        """
        # Require origin to be set
        if self.manipulator_origin is None:
            logger.warning("Cannot set target: No manipulator origin set")
            return False

        if target_object and "position" in target_object:
            self.current_target = target_object

            # Update to robot frame
            self._update_target_robot_frame()

            logger.info(f"New target set: ID {target_object.get('object_id', 'unknown')}")
            return True
        return False

    def clear_target(self):
        """Clear the current target."""
        self.current_target = None
        self.last_position_error = None
        self.last_rotation_error = None
        self.last_velocity_cmd = None
        self.last_angular_velocity_cmd = None
        self.last_target_reached = False
        logger.info("Target cleared")

    def update_target_tracking(self, new_detections: List[Dict[str, Any]]) -> bool:
        """
        Update target by matching to closest object in new detections.

        Args:
            new_detections: List of newly detected objects

        Returns:
            True if target was successfully tracked, False if lost
        """
        if not self.current_target or "position" not in self.current_target:
            return False

        if not new_detections:
            logger.debug("No detections for target tracking")
            return False

        # Get current target position (in ZED world frame for matching)
        target_pos = self.current_target["position"]
        if isinstance(target_pos, (Vector, Vector3)):
            target_xyz = np.array([target_pos.x, target_pos.y, target_pos.z])
        else:
            target_xyz = np.array([target_pos["x"], target_pos["y"], target_pos["z"]])

        # Find closest match
        min_distance = float("inf")
        best_match = None

        for detection in new_detections:
            if "position" not in detection:
                continue

            det_pos = detection["position"]
            if isinstance(det_pos, (Vector, Vector3)):
                det_xyz = np.array([det_pos.x, det_pos.y, det_pos.z])
            else:
                det_xyz = np.array([det_pos["x"], det_pos["y"], det_pos["z"]])

            distance = np.linalg.norm(target_xyz - det_xyz)

            if distance < min_distance and distance < self.tracking_distance_threshold:
                min_distance = distance
                best_match = detection

        if best_match:
            self.current_target = best_match
            # Update to robot frame
            self._update_target_robot_frame()
            return True
        return False

    def _get_ee_pose_from_camera(self, camera_pose: Pose) -> Pose:
        """
        Get end-effector pose from camera pose using 6DOF EE to camera transform.

        Args:
            camera_pose: Current camera pose in robot frame

        Returns:
            End-effector pose in robot frame
        """
        # Transform camera pose to EE frame
        camera_transform = pose_to_matrix(camera_pose)
        ee_transform = camera_transform @ np.linalg.inv(self.ee_to_camera_transform)

        # Extract position and rotation
        ee_pos = Vector3(ee_transform[0:3, 3])
        ee_rot_matrix = ee_transform[0:3, 0:3]

        # Convert rotation matrix to quaternion

        # Ensure the rotation matrix is valid (orthogonal with det=1)
        try:
            rotation = R.from_matrix(ee_rot_matrix)
            quat = rotation.as_quat()  # [x, y, z, w]
            ee_orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        except ValueError as e:
            logger.warning(f"Invalid rotation matrix in EE pose calculation: {e}")
            # Fallback to identity quaternion
            ee_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return Pose(ee_pos, ee_orientation)

    def compute_control(
        self, camera_pose: Pose, new_detections: Optional[List[Dict[str, Any]]] = None
    ) -> Tuple[Optional[Vector], Optional[Vector], bool, bool]:
        """
        Compute PBVS control with position and orientation servoing.

        Args:
            camera_pose: Current camera pose in ZED world frame
            new_detections: Optional new detections for target tracking

        Returns:
            Tuple of (velocity_command, angular_velocity_command, target_reached, has_target)
            - velocity_command: Linear velocity vector or None if no target
            - angular_velocity_command: Angular velocity vector or None if no target
            - target_reached: True if within target tolerance
            - has_target: True if currently tracking a target
        """
        # Check if we have a target and origin
        if not self.current_target or "position" not in self.current_target:
            return None, None, False, False

        if self.manipulator_origin is None:
            logger.warning("Cannot compute control: No manipulator origin set")
            return None, None, False, False

        # Try to update target tracking if new detections provided
        if new_detections is not None:
            self.update_target_tracking(new_detections)

        # Transform camera pose to robot frame
        camera_pose_robot = apply_transform(camera_pose, self.manipulator_origin)

        # Get EE pose from camera pose
        ee_pose_robot = self._get_ee_pose_from_camera(camera_pose_robot)

        # Get target in robot frame
        target_pos = self.current_target.get("robot_position")
        target_rot = self.current_target.get("robot_rotation")

        if target_pos is None or target_rot is None:
            logger.warning("Target position or rotation not available")
            return None, None, False, False

        # Calculate position error (target - EE position)
        error = Vector3(
            target_pos.x - ee_pose_robot.position.x,
            target_pos.y - ee_pose_robot.position.y,
            target_pos.z - ee_pose_robot.position.z,
        )
        self.last_position_error = error

        # Compute velocity command with proportional control
        velocity_cmd = Vector(
            [
                error.x * self.position_gain,
                error.y * self.position_gain,
                error.z * self.position_gain,
            ]
        )

        # Limit velocity magnitude
        vel_magnitude = np.linalg.norm([velocity_cmd.x, velocity_cmd.y, velocity_cmd.z])
        if vel_magnitude > self.max_velocity:
            scale = self.max_velocity / vel_magnitude
            velocity_cmd = Vector(
                [
                    float(velocity_cmd.x * scale),
                    float(velocity_cmd.y * scale),
                    float(velocity_cmd.z * scale),
                ]
            )

        self.last_velocity_cmd = velocity_cmd

        # Compute angular velocity for orientation control
        angular_velocity_cmd = self._compute_angular_velocity(target_rot, ee_pose_robot)

        # Check if target reached
        error_magnitude = np.linalg.norm([error.x, error.y, error.z])
        target_reached = bool(error_magnitude < self.target_tolerance)
        self.last_target_reached = target_reached

        # Clear target only if it's reached
        if target_reached:
            logger.info(
                f"Target reached! Clearing target ID {self.current_target.get('object_id', 'unknown')}"
            )
            self.clear_target()

        return velocity_cmd, angular_velocity_cmd, target_reached, True

    def _compute_angular_velocity(self, target_rot: Quaternion, current_pose: Pose) -> Vector:
        """
        Compute angular velocity commands for orientation control.
        Uses quaternion error computation for better numerical stability.

        Args:
            target_rot: Target orientation (quaternion)
            current_pose: Current EE pose

        Returns:
            Angular velocity command as Vector
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

        self.last_rotation_error = Vector([roll_error, pitch_error, yaw_error])

        # Apply proportional control
        angular_velocity = Vector(
            [
                roll_error * self.rotation_gain,
                pitch_error * self.rotation_gain,
                yaw_error * self.rotation_gain,
            ]
        )

        # Limit angular velocity magnitude
        ang_vel_magnitude = np.sqrt(
            angular_velocity.x**2 + angular_velocity.y**2 + angular_velocity.z**2
        )
        if ang_vel_magnitude > self.max_angular_velocity:
            scale = self.max_angular_velocity / ang_vel_magnitude
            angular_velocity = angular_velocity * scale

        self.last_angular_velocity_cmd = angular_velocity

        return angular_velocity

    def get_camera_pose_robot_frame(self, camera_pose_zed: Pose) -> Optional[Pose]:
        """
        Get camera pose in robot frame coordinates.

        Args:
            camera_pose_zed: Camera pose in ZED world frame

        Returns:
            Camera pose in robot frame or None if no origin set
        """
        if self.manipulator_origin is None:
            return None

        camera_pose_manip = apply_transform(camera_pose_zed, self.manipulator_origin)
        return camera_pose_manip

    def get_ee_pose_robot_frame(self, camera_pose_zed: Pose) -> Optional[Pose]:
        """
        Get end-effector pose in robot frame coordinates.

        Args:
            camera_pose_zed: Camera pose in ZED world frame

        Returns:
            End-effector pose in robot frame or None if no origin set
        """
        if self.manipulator_origin is None:
            return None

        camera_pose_robot = apply_transform(camera_pose_zed, self.manipulator_origin)
        return self._get_ee_pose_from_camera(camera_pose_robot)

    def get_object_pose_robot_frame(
        self, object_pos_zed: Vector
    ) -> Optional[Tuple[Vector, Vector]]:
        """
        Get object pose in robot frame coordinates with orientation.

        Args:
            object_pos_zed: Object position in ZED world frame

        Returns:
            Tuple of (position, rotation) in robot frame or None if no origin set
        """
        if self.manipulator_origin is None:
            return None

        # Transform position
        obj_pose_zed = Pose(object_pos_zed, Quaternion())  # Identity quaternion
        obj_pose_manip = apply_transform(obj_pose_zed, self.manipulator_origin)

        # Calculate orientation pointing at origin
        yaw_to_origin = yaw_towards_point(
            Vector(obj_pose_manip.position.x, obj_pose_manip.position.y, obj_pose_manip.position.z)
        )

        # Convert euler angles to quaternion
        euler = [0.0, 0.0, yaw_to_origin]  # Level grasp
        quat = R.from_euler("xyz", euler).as_quat()  # [x, y, z, w]
        orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        return obj_pose_manip.position, orientation

    def create_status_overlay(
        self, image: np.ndarray, camera_intrinsics: Optional[list] = None
    ) -> np.ndarray:
        """
        Create PBVS status overlay on image.

        Args:
            image: Input image
            camera_intrinsics: Optional [fx, fy, cx, cy] (not used)

        Returns:
            Image with PBVS status overlay
        """
        viz_img = image.copy()
        height, width = image.shape[:2]

        # Status panel
        if self.current_target:
            panel_height = 140  # Adjusted panel height
            panel_y = height - panel_height
            overlay = viz_img.copy()
            cv2.rectangle(overlay, (0, panel_y), (width, height), (0, 0, 0), -1)
            viz_img = cv2.addWeighted(viz_img, 0.7, overlay, 0.3, 0)

            # Status text
            y = panel_y + 20
            cv2.putText(
                viz_img, "PBVS Status", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
            )

            # Add frame info
            frame_text = (
                "Frame: Robot" if self.manipulator_origin is not None else "Frame: ZED World"
            )
            cv2.putText(
                viz_img, frame_text, (200, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
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

            if self.last_velocity_cmd:
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

            if self.last_angular_velocity_cmd:
                cv2.putText(
                    viz_img,
                    f"Ang Vel: ({self.last_angular_velocity_cmd.x:.2f}, {self.last_angular_velocity_cmd.y:.2f}, {self.last_angular_velocity_cmd.z:.2f})rad/s",
                    (10, y + 105),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 0),
                    1,
                )

            # Add config info
            ee_transform = self.ee_to_camera_transform_vec.to_list()
            cv2.putText(
                viz_img,
                f"Pregrasp: {self.pregrasp_distance:.3f}m | EE Transform: [{ee_transform[0]:.2f},{ee_transform[1]:.2f},{ee_transform[2]:.2f}]",
                (10, y + 125),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
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
