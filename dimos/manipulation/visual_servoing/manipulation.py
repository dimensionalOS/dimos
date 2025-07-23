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
Manipulation system for robotic grasping with visual servoing.
Handles grasping logic, state machine, and hardware coordination.
"""

import cv2
import time
from typing import Optional, Tuple, Any
from enum import Enum
from collections import deque

import numpy as np

from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.perception.common.utils import (
    find_clicked_detection,
)
from dimos.manipulation.visual_servoing.utils import (
    create_manipulation_visualization,
)
from dimos.utils.transform_utils import (
    pose_to_matrix,
    matrix_to_pose,
    create_transform_from_6dof,
    compose_transforms,
)
from dimos.utils.logging_config import setup_logger
from dimos_lcm.geometry_msgs import Vector3, Pose
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray

logger = setup_logger("dimos.manipulation.manipulation")


class GraspStage(Enum):
    """Enum for different grasp stages."""

    IDLE = "idle"  # No target set
    PRE_GRASP = "pre_grasp"  # Target set, moving to pre-grasp position
    GRASP = "grasp"  # Executing final grasp
    CLOSE_AND_RETRACT = "close_and_retract"  # Close gripper and retract


class Feedback:
    """
    Feedback data returned by the manipulation system update.

    Contains comprehensive state information about the manipulation process.
    """

    def __init__(
        self,
        grasp_stage: GraspStage,
        target_tracked: bool,
        last_commanded_pose: Optional[Pose] = None,
        current_ee_pose: Optional[Pose] = None,
        current_camera_pose: Optional[Pose] = None,
        target_pose: Optional[Pose] = None,
        waiting_for_reach: bool = False,
        grasp_successful: Optional[bool] = None,
    ):
        self.grasp_stage = grasp_stage
        self.target_tracked = target_tracked
        self.last_commanded_pose = last_commanded_pose
        self.current_ee_pose = current_ee_pose
        self.current_camera_pose = current_camera_pose
        self.target_pose = target_pose
        self.waiting_for_reach = waiting_for_reach
        self.grasp_successful = grasp_successful


class Manipulation:
    """
    High-level manipulation orchestrator for visual servoing and grasping.

    Handles:
    - State machine for grasping sequences
    - Grasp execution logic
    - Coordination between perception and control

    This class is hardware-agnostic and accepts camera and arm objects.
    """

    def __init__(
        self,
        camera: Any,  # Generic camera object with required interface
        arm: Any,  # Generic arm object with required interface
        ee_to_camera_6dof: Optional[list] = None,
    ):
        """
        Initialize manipulation system.

        Args:
            camera: Camera object with capture_frame_with_pose() and calculate_intrinsics() methods
            arm: Robot arm object with get_ee_pose(), cmd_ee_pose(),
                 cmd_gripper_ctrl(), release_gripper(), softStop(), gotoZero(), gotoObserve(), and disable() methods
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
        """
        self.camera = camera
        self.arm = arm

        # Default EE to camera transform if not provided
        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [-0.065, 0.03, -0.105, 0.0, -1.57, 0.0]

        # Create transform matrices
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        # Get camera intrinsics
        cam_intrinsics = camera.calculate_intrinsics()
        camera_intrinsics = [
            cam_intrinsics["focal_length_x"],
            cam_intrinsics["focal_length_y"],
            cam_intrinsics["principal_point_x"],
            cam_intrinsics["principal_point_y"],
        ]

        # Initialize processors
        self.detector = Detection3DProcessor(camera_intrinsics)
        self.pbvs = PBVS(
            target_tolerance=0.05,
        )

        # Control state
        self.last_valid_target = None
        self.waiting_for_reach = False  # True when waiting for robot to reach commanded pose
        self.last_commanded_pose = None  # Last pose sent to robot
        self.target_updated = False  # True when target has been updated with fresh detections
        self.waiting_start_time = None  # Time when waiting for reach started
        self.reach_pose_timeout = 10.0  # Timeout for reaching commanded pose (seconds)

        # Grasp parameters
        self.grasp_width_offset = 0.03  # Default grasp width offset
        self.grasp_pitch_degrees = 30.0  # Default grasp pitch in degrees
        self.pregrasp_distance = 0.25  # Distance to maintain before grasping (m)
        self.grasp_distance_range = 0.03  # Range for grasp distance mapping (±5cm = ±0.05m)
        self.grasp_close_delay = 2.0  # Time to wait at grasp pose before closing (seconds)
        self.grasp_reached_time = None  # Time when grasp pose was reached
        self.gripper_max_opening = 0.07  # Maximum gripper opening (m)

        # Grasp stage tracking
        self.grasp_stage = GraspStage.IDLE

        # Pose stabilization tracking
        self.pose_history_size = 4  # Number of poses to check for stabilization
        self.pose_stabilization_threshold = 0.01  # 1cm threshold for stabilization
        self.stabilization_timeout = 15.0  # Timeout in seconds before giving up
        self.stabilization_start_time = None  # Time when stabilization started
        self.reached_poses = deque(
            maxlen=self.pose_history_size
        )  # Only stores poses that were reached
        self.adjustment_count = 0

        # State for visualization
        self.current_visualization = None
        self.last_detection_3d_array = None
        self.last_detection_2d_array = None

        # Grasp result
        self.pick_success = None  # True if last grasp was successful
        self.final_pregrasp_pose = None  # Store the final pre-grasp pose for retraction

        # Go to observe position
        self.arm.gotoObserve()

    def set_grasp_stage(self, stage: GraspStage):
        """
        Set the grasp stage.

        Args:
            stage: The new grasp stage
        """
        self.grasp_stage = stage
        logger.info(f"Grasp stage: {stage.value}")

    def set_grasp_pitch(self, pitch_degrees: float):
        """
        Set the grasp pitch angle.

        Args:
            pitch_degrees: Grasp pitch angle in degrees (0-90)
                          0 = level grasp, 90 = top-down grasp
        """
        # Clamp to valid range
        pitch_degrees = max(0.0, min(90.0, pitch_degrees))
        self.grasp_pitch_degrees = pitch_degrees
        self.pbvs.set_grasp_pitch(pitch_degrees)

    def _check_reach_timeout(self) -> bool:
        """
        Check if robot has exceeded timeout while reaching pose.

        Returns:
            True if timeout exceeded, False otherwise
        """
        if (
            self.waiting_start_time
            and (time.time() - self.waiting_start_time) > self.reach_pose_timeout
        ):
            logger.warning(f"Robot failed to reach pose within {self.reach_pose_timeout}s timeout")
            self.reset_to_idle()
            return True
        return False

    def _update_tracking(self, detection_3d_array: Optional[Detection3DArray]) -> bool:
        """
        Update tracking with new detections in a compact way.

        Args:
            detection_3d_array: Optional detection array

        Returns:
            True if target is tracked
        """
        if not detection_3d_array:
            return False

        target_tracked = self.pbvs.update_tracking(detection_3d_array)
        if target_tracked:
            self.target_updated = True
            self.last_valid_target = self.pbvs.get_current_target()
        return target_tracked

    def reset_to_idle(self):
        """Reset the manipulation system to IDLE state."""
        self.pbvs.clear_target()
        self.grasp_stage = GraspStage.IDLE
        self.reached_poses.clear()
        self.adjustment_count = 0
        self.waiting_for_reach = False
        self.last_commanded_pose = None
        self.target_updated = False
        self.stabilization_start_time = None
        self.grasp_reached_time = None
        self.waiting_start_time = None
        self.pick_success = None
        self.final_pregrasp_pose = None

        self.arm.gotoObserve()

    def execute_idle(self):
        """Execute idle stage: just visualization, no control."""
        # Nothing to do in idle
        pass

    def execute_pre_grasp(self):
        """Execute pre-grasp stage: visual servoing to pre-grasp position."""
        ee_pose = self.arm.get_ee_pose()

        # Check if waiting for robot to reach commanded pose
        if self.waiting_for_reach and self.last_commanded_pose:
            # Check for timeout
            if self._check_reach_timeout():
                return

            reached = self.pbvs.is_target_reached(ee_pose)

            if reached:
                self.waiting_for_reach = False
                self.waiting_start_time = None
                self.reached_poses.append(self.last_commanded_pose)
                self.target_updated = False  # Reset flag so we wait for fresh update
                time.sleep(0.3)

            # While waiting, don't process new commands
            return

        # Check stabilization timeout
        if (
            self.stabilization_start_time
            and (time.time() - self.stabilization_start_time) > self.stabilization_timeout
        ):
            logger.warning(
                f"Failed to get stable grasp after {self.stabilization_timeout} seconds, resetting"
            )
            self.reset_to_idle()
            return

        # PBVS control with pre-grasp distance
        _, _, _, has_target, target_pose = self.pbvs.compute_control(
            ee_pose, self.pregrasp_distance
        )

        # Handle pose control
        if target_pose and has_target:
            # Check if we have enough reached poses and they're stable
            if self.check_target_stabilized():
                logger.info("Target stabilized, transitioning to GRASP")
                self.final_pregrasp_pose = self.last_commanded_pose
                self.grasp_stage = GraspStage.GRASP
                self.adjustment_count = 0
                self.waiting_for_reach = False
            elif not self.waiting_for_reach and self.target_updated:
                # Command the pose only if target has been updated
                self.arm.cmd_ee_pose(target_pose)
                self.last_commanded_pose = target_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
                self.target_updated = False
                self.adjustment_count += 1
                time.sleep(0.2)

    def execute_grasp(self):
        """Execute grasp stage: move to final grasp position."""
        ee_pose = self.arm.get_ee_pose()

        # Handle waiting with special grasp logic
        if self.waiting_for_reach:
            if self._check_reach_timeout():
                return

            if self.pbvs.is_target_reached(ee_pose) and not self.grasp_reached_time:
                self.grasp_reached_time = time.time()
                self.waiting_start_time = None

            # Check if delay completed
            if (
                self.grasp_reached_time
                and (time.time() - self.grasp_reached_time) >= self.grasp_close_delay
            ):
                logger.info("Grasp delay completed, closing gripper")
                self.grasp_stage = GraspStage.CLOSE_AND_RETRACT
                self.waiting_for_reach = False
            return

        # Only command grasp if not waiting and have valid target
        if self.last_valid_target:
            # Calculate grasp distance based on pitch angle (0° -> -5cm, 90° -> +5cm)
            normalized_pitch = self.grasp_pitch_degrees / 90.0
            grasp_distance = -self.grasp_distance_range + (
                2 * self.grasp_distance_range * normalized_pitch
            )

            # PBVS control with calculated grasp distance
            _, _, _, has_target, target_pose = self.pbvs.compute_control(ee_pose, grasp_distance)

            if target_pose and has_target:
                # Calculate gripper opening
                object_width = self.last_valid_target.bbox.size.x
                gripper_opening = max(
                    0.005, min(object_width + self.grasp_width_offset, self.gripper_max_opening)
                )

                logger.info(f"Executing grasp: gripper={gripper_opening * 1000:.1f}mm")

                # Command gripper and pose
                self.arm.cmd_gripper_ctrl(gripper_opening)
                self.arm.cmd_ee_pose(target_pose, line_mode=True)
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()

    def execute_close_and_retract(self):
        """Execute the retraction sequence after gripper has been closed."""
        ee_pose = self.arm.get_ee_pose()

        if self.waiting_for_reach:
            if self._check_reach_timeout():
                return

            # Check if reached retraction pose
            original_target = self.pbvs.target_grasp_pose
            self.pbvs.target_grasp_pose = self.final_pregrasp_pose
            reached = self.pbvs.is_target_reached(ee_pose)
            self.pbvs.target_grasp_pose = original_target

            if reached:
                logger.info("Reached pre-grasp retraction position")
                self.waiting_for_reach = False
                self.pick_success = self.arm.gripper_object_detected()
                logger.info(f"Grasp sequence completed")
                if self.pick_success:
                    logger.info("Object successfully grasped!")
                else:
                    logger.warning("No object detected in gripper")
                self.reset_to_idle()
        else:
            # Command retraction to pre-grasp
            logger.info("Retracting to pre-grasp position")
            self.arm.cmd_ee_pose(self.final_pregrasp_pose, line_mode=True)
            self.arm.close_gripper()
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()

    def capture_and_process(
        self,
    ) -> Tuple[
        Optional[np.ndarray], Optional[Detection3DArray], Optional[Detection2DArray], Optional[Pose]
    ]:
        """
        Capture frame from camera and process detections.

        Returns:
            Tuple of (rgb_image, detection_3d_array, detection_2d_array, camera_pose)
            Returns None values if capture fails
        """
        # Capture frame
        bgr, _, depth, _ = self.camera.capture_frame_with_pose()
        if bgr is None or depth is None:
            return None, None, None, None

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # Get EE pose and camera transform
        ee_pose = self.arm.get_ee_pose()
        ee_transform = pose_to_matrix(ee_pose)
        camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)
        camera_pose = matrix_to_pose(camera_transform)

        # Process detections
        detection_3d_array, detection_2d_array = self.detector.process_frame(
            rgb, depth, camera_transform
        )

        return rgb, detection_3d_array, detection_2d_array, camera_pose

    def pick_target(self, x: int, y: int) -> bool:
        """
        Select a target object at the given pixel coordinates.

        Args:
            x: X coordinate in image
            y: Y coordinate in image

        Returns:
            True if a target was successfully selected
        """
        if not self.last_detection_2d_array or not self.last_detection_3d_array:
            logger.warning("No detections available for target selection")
            return False

        clicked_3d = find_clicked_detection(
            (x, y), self.last_detection_2d_array.detections, self.last_detection_3d_array.detections
        )
        if clicked_3d:
            self.pbvs.set_target(clicked_3d)
            logger.info(
                f"Target selected: ID={clicked_3d.id}, pos=({clicked_3d.bbox.center.position.x:.3f}, {clicked_3d.bbox.center.position.y:.3f}, {clicked_3d.bbox.center.position.z:.3f})"
            )
            self.grasp_stage = GraspStage.PRE_GRASP  # Transition from IDLE to PRE_GRASP
            self.reached_poses.clear()  # Clear pose history
            self.adjustment_count = 0  # Reset adjustment counter
            self.waiting_for_reach = False  # Ensure we're not stuck in waiting state
            self.last_commanded_pose = None
            self.stabilization_start_time = time.time()  # Start the timeout timer
            return True
        return False

    def update(self) -> Optional[Feedback]:
        """
        Main update function that handles capture, processing, control, and visualization.

        Returns:
            Feedback object with current state information, or None if capture failed
        """
        # Capture and process frame
        rgb, detection_3d_array, detection_2d_array, camera_pose = self.capture_and_process()
        if rgb is None:
            return None

        # Store for target selection
        self.last_detection_3d_array = detection_3d_array
        self.last_detection_2d_array = detection_2d_array

        # Update tracking if we have detections and not in IDLE or CLOSE_AND_RETRACT
        # Only update if not waiting for reach (to ensure fresh updates after reaching)
        if (
            detection_3d_array
            and self.grasp_stage in [GraspStage.PRE_GRASP, GraspStage.GRASP]
            and not self.waiting_for_reach
        ):
            self._update_tracking(detection_3d_array)

        # Execute stage-specific logic
        stage_handlers = {
            GraspStage.IDLE: self.execute_idle,
            GraspStage.PRE_GRASP: self.execute_pre_grasp,
            GraspStage.GRASP: self.execute_grasp,
            GraspStage.CLOSE_AND_RETRACT: self.execute_close_and_retract,
        }
        if self.grasp_stage in stage_handlers:
            stage_handlers[self.grasp_stage]()

        # Get tracking status
        target_tracked = self.pbvs.get_current_target() is not None

        # Create feedback
        ee_pose = self.arm.get_ee_pose()
        feedback = Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            last_commanded_pose=self.last_commanded_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=camera_pose,
            target_pose=self.pbvs.target_grasp_pose,
            waiting_for_reach=self.waiting_for_reach,
            grasp_successful=self.pick_success,
        )

        # Create simple visualization using feedback
        self.current_visualization = create_manipulation_visualization(
            rgb, feedback, detection_3d_array, detection_2d_array
        )

        return feedback

    def get_visualization(self) -> Optional[np.ndarray]:
        """
        Get the current visualization image.

        Returns:
            BGR image with visualizations, or None if no visualization available
        """
        return self.current_visualization

    def handle_keyboard_command(self, key: int) -> str:
        """
        Handle keyboard commands for robot control.

        Args:
            key: Keyboard key code

        Returns:
            Action taken as string, or empty string if no action
        """
        if key == ord("r"):
            self.reset_to_idle()
            return "reset"
        elif key == ord("s"):
            print("SOFT STOP - Emergency stopping robot!")
            self.arm.softStop()
            return "stop"
        elif key == ord(" ") and self.pbvs.target_grasp_pose:
            # Manual override - immediately transition to GRASP if in PRE_GRASP
            if self.grasp_stage == GraspStage.PRE_GRASP:
                self.set_grasp_stage(GraspStage.GRASP)
            print("Executing target pose")
            return "execute"
        elif key == 82:  # Up arrow - increase pitch
            new_pitch = min(90.0, self.grasp_pitch_degrees + 15.0)
            self.set_grasp_pitch(new_pitch)
            print(f"Grasp pitch: {new_pitch:.0f} degrees")
            return "pitch_up"
        elif key == 84:  # Down arrow - decrease pitch
            new_pitch = max(0.0, self.grasp_pitch_degrees - 15.0)
            self.set_grasp_pitch(new_pitch)
            print(f"Grasp pitch: {new_pitch:.0f} degrees")
            return "pitch_down"
        elif key == ord("g"):
            print("Opening gripper")
            self.arm.release_gripper()
            return "release"

        return ""

    def check_target_stabilized(self) -> bool:
        """
        Check if the commanded poses have stabilized.

        Returns:
            True if poses are stable, False otherwise
        """
        if len(self.reached_poses) < self.reached_poses.maxlen:
            return False  # Not enough poses yet

        # Extract positions
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.reached_poses]
        )

        # Calculate standard deviation for each axis
        std_devs = np.std(positions, axis=0)

        # Check if all axes are below threshold
        return np.all(std_devs < self.pose_stabilization_threshold)
