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
    bbox2d_to_corners,
)
from dimos.manipulation.visual_servoing.utils import (
    match_detection_by_id,
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
        pose_count: int = 0,
        max_poses: int = 0,
        stabilization_time: float = 0.0,
        grasp_successful: Optional[bool] = None,
        adjustment_count: int = 0,
    ):
        self.grasp_stage = grasp_stage
        self.target_tracked = target_tracked
        self.last_commanded_pose = last_commanded_pose
        self.current_ee_pose = current_ee_pose
        self.current_camera_pose = current_camera_pose
        self.target_pose = target_pose
        self.waiting_for_reach = waiting_for_reach
        self.pose_count = pose_count
        self.max_poses = max_poses
        self.stabilization_time = stabilization_time
        self.grasp_successful = grasp_successful
        self.adjustment_count = adjustment_count


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
        self.last_grasp_successful = None  # True if last grasp was successful
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
        self.last_grasp_successful = None
        self.final_pregrasp_pose = None

        self.arm.gotoObserve()

    def execute_idle(self):
        """Execute idle stage: just visualization, no control."""
        # Nothing to do in idle
        pass

    def execute_pre_grasp(self, detection_3d_array: Detection3DArray):
        """Execute pre-grasp stage: visual servoing to pre-grasp position."""
        # Get EE pose
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

        # Check timeout
        if (
            self.stabilization_start_time
            and (time.time() - self.stabilization_start_time) > self.stabilization_timeout
        ):
            logger.warning(
                f"Failed to get stable grasp after {self.stabilization_timeout} seconds, resetting"
            )

            self.reset_to_idle()
            return

        # Update tracking with new detections
        target_tracked = False
        if detection_3d_array:
            target_tracked = self.pbvs.update_tracking(detection_3d_array)
            if target_tracked:
                self.target_updated = True
                self.last_valid_target = self.pbvs.get_current_target()

        # PBVS control with pre-grasp distance
        _, _, _, has_target, target_pose = self.pbvs.compute_control(
            ee_pose, self.pregrasp_distance
        )

        # Handle pose control
        if target_pose and has_target:
            # Check if we have enough reached poses and they're stable
            if self.check_target_stabilized():
                logger.info("Target stabilized, transitioning to GRASP")
                # Store the final pre-grasp pose for retraction
                self.final_pregrasp_pose = self.last_commanded_pose
                self.grasp_stage = GraspStage.GRASP
                self.adjustment_count = 0
                self.waiting_for_reach = False
            elif not self.waiting_for_reach and self.target_updated:
                # Command the pose only if target has been updated
                self.arm.cmd_ee_pose(target_pose)
                self.last_commanded_pose = target_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()  # Start timeout timer
                self.target_updated = False  # Reset flag after commanding
                self.adjustment_count += 1

                # Sleep for 200ms after commanding to avoid rapid commands
                time.sleep(0.2)

    def execute_grasp(self, detection_3d_array: Detection3DArray):
        """Execute grasp stage: move to final grasp position."""
        # Get EE pose
        ee_pose = self.arm.get_ee_pose()

        # Check if waiting for robot to reach grasp pose
        if self.waiting_for_reach:
            # Check for timeout
            if self._check_reach_timeout():
                return

            reached = self.pbvs.is_target_reached(ee_pose)

            if reached and not self.grasp_reached_time:
                # First time reaching grasp pose
                self.grasp_reached_time = time.time()
                self.waiting_start_time = None  # Reset timeout timer
                # Robot reached grasp pose

            # Wait for delay then transition to CLOSE_AND_RETRACT
            if (
                self.grasp_reached_time
                and (time.time() - self.grasp_reached_time) >= self.grasp_close_delay
            ):
                logger.info("Grasp delay completed, closing gripper")
                self.grasp_stage = GraspStage.CLOSE_AND_RETRACT
                self.waiting_for_reach = False

            # While waiting, don't process new commands
            return

        # Only command grasp if not waiting and have valid target
        if self.last_valid_target:
            # Update tracking with new detections
            if detection_3d_array:
                self.pbvs.update_tracking(detection_3d_array)

            # Calculate grasp distance based on pitch angle
            # Maps from -grasp_distance_range to +grasp_distance_range based on pitch
            # 0° pitch (level grasp) -> -5cm (move 5cm closer to object)
            # 90° pitch (top-down) -> +5cm (stay 5cm farther from object)
            normalized_pitch = self.grasp_pitch_degrees / 90.0  # 0.0 to 1.0
            grasp_distance = -self.grasp_distance_range + (
                2 * self.grasp_distance_range * normalized_pitch
            )

            # PBVS control with calculated grasp distance
            _, _, _, has_target, target_pose = self.pbvs.compute_control(ee_pose, grasp_distance)

            if target_pose and has_target:
                # Get object size and calculate gripper opening
                object_size = self.last_valid_target.bbox.size
                object_width = object_size.x
                gripper_opening = object_width + self.grasp_width_offset
                gripper_opening = max(0.005, min(gripper_opening, self.gripper_max_opening))

                logger.info(f"Executing grasp: gripper={gripper_opening * 1000:.1f}mm")

                # Command gripper to open and move to grasp pose
                self.arm.cmd_gripper_ctrl(gripper_opening)
                self.arm.cmd_ee_pose(target_pose, line_mode=True)
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()  # Start timeout timer

    def execute_close_and_retract(self):
        """Execute the close and retract sequence."""
        # Close gripper and check if object is grasped (only once)
        if self.last_grasp_successful is None:
            _, object_present = self.arm.close_gripper()
            self.last_grasp_successful = object_present

            if object_present:
                logger.info("Object successfully grasped!")
            else:
                logger.warning("No object detected in gripper")

        # Get EE pose
        ee_pose = self.arm.get_ee_pose()

        # Check if waiting for robot to reach retraction pose
        if self.waiting_for_reach:
            # Check for timeout
            if self._check_reach_timeout():
                return

            # Temporarily set PBVS target to check if reached
            original_target = self.pbvs.target_grasp_pose
            self.pbvs.target_grasp_pose = self.final_pregrasp_pose
            reached = self.pbvs.is_target_reached(ee_pose)
            self.pbvs.target_grasp_pose = original_target

            if reached:
                logger.info("Reached pre-grasp retraction position")
                self.waiting_for_reach = False
                logger.info(
                    f"Grasp sequence completed (object_grasped={self.last_grasp_successful})"
                )
                self.reset_to_idle()

        else:
            # Command retraction to pre-grasp
            logger.info("Retracting to pre-grasp position")
            self.arm.cmd_ee_pose(self.final_pregrasp_pose, line_mode=True)
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

        # Execute stage-specific logic
        if self.grasp_stage == GraspStage.IDLE:
            self.execute_idle()
        elif self.grasp_stage == GraspStage.PRE_GRASP:
            if detection_3d_array:
                self.execute_pre_grasp(detection_3d_array)
        elif self.grasp_stage == GraspStage.GRASP:
            if detection_3d_array:
                self.execute_grasp(detection_3d_array)
        elif self.grasp_stage == GraspStage.CLOSE_AND_RETRACT:
            self.execute_close_and_retract()

        # Update tracking status from PBVS
        target_tracked = self.pbvs.get_current_target() is not None

        # Create visualization
        if self.waiting_for_reach:
            self.current_visualization = self._create_waiting_visualization(rgb)
        elif detection_3d_array and detection_2d_array and camera_pose:
            self.current_visualization = self.create_visualization(
                rgb, detection_3d_array, detection_2d_array, camera_pose, target_tracked
            )
        else:
            # Basic visualization with just the RGB image
            self.current_visualization = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Get current EE pose
        ee_pose = self.arm.get_ee_pose()

        # Calculate stabilization time if applicable
        stabilization_time = 0.0
        if self.stabilization_start_time:
            stabilization_time = time.time() - self.stabilization_start_time

        # Get target pose from PBVS if available
        target_pose = None
        if self.pbvs.target_grasp_pose:
            target_pose = self.pbvs.target_grasp_pose

        # Create and return feedback
        return Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            last_commanded_pose=self.last_commanded_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=camera_pose,
            target_pose=target_pose,
            waiting_for_reach=self.waiting_for_reach,
            pose_count=len(self.reached_poses),
            max_poses=self.pose_history_size,
            stabilization_time=stabilization_time,
            grasp_successful=self.last_grasp_successful,
            adjustment_count=self.adjustment_count,
        )

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

    def create_visualization(
        self,
        rgb: np.ndarray,
        detection_3d_array: Detection3DArray,
        detection_2d_array: Detection2DArray,
        camera_pose: Pose,
        target_tracked: bool,
    ) -> np.ndarray:
        """
        Create visualization with detections and status overlays.

        Args:
            rgb: RGB image
            detection_3d_array: 3D detections
            detection_2d_array: 2D detections
            camera_pose: Current camera pose
            target_tracked: Whether target is being tracked

        Returns:
            BGR image with visualizations
        """
        # Create visualization with position overlays
        viz = self.detector.visualize_detections(
            rgb, detection_3d_array.detections, detection_2d_array.detections
        )

        # Add PBVS status overlay
        viz = self.pbvs.create_status_overlay(viz, self.grasp_stage)

        # Highlight target
        current_target = self.pbvs.get_current_target()
        if target_tracked and current_target:
            det_2d = match_detection_by_id(
                current_target, detection_3d_array.detections, detection_2d_array.detections
            )
            if det_2d and det_2d.bbox:
                x1, y1, x2, y2 = bbox2d_to_corners(det_2d.bbox)
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(
                    viz, "TARGET", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

        # Convert back to BGR for OpenCV display
        viz_bgr = cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)

        # Add pose info
        cv2.putText(
            viz_bgr,
            "Eye-in-Hand Visual Servoing",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )

        # Get EE pose for display
        ee_pose = self.arm.get_ee_pose()

        camera_text = f"Camera: ({camera_pose.position.x:.2f}, {camera_pose.position.y:.2f}, {camera_pose.position.z:.2f})m"
        cv2.putText(viz_bgr, camera_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        ee_text = (
            f"EE: ({ee_pose.position.x:.2f}, {ee_pose.position.y:.2f}, {ee_pose.position.z:.2f})m"
        )
        cv2.putText(viz_bgr, ee_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Add control status
        status_text, status_color = self._get_status_text_and_color()
        cv2.putText(viz_bgr, status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        cv2.putText(
            viz_bgr,
            "s=STOP | r=RESET | SPACE=FORCE GRASP | g=RELEASE",
            (10, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
        )

        return viz_bgr

    def _create_waiting_visualization(self, rgb: np.ndarray) -> np.ndarray:
        """
        Create a simple visualization while waiting for robot to reach pose.

        Args:
            rgb: RGB image

        Returns:
            BGR image with waiting status
        """
        viz_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Add waiting status
        cv2.putText(
            viz_bgr,
            "WAITING FOR ROBOT TO REACH TARGET...",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        # Add current stage info
        stage_text = f"Stage: {self.grasp_stage.value.upper()}"
        cv2.putText(
            viz_bgr,
            stage_text,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
        )

        # Add progress info based on stage
        if self.grasp_stage == GraspStage.PRE_GRASP:
            progress_text = f"Reached poses: {len(self.reached_poses)}/{self.pose_history_size}"
        elif self.grasp_stage == GraspStage.GRASP and self.grasp_reached_time:
            time_remaining = max(
                0, self.grasp_close_delay - (time.time() - self.grasp_reached_time)
            )
            progress_text = f"Closing gripper in: {time_remaining:.1f}s"
        else:
            progress_text = ""

        if progress_text:
            cv2.putText(
                viz_bgr,
                progress_text,
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

        return viz_bgr

    def _get_status_text_and_color(self) -> Tuple[str, Tuple[int, int, int]]:
        """
        Get status text and color based on current stage and state.

        Returns:
            Tuple of (status_text, status_color)
        """
        if self.grasp_stage == GraspStage.IDLE:
            return "IDLE - Click object to select target", (100, 100, 100)
        elif self.grasp_stage == GraspStage.PRE_GRASP:
            if self.waiting_for_reach:
                return "PRE-GRASP - Waiting for robot to reach target...", (255, 255, 0)
            else:
                poses_text = f" ({len(self.reached_poses)}/{self.pose_history_size} poses)"
                elapsed_time = (
                    time.time() - self.stabilization_start_time
                    if self.stabilization_start_time
                    else 0
                )
                time_text = f" [{elapsed_time:.1f}s/{self.stabilization_timeout:.0f}s]"
                return f"PRE-GRASP - Collecting stable poses{poses_text}{time_text}", (0, 255, 255)
        elif self.grasp_stage == GraspStage.GRASP:
            if self.grasp_reached_time:
                time_remaining = self.grasp_close_delay - (time.time() - self.grasp_reached_time)
                return f"GRASP - Waiting to close ({time_remaining:.1f}s)", (0, 255, 0)
            else:
                return "GRASP - Moving to grasp pose", (0, 255, 0)
        else:  # CLOSE_AND_RETRACT
            return "CLOSE_AND_RETRACT - Closing gripper and retracting", (255, 0, 255)

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

    def pick_and_place(
        self, object_point: Tuple[int, int], target_point: Optional[Tuple[int, int]] = None
    ) -> bool:
        """
        Execute a complete pick and place operation.

        Similar to navigate_path_local, this function handles the complete pick operation
        autonomously, including object selection, grasping, and optional placement.

        Args:
            object_point: (x, y) pixel coordinates of the object to pick
            target_point: Optional (x, y) pixel coordinates for placement (not implemented yet)

        Returns:
            True if object was successfully picked, False otherwise
        """
        # Validate input
        if not isinstance(object_point, tuple) or len(object_point) != 2:
            logger.error(f"Invalid object_point: {object_point}. Expected (x, y) tuple.")
            return False

        logger.info(f"Starting pick operation at pixel ({object_point[0]}, {object_point[1]})")

        # Reset to ensure clean state
        self.reset_to_idle()

        # Configuration
        max_operation_time = 60.0  # Maximum time for complete pick operation
        perception_init_time = 2.0  # Time to allow perception to stabilize

        # Wait for perception to initialize
        init_start = time.time()
        perception_ready = False

        while (time.time() - init_start) < perception_init_time:
            feedback = self.update()
            if feedback is not None:
                perception_ready = True
            time.sleep(0.1)

        if not perception_ready:
            logger.error("Perception system failed to initialize")
            return False

        # Select the target object
        x, y = object_point
        try:
            if not self.pick_target(x, y):
                logger.error(f"No valid object detected at pixel ({x}, {y})")
                return False
        except Exception as e:
            logger.error(f"Exception during target selection: {e}")
            return False

        # Execute pick operation
        operation_start = time.time()

        while (time.time() - operation_start) < max_operation_time:
            try:
                # Update the manipulation system
                feedback = self.update()
                if feedback is None:
                    logger.error("Lost perception during pick operation")
                    self.reset_to_idle()
                    return False

                # Check if grasp sequence completed
                if feedback.grasp_successful is not None:
                    if feedback.grasp_successful:
                        logger.info("Object successfully grasped")
                        if target_point:
                            logger.info("Place operation not yet implemented")
                        return True
                    else:
                        logger.warning("Grasp attempt failed - no object detected in gripper")
                        return False

            except Exception as e:
                logger.error(f"Unexpected error during pick operation: {e}")
                self.reset_to_idle()
                return False

        # Operation timeout
        logger.error(f"Pick operation exceeded maximum time of {max_operation_time}s")
        self.reset_to_idle()
        return False

    def cleanup(self):
        """Clean up resources (detector only, hardware cleanup is caller's responsibility)."""
        self.detector.cleanup()
