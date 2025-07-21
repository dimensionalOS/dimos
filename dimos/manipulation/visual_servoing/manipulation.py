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
    CLOSE_AND_LIFT = "close_and_lift"  # Close gripper and lift


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
        camera_intrinsics: list,  # [fx, fy, cx, cy]
        direct_ee_control: bool = True,
        ee_to_camera_6dof: Optional[list] = None,
    ):
        """
        Initialize manipulation system.

        Args:
            camera: Camera object with capture_frame_with_pose() method
            arm: Robot arm object with get_ee_pose(), cmd_ee_pose(), cmd_vel_ee(),
                 cmd_gripper_ctrl(), release_gripper(), softStop(), gotoZero(), and disable() methods
            camera_intrinsics: Camera intrinsics [fx, fy, cx, cy]
            direct_ee_control: If True, use direct EE pose control; if False, use velocity control
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
        """
        self.camera = camera
        self.arm = arm
        self.direct_ee_control = direct_ee_control

        # Default EE to camera transform if not provided
        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [-0.06, 0.03, -0.05, 0.0, -1.57, 0.0]

        # Create transform matrices
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        # Initialize processors
        self.detector = Detection3DProcessor(camera_intrinsics)
        self.pbvs = PBVS(
            position_gain=0.3,
            rotation_gain=0.2,
            target_tolerance=0.05,
            direct_ee_control=direct_ee_control,
        )

        # Control state
        self.last_valid_target = None
        self.waiting_for_reach = False  # True when waiting for robot to reach commanded pose
        self.last_commanded_pose = None  # Last pose sent to robot
        self.target_updated = False  # True when target has been updated with fresh detections

        # Grasp parameters
        self.grasp_width_offset = 0.03  # Default grasp width offset
        self.grasp_pitch_degrees = 30.0  # Default grasp pitch in degrees
        self.pregrasp_distance = 0.3  # Distance to maintain before grasping (m)
        self.grasp_distance = 0.01  # Distance for final grasp approach (m)
        self.grasp_close_delay = 3.0  # Time to wait at grasp pose before closing (seconds)
        self.grasp_reached_time = None  # Time when grasp pose was reached

        # Grasp stage tracking
        self.grasp_stage = GraspStage.IDLE

        # Pose stabilization tracking
        self.pose_history_size = 4  # Number of poses to check for stabilization
        self.pose_stabilization_threshold = 0.005  # 1cm threshold for stabilization
        self.stabilization_timeout = 10.0  # Timeout in seconds before giving up
        self.stabilization_start_time = None  # Time when stabilization started
        self.reached_poses = deque(
            maxlen=self.pose_history_size
        )  # Only stores poses that were reached
        self.adjustment_count = 0

        # State for visualization
        self.current_visualization = None
        self.last_rgb = None
        self.last_detection_3d_array = None
        self.last_detection_2d_array = None
        self.last_camera_pose = None
        self.last_target_tracked = False

        logger.info(
            f"Initialized Manipulation system in {'Direct EE' if direct_ee_control else 'Velocity'} control mode"
        )

    def set_grasp_stage(self, stage: GraspStage):
        """
        Set the grasp stage.

        Args:
            stage: The new grasp stage
        """
        self.grasp_stage = stage
        logger.info(f"Set grasp stage to: {stage.value}")

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
        logger.info(f"Set grasp pitch to: {pitch_degrees} degrees")

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

    def execute_pre_grasp(self, detection_3d_array: Detection3DArray) -> bool:
        """
        Execute pre-grasp stage: visual servoing to pre-grasp position.

        Args:
            detection_3d_array: Current 3D detections

        Returns:
            True if target is being tracked
        """
        # Get EE pose
        ee_pose = self.arm.get_ee_pose()

        # PBVS control with pre-grasp distance
        vel_cmd, ang_vel_cmd, _, target_tracked, target_pose = self.pbvs.compute_control(
            ee_pose, detection_3d_array, self.pregrasp_distance
        )

        if (
            self.stabilization_start_time
            and (time.time() - self.stabilization_start_time) > self.stabilization_timeout
        ):
            logger.warning(
                f"Failed to get stable grasp after {self.stabilization_timeout} seconds, resetting"
            )
            self.arm.gotoZero()
            time.sleep(1.0)
            self.reset_to_idle()
            return False

        # Set target_updated flag if target was successfully tracked
        if target_tracked and target_pose:
            self.target_updated = True
            self.last_valid_target = self.pbvs.get_current_target()

        # Handle direct EE control
        if self.direct_ee_control and target_pose and target_tracked:
            # Check if we have enough reached poses and they're stable
            if self.check_target_stabilized():
                logger.info("Target stabilized, transitioning to GRASP stage")
                self.grasp_stage = GraspStage.GRASP
                self.adjustment_count = 0
                self.waiting_for_reach = False
            elif not self.waiting_for_reach and self.target_updated:
                # Command the pose only if target has been updated
                self.arm.cmd_ee_pose(target_pose)
                self.last_commanded_pose = target_pose
                self.waiting_for_reach = True
                self.target_updated = False  # Reset flag after commanding
                self.adjustment_count += 1

                elapsed_time = (
                    time.time() - self.stabilization_start_time
                    if self.stabilization_start_time
                    else 0
                )
                logger.info(
                    f"Commanded target pose: pos=({target_pose.position.x:.3f}, "
                    f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f}), "
                    f"attempt {self.adjustment_count} (elapsed: {elapsed_time:.1f}s)"
                )

                # Sleep for 200ms after commanding to avoid rapid commands
                time.sleep(0.2)

        elif not self.direct_ee_control and vel_cmd and ang_vel_cmd:
            # Velocity control
            self.arm.cmd_vel_ee(
                vel_cmd.x, vel_cmd.y, vel_cmd.z, ang_vel_cmd.x, ang_vel_cmd.y, ang_vel_cmd.z
            )

        return target_tracked

    def execute_grasp(self, detection_3d_array: Detection3DArray) -> bool:
        """
        Execute grasp stage: move to final grasp position.

        Args:
            detection_3d_array: Current 3D detections

        Returns:
            True if target is being tracked
        """
        if not self.waiting_for_reach and self.last_valid_target:
            # Get EE pose
            ee_pose = self.arm.get_ee_pose()

            # PBVS control with grasp distance
            vel_cmd, ang_vel_cmd, _, target_tracked, target_pose = self.pbvs.compute_control(
                ee_pose, detection_3d_array, self.grasp_distance
            )

            if self.direct_ee_control and target_pose and target_tracked:
                # Get object size and calculate gripper opening
                object_size = self.last_valid_target.bbox.size
                object_width = object_size.x
                gripper_opening = object_width + self.grasp_width_offset
                gripper_opening = max(0.005, min(gripper_opening, 0.1))

                logger.info(f"Executing grasp: opening gripper to {gripper_opening * 1000:.1f}mm")
                print(f"Executing grasp: opening gripper to {gripper_opening * 1000:.1f}mm")

                # Command gripper to open and move to grasp pose
                self.arm.cmd_gripper_ctrl(gripper_opening)
                self.arm.cmd_ee_pose(target_pose, line_mode=True)
                self.waiting_for_reach = True
                logger.info("Grasp pose commanded")

                return target_tracked

        return False

    def execute_close_and_lift(self):
        """Execute the close and lift sequence."""
        logger.info("Executing CLOSE_AND_LIFT sequence")

        # Close gripper
        logger.info("Closing gripper")
        self.arm.cmd_gripper_ctrl(0.0)  # Close gripper completely
        time.sleep(0.5)  # Wait for gripper to close

        # Return to home position
        logger.info("Returning to home position")
        self.arm.gotoZero()

        # Reset to IDLE after completion
        logger.info("Grasp sequence completed, returning to IDLE")
        self.reset_to_idle()

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

        # Process
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # Get EE pose from robot (this serves as our odometry)
        ee_pose = self.arm.get_ee_pose()

        # Transform EE pose to camera pose
        ee_transform = pose_to_matrix(ee_pose)
        camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)
        camera_pose = matrix_to_pose(camera_transform)

        # Process detections using camera transform
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
            self.grasp_stage = GraspStage.PRE_GRASP  # Transition from IDLE to PRE_GRASP
            self.reached_poses.clear()  # Clear pose history
            self.adjustment_count = 0  # Reset adjustment counter
            self.waiting_for_reach = False  # Ensure we're not stuck in waiting state
            self.last_commanded_pose = None
            self.stabilization_start_time = time.time()  # Start the timeout timer
            logger.info(f"Target selected at ({x}, {y})")
            return True
        return False

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
        mode_text = "Direct EE" if self.direct_ee_control else "Velocity"
        cv2.putText(
            viz_bgr,
            f"Eye-in-Hand ({mode_text})",
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

        # Add control status for direct EE mode
        if self.direct_ee_control:
            if self.grasp_stage == GraspStage.IDLE:
                status_text = "IDLE - Click object to select target"
                status_color = (100, 100, 100)
            elif self.grasp_stage == GraspStage.PRE_GRASP:
                if self.waiting_for_reach:
                    status_text = "PRE-GRASP - Waiting for robot to reach target..."
                    status_color = (255, 255, 0)
                else:
                    poses_text = f" ({len(self.reached_poses)}/{self.pose_history_size} poses)"
                    elapsed_time = (
                        time.time() - self.stabilization_start_time
                        if self.stabilization_start_time
                        else 0
                    )
                    time_text = f" [{elapsed_time:.1f}s/{self.stabilization_timeout:.0f}s]"
                    status_text = f"PRE-GRASP - Collecting stable poses{poses_text}{time_text}"
                    status_color = (0, 255, 255)
            elif self.grasp_stage == GraspStage.GRASP:
                if self.grasp_reached_time:
                    time_remaining = self.grasp_close_delay - (
                        time.time() - self.grasp_reached_time
                    )
                    status_text = f"GRASP - Waiting to close ({time_remaining:.1f}s)"
                else:
                    status_text = "GRASP - Moving to grasp pose"
                status_color = (0, 255, 0)
            else:  # CLOSE_AND_LIFT
                status_text = "CLOSE_AND_LIFT - Closing gripper and lifting"
                status_color = (255, 0, 255)

            cv2.putText(
                viz_bgr, status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1
            )
            cv2.putText(
                viz_bgr,
                "s=STOP | h=HOME | SPACE=FORCE GRASP | g=RELEASE",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )

        return viz_bgr

    def update(self) -> bool:
        """
        Main update function that handles capture, processing, control, and visualization.

        Returns:
            True if update was successful, False if capture failed
        """
        # Always capture frame for visualization
        bgr, _, depth, _ = self.camera.capture_frame_with_pose()
        if bgr is None or depth is None:
            return False

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # If waiting for robot to reach target, check if reached
        if self.waiting_for_reach and self.last_commanded_pose:
            ee_pose = self.arm.get_ee_pose()

            if self.grasp_stage == GraspStage.GRASP:
                # Check if grasp pose is reached
                grasp_distance = self.grasp_distance
                reached = self.pbvs.is_target_reached(ee_pose, grasp_distance)

                if reached and not self.grasp_reached_time:
                    # First time reaching grasp pose
                    self.grasp_reached_time = time.time()
                    logger.info(
                        f"Robot reached grasp pose, waiting {self.grasp_close_delay}s before closing gripper"
                    )

                # Wait for delay then transition to CLOSE_AND_LIFT
                if (
                    self.grasp_reached_time
                    and (time.time() - self.grasp_reached_time) >= self.grasp_close_delay
                ):
                    logger.info(
                        f"Waited {self.grasp_close_delay}s at grasp pose, transitioning to CLOSE_AND_LIFT"
                    )
                    self.grasp_stage = GraspStage.CLOSE_AND_LIFT
                    self.waiting_for_reach = False
            else:
                # For PRE_GRASP stage, check if reached
                grasp_distance = (
                    self.pregrasp_distance
                    if self.grasp_stage == GraspStage.PRE_GRASP
                    else self.grasp_distance
                )
                reached = self.pbvs.is_target_reached(ee_pose, grasp_distance)

                if reached:
                    logger.info("Robot reached commanded pose")
                    self.waiting_for_reach = False
                    self.reached_poses.append(self.last_commanded_pose)
                    self.target_updated = False  # Reset flag so we wait for fresh update
                    time.sleep(0.3)

            # Create basic visualization while waiting
            self.current_visualization = self._create_waiting_visualization(rgb)
            return True

        # Normal processing when not waiting
        # Get EE pose and camera transform
        ee_pose = self.arm.get_ee_pose()
        ee_transform = pose_to_matrix(ee_pose)
        camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)
        camera_pose = matrix_to_pose(camera_transform)

        # Process detections
        detection_3d_array, detection_2d_array = self.detector.process_frame(
            rgb, depth, camera_transform
        )

        # Store for target selection
        self.last_rgb = rgb
        self.last_detection_3d_array = detection_3d_array
        self.last_detection_2d_array = detection_2d_array
        self.last_camera_pose = camera_pose

        # Execute stage-specific logic
        target_tracked = False

        if self.grasp_stage == GraspStage.IDLE:
            # Nothing to do in IDLE
            pass
        elif self.grasp_stage == GraspStage.PRE_GRASP:
            if detection_3d_array:
                target_tracked = self.execute_pre_grasp(detection_3d_array)
                self.last_target_tracked = target_tracked
        elif self.grasp_stage == GraspStage.GRASP:
            if detection_3d_array:
                target_tracked = self.execute_grasp(detection_3d_array)
                self.last_target_tracked = target_tracked
        elif self.grasp_stage == GraspStage.CLOSE_AND_LIFT:
            # No visual servoing needed for close and lift
            self.execute_close_and_lift()

        # Create full visualization
        if detection_3d_array and detection_2d_array and camera_pose:
            self.current_visualization = self.create_visualization(
                rgb, detection_3d_array, detection_2d_array, camera_pose, target_tracked
            )
        else:
            # Basic visualization with just the RGB image
            self.current_visualization = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        return True

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
        elif key == ord("h"):
            print("GO HOME - Returning to safe position...")
            self.arm.gotoZero()
            return "home"
        elif key == ord(" ") and self.direct_ee_control and self.pbvs.target_grasp_pose:
            # Manual override - immediately transition to GRASP if in PRE_GRASP
            if self.grasp_stage == GraspStage.PRE_GRASP:
                logger.info("Manual grasp execution requested")
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

    def cleanup(self):
        """Clean up resources (detector only, hardware cleanup is caller's responsibility)."""
        self.detector.cleanup()
        logger.info("Cleaned up manipulation system resources")
