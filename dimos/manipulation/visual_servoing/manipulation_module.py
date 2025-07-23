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
Manipulation module for robotic grasping with visual servoing.
Handles grasping logic, state machine, and hardware coordination as a Dimos module.
"""

import cv2
import time
import threading
from typing import Optional, Tuple, Any, Dict
from enum import Enum
from collections import deque

import numpy as np

from dimos.core import Module, In, Out, rpc
from dimos_lcm.sensor_msgs import Image, CameraInfo
from dimos_lcm.geometry_msgs import Vector3, Pose, Point, Quaternion
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray

from dimos.hardware.piper_arm import PiperArm
from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.perception.common.utils import find_clicked_detection
from dimos.manipulation.visual_servoing.utils import (
    create_manipulation_visualization,
    select_points_from_depth,
    transform_points_3d,
    update_target_grasp_pose,
    apply_grasp_distance,
    is_target_reached,
)
from dimos.utils.transform_utils import (
    pose_to_matrix,
    matrix_to_pose,
    create_transform_from_6dof,
    compose_transforms,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.manipulation_module")


class GraspStage(Enum):
    """Enum for different grasp stages."""

    IDLE = "idle"  # No target set
    PRE_GRASP = "pre_grasp"  # Target set, moving to pre-grasp position
    GRASP = "grasp"  # Executing final grasp
    CLOSE_AND_RETRACT = "close_and_retract"  # Close gripper and retract
    PLACE = "place"  # Move to place position and release object
    RETRACT = "retract"  # Retract from place position


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
        success: Optional[bool] = None,
    ):
        self.grasp_stage = grasp_stage
        self.target_tracked = target_tracked
        self.last_commanded_pose = last_commanded_pose
        self.current_ee_pose = current_ee_pose
        self.current_camera_pose = current_camera_pose
        self.target_pose = target_pose
        self.waiting_for_reach = waiting_for_reach
        self.success = success


class ManipulationModule(Module):
    """
    Manipulation module for visual servoing and grasping.

    Subscribes to:
        - ZED RGB images
        - ZED depth images
        - ZED camera info

    Publishes:
        - Visualization images

    RPC methods:
        - handle_keyboard_command: Process keyboard input
        - pick_and_place: Execute pick and place task
    """

    # LCM inputs
    rgb_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    # LCM outputs
    viz_image: Out[Image] = None

    def __init__(
        self,
        ee_to_camera_6dof: Optional[list] = None,
        **kwargs,
    ):
        """
        Initialize manipulation module.

        Args:
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
        """
        super().__init__(**kwargs)

        # Initialize arm directly
        self.arm = PiperArm()

        # Default EE to camera transform if not provided
        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [-0.065, 0.03, -0.105, 0.0, -1.57, 0.0]

        # Create transform matrices
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        # Camera intrinsics will be set when camera info is received
        self.camera_intrinsics = None
        self.detector = None
        self.pbvs = None

        # Control state
        self.last_valid_target = None
        self.waiting_for_reach = False
        self.last_commanded_pose = None
        self.target_updated = False
        self.waiting_start_time = None
        self.reach_pose_timeout = 10.0

        # Grasp parameters
        self.grasp_width_offset = 0.03
        self.grasp_pitch_degrees = 30.0
        self.pregrasp_distance = 0.25
        self.grasp_distance_range = 0.03
        self.grasp_close_delay = 2.0
        self.grasp_reached_time = None
        self.gripper_max_opening = 0.07

        # Grasp stage tracking
        self.grasp_stage = GraspStage.IDLE

        # Pose stabilization tracking
        self.pose_history_size = 4
        self.pose_stabilization_threshold = 0.01
        self.stabilization_timeout = 15.0
        self.stabilization_start_time = None
        self.reached_poses = deque(maxlen=self.pose_history_size)
        self.adjustment_count = 0

        # State for visualization
        self.current_visualization = None
        self.last_detection_3d_array = None
        self.last_detection_2d_array = None

        # Grasp result and task tracking
        self.pick_success = None
        self.final_pregrasp_pose = None
        self.task_failed = False  # New variable for tracking task failure
        self.overall_success = None  # Track overall pick and place success

        # Task control
        self.task_running = False
        self.task_thread = None
        self.stop_event = threading.Event()

        # Latest sensor data
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None

        # Target selection
        self.target_click = None

        # Place target position and object info
        self.place_target_position = None
        self.target_object_height = None
        self.place_pose = None  # Store the calculated place pose for retraction

        # Move arm to observe position on init
        self.arm.gotoObserve()

    @rpc
    def start(self):
        """Start the manipulation module."""
        # Subscribe to camera data
        self.rgb_image.subscribe(self._on_rgb_image)
        self.depth_image.subscribe(self._on_depth_image)
        self.camera_info.subscribe(self._on_camera_info)

        logger.info("Manipulation module started")

    @rpc
    def stop(self):
        """Stop the manipulation module."""
        # Stop any running task
        self.stop_event.set()
        if self.task_thread and self.task_thread.is_alive():
            self.task_thread.join(timeout=5.0)

        # Disable arm
        self.arm.disable()
        logger.info("Manipulation module stopped")

    def _on_rgb_image(self, msg: Image):
        """Handle RGB image messages."""
        try:
            # Convert LCM message to numpy array
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "rgb8":
                self.latest_rgb = data.reshape((msg.height, msg.width, 3))
            else:
                logger.warning(f"Unsupported RGB encoding: {msg.encoding}")
        except Exception as e:
            logger.error(f"Error processing RGB image: {e}")

    def _on_depth_image(self, msg: Image):
        """Handle depth image messages."""
        try:
            # Convert LCM message to numpy array
            if msg.encoding == "32FC1":
                data = np.frombuffer(msg.data, dtype=np.float32)
                self.latest_depth = data.reshape((msg.height, msg.width))
            else:
                logger.warning(f"Unsupported depth encoding: {msg.encoding}")
        except Exception as e:
            logger.error(f"Error processing depth image: {e}")

    def _on_camera_info(self, msg: CameraInfo):
        """Handle camera info messages."""
        try:
            # Extract camera intrinsics
            self.camera_intrinsics = [
                msg.K[0],  # fx
                msg.K[4],  # fy
                msg.K[2],  # cx
                msg.K[5],  # cy
            ]

            # Initialize processors if not already done
            if self.detector is None:
                self.detector = Detection3DProcessor(self.camera_intrinsics)
                self.pbvs = PBVS(target_tolerance=0.05)
                logger.info("Initialized detection and PBVS processors")

            self.latest_camera_info = msg
        except Exception as e:
            logger.error(f"Error processing camera info: {e}")

    @rpc
    def handle_keyboard_command(self, key: str) -> str:
        """
        Handle keyboard commands for robot control.

        Args:
            key: Keyboard key as string

        Returns:
            Action taken as string, or empty string if no action
        """
        key_code = ord(key) if len(key) == 1 else int(key)

        if key_code == ord("r"):
            self.stop_event.set()
            self.task_running = False
            self.reset_to_idle()
            return "reset"
        elif key_code == ord("s"):
            logger.info("SOFT STOP - Emergency stopping robot!")
            self.arm.softStop()
            self.stop_event.set()
            self.task_running = False
            return "stop"
        elif key_code == ord(" ") and self.pbvs and self.pbvs.target_grasp_pose:
            # Manual override - immediately transition to GRASP if in PRE_GRASP
            if self.grasp_stage == GraspStage.PRE_GRASP:
                self.set_grasp_stage(GraspStage.GRASP)
            logger.info("Executing target pose")
            return "execute"
        elif key_code == 82:  # Up arrow - increase pitch
            new_pitch = min(90.0, self.grasp_pitch_degrees + 15.0)
            self.set_grasp_pitch(new_pitch)
            logger.info(f"Grasp pitch: {new_pitch:.0f} degrees")
            return "pitch_up"
        elif key_code == 84:  # Down arrow - decrease pitch
            new_pitch = max(0.0, self.grasp_pitch_degrees - 15.0)
            self.set_grasp_pitch(new_pitch)
            logger.info(f"Grasp pitch: {new_pitch:.0f} degrees")
            return "pitch_down"
        elif key_code == ord("g"):
            logger.info("Opening gripper")
            self.arm.release_gripper()
            return "release"

        return ""

    @rpc
    def pick_and_place(
        self, target_x: int = None, target_y: int = None, place_x: int = None, place_y: int = None
    ) -> Dict[str, Any]:
        """
        Start a pick and place task.

        Args:
            target_x: Optional X coordinate of target object
            target_y: Optional Y coordinate of target object
            place_x: Optional X coordinate of place location
            place_y: Optional Y coordinate of place location

        Returns:
            Dict with status and message
        """
        if self.task_running:
            return {"status": "error", "message": "Task already running"}

        if self.camera_intrinsics is None:
            return {"status": "error", "message": "Camera not initialized"}

        # Set target if coordinates provided
        if target_x is not None and target_y is not None:
            self.target_click = (target_x, target_y)

        # Process place location if provided
        if place_x is not None and self.latest_depth is not None:
            # Select points around the place location from depth image
            points_3d_camera = select_points_from_depth(
                self.latest_depth,
                (place_x, place_y),
                self.camera_intrinsics,
                radius=10,  # 10 pixel radius around place point
            )

            if points_3d_camera.size > 0:
                # Get current camera transform to transform points to world frame
                ee_pose = self.arm.get_ee_pose()
                ee_transform = pose_to_matrix(ee_pose)
                camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)

                # Transform points from camera frame to world frame
                points_3d_world = transform_points_3d(
                    points_3d_camera,
                    camera_transform,
                    to_robot=True,  # Convert from optical to robot frame
                )

                # Average the 3D points to get place position
                place_position = np.mean(points_3d_world, axis=0)

                # Create place target pose with same orientation as current EE
                # For now, just store the position - full implementation will come later
                self.place_target_position = place_position
                logger.info(
                    f"Place target set at position: ({place_position[0]:.3f}, {place_position[1]:.3f}, {place_position[2]:.3f})"
                )
                logger.info("Note: Z-offset will be applied once target object is detected")
            else:
                logger.warning("No valid depth points found at place location")
                self.place_target_position = None
        else:
            self.place_target_position = None

        # Reset task state
        self.task_failed = False
        self.stop_event.clear()

        # Ensure any previous thread has finished
        if self.task_thread and self.task_thread.is_alive():
            self.stop_event.set()
            self.task_thread.join(timeout=1.0)

        # Start task in separate thread
        self.task_thread = threading.Thread(target=self._run_pick_and_place, daemon=True)
        self.task_thread.start()

        return {"status": "started", "message": "Pick and place task started"}

    def _run_pick_and_place(self):
        """Run the pick and place task loop."""
        self.task_running = True
        logger.info("Starting pick and place task")

        try:
            while not self.stop_event.is_set():
                # Check for task failure
                if self.task_failed:
                    logger.error("Task failed, terminating pick and place")
                    self.stop_event.set()
                    break

                # Update manipulation system
                feedback = self.update()
                if feedback is None:
                    time.sleep(0.01)
                    continue

                # Check if task is complete
                if feedback.success is not None:
                    if feedback.success:
                        logger.info("Pick and place completed successfully!")
                    else:
                        logger.warning("Pick and place failed - no object detected")
                    # Reset to idle state and stop the event loop
                    self.reset_to_idle()
                    self.stop_event.set()
                    break

                # Small delay to prevent CPU overload
                time.sleep(0.01)

        except Exception as e:
            logger.error(f"Error in pick and place task: {e}")
            self.task_failed = True
        finally:
            self.task_running = False
            logger.info("Pick and place task ended")

    def set_grasp_stage(self, stage: GraspStage):
        """Set the grasp stage."""
        self.grasp_stage = stage
        logger.info(f"Grasp stage: {stage.value}")

    def set_grasp_pitch(self, pitch_degrees: float):
        """Set the grasp pitch angle."""
        pitch_degrees = max(0.0, min(90.0, pitch_degrees))
        self.grasp_pitch_degrees = pitch_degrees
        if self.pbvs:
            self.pbvs.set_grasp_pitch(pitch_degrees)

    def _check_reach_timeout(self) -> bool:
        """Check if robot has exceeded timeout while reaching pose."""
        if (
            self.waiting_start_time
            and (time.time() - self.waiting_start_time) > self.reach_pose_timeout
        ):
            logger.warning(f"Robot failed to reach pose within {self.reach_pose_timeout}s timeout")
            self.task_failed = True
            self.reset_to_idle()
            return True
        return False

    def _update_tracking(self, detection_3d_array: Optional[Detection3DArray]) -> bool:
        """Update tracking with new detections."""
        if not detection_3d_array or not self.pbvs:
            return False

        target_tracked = self.pbvs.update_tracking(detection_3d_array)
        if target_tracked:
            self.target_updated = True
            self.last_valid_target = self.pbvs.get_current_target()
        return target_tracked

    def reset_to_idle(self):
        """Reset the manipulation system to IDLE state."""
        if self.pbvs:
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
        self.overall_success = None
        self.place_pose = None

        self.arm.gotoObserve()

    def execute_idle(self):
        """Execute idle stage: just visualization, no control."""
        pass

    def execute_pre_grasp(self):
        """Execute pre-grasp stage: visual servoing to pre-grasp position."""
        ee_pose = self.arm.get_ee_pose()

        # Check if waiting for robot to reach commanded pose
        if self.waiting_for_reach and self.last_commanded_pose:
            # Check for timeout
            if self._check_reach_timeout():
                return

            reached = is_target_reached(
                self.last_commanded_pose, ee_pose, self.pbvs.target_tolerance
            )

            if reached:
                self.waiting_for_reach = False
                self.waiting_start_time = None
                self.reached_poses.append(self.last_commanded_pose)
                self.target_updated = False
                time.sleep(0.3)

            return

        # Check stabilization timeout
        if (
            self.stabilization_start_time
            and (time.time() - self.stabilization_start_time) > self.stabilization_timeout
        ):
            logger.warning(
                f"Failed to get stable grasp after {self.stabilization_timeout} seconds, resetting"
            )
            self.task_failed = True
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

            if (
                is_target_reached(self.pbvs.target_grasp_pose, ee_pose, self.pbvs.target_tolerance)
                and not self.grasp_reached_time
            ):
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
            # Calculate grasp distance based on pitch angle
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
            reached = is_target_reached(
                self.final_pregrasp_pose, ee_pose, self.pbvs.target_tolerance
            )

            if reached:
                logger.info("Reached pre-grasp retraction position")
                self.waiting_for_reach = False
                self.pick_success = self.arm.gripper_object_detected()
                logger.info(f"Grasp sequence completed")
                if self.pick_success:
                    logger.info("Object successfully grasped!")
                    # Transition to PLACE stage if place position is available
                    if self.place_target_position is not None:
                        logger.info("Transitioning to PLACE stage")
                        self.grasp_stage = GraspStage.PLACE
                    else:
                        # No place position, just mark as overall success
                        self.overall_success = True
                else:
                    logger.warning("No object detected in gripper")
                    self.task_failed = True
                    self.overall_success = False
        else:
            # Command retraction to pre-grasp
            logger.info("Retracting to pre-grasp position")
            self.arm.cmd_ee_pose(self.final_pregrasp_pose, line_mode=True)
            self.arm.close_gripper()
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()

    def execute_place(self):
        """Execute place stage: move to place position and release object."""
        ee_pose = self.arm.get_ee_pose()

        if self.waiting_for_reach:
            if self._check_reach_timeout():
                return

            # Check if reached place pose
            place_pose = self.get_place_target_pose()
            if place_pose:
                reached = is_target_reached(place_pose, ee_pose, self.pbvs.target_tolerance)

                if reached:
                    logger.info("Reached place position, releasing gripper")
                    self.arm.release_gripper()
                    time.sleep(1.0)  # Give time for gripper to open

                    # Store the place pose for retraction
                    self.place_pose = place_pose

                    # Transition to RETRACT stage
                    logger.info("Transitioning to RETRACT stage")
                    self.grasp_stage = GraspStage.RETRACT
                    self.waiting_for_reach = False
        else:
            # Get place pose and command movement
            place_pose = self.get_place_target_pose()
            if place_pose:
                logger.info("Moving to place position")
                self.arm.cmd_ee_pose(place_pose, line_mode=True)
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
            else:
                logger.error("Failed to get place target pose")
                self.task_failed = True
                self.overall_success = False

    def execute_retract(self):
        """Execute retract stage: retract from place position."""
        ee_pose = self.arm.get_ee_pose()

        if self.waiting_for_reach:
            if self._check_reach_timeout():
                return

            # Check if reached retract pose
            if self.place_pose:
                reached = is_target_reached(self.retract_pose, ee_pose, self.pbvs.target_tolerance)

                if reached:
                    logger.info("Reached retract position")
                    # Return to observe position
                    logger.info("Returning to observe position")
                    self.arm.gotoObserve()
                    self.arm.close_gripper()

                    # Mark overall success
                    self.overall_success = True
                    logger.info("Pick and place completed successfully!")
                    self.waiting_for_reach = False
        else:
            # Calculate and command retract pose
            if self.place_pose:
                self.retract_pose = apply_grasp_distance(self.place_pose, self.pregrasp_distance)
                logger.info("Retracting from place position")
                self.arm.cmd_ee_pose(self.retract_pose, line_mode=True)
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
            else:
                logger.error("No place pose stored for retraction")
                self.task_failed = True
                self.overall_success = False

    def capture_and_process(
        self,
    ) -> Tuple[
        Optional[np.ndarray], Optional[Detection3DArray], Optional[Detection2DArray], Optional[Pose]
    ]:
        """Capture frame from camera data and process detections."""
        # Check if we have all required data
        if self.latest_rgb is None or self.latest_depth is None or self.detector is None:
            return None, None, None, None

        # Get EE pose and camera transform
        ee_pose = self.arm.get_ee_pose()
        ee_transform = pose_to_matrix(ee_pose)
        camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)
        camera_pose = matrix_to_pose(camera_transform)

        # Process detections
        detection_3d_array, detection_2d_array = self.detector.process_frame(
            self.latest_rgb, self.latest_depth, camera_transform
        )

        return self.latest_rgb, detection_3d_array, detection_2d_array, camera_pose

    def pick_target(self, x: int, y: int) -> bool:
        """Select a target object at the given pixel coordinates."""
        if not self.last_detection_2d_array or not self.last_detection_3d_array:
            logger.warning("No detections available for target selection")
            return False

        clicked_3d = find_clicked_detection(
            (x, y), self.last_detection_2d_array.detections, self.last_detection_3d_array.detections
        )
        if clicked_3d and self.pbvs:
            self.pbvs.set_target(clicked_3d)

            # Store target object height (z dimension)
            if clicked_3d.bbox and clicked_3d.bbox.size:
                self.target_object_height = clicked_3d.bbox.size.z
                logger.info(f"Target object height: {self.target_object_height:.3f}m")

            logger.info(
                f"Target selected: ID={clicked_3d.id}, pos=({clicked_3d.bbox.center.position.x:.3f}, {clicked_3d.bbox.center.position.y:.3f}, {clicked_3d.bbox.center.position.z:.3f})"
            )
            self.grasp_stage = GraspStage.PRE_GRASP
            self.reached_poses.clear()
            self.adjustment_count = 0
            self.waiting_for_reach = False
            self.last_commanded_pose = None
            self.stabilization_start_time = time.time()
            return True
        return False

    def update(self) -> Optional[Dict[str, Any]]:
        """Main update function that handles capture, processing, control, and visualization."""
        # Capture and process frame
        rgb, detection_3d_array, detection_2d_array, camera_pose = self.capture_and_process()
        if rgb is None:
            return None

        # Store for target selection
        self.last_detection_3d_array = detection_3d_array
        self.last_detection_2d_array = detection_2d_array

        # Handle target selection if click is pending
        if self.target_click:
            x, y = self.target_click
            if self.pick_target(x, y):
                self.target_click = None

        # Update tracking if we have detections and not in IDLE or CLOSE_AND_RETRACT
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
            GraspStage.PLACE: self.execute_place,
            GraspStage.RETRACT: self.execute_retract,
        }
        if self.grasp_stage in stage_handlers:
            stage_handlers[self.grasp_stage]()

        # Get tracking status
        target_tracked = self.pbvs.get_current_target() is not None if self.pbvs else False

        # Create feedback object
        ee_pose = self.arm.get_ee_pose()
        feedback = Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            last_commanded_pose=self.last_commanded_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=camera_pose,
            target_pose=self.pbvs.target_grasp_pose if self.pbvs else None,
            waiting_for_reach=self.waiting_for_reach,
            success=self.overall_success,
        )

        # Create visualization only if task is running
        if self.task_running:
            self.current_visualization = create_manipulation_visualization(
                rgb, feedback, detection_3d_array, detection_2d_array
            )

            # Publish visualization
            if self.current_visualization is not None:
                self._publish_visualization(self.current_visualization)

        return feedback

    def _publish_visualization(self, viz_image: np.ndarray):
        """Publish visualization image to LCM."""
        try:
            # Convert BGR to RGB for publishing
            viz_rgb = cv2.cvtColor(viz_image, cv2.COLOR_BGR2RGB)

            # Create LCM Image message
            height, width = viz_rgb.shape[:2]
            data = viz_rgb.tobytes()

            msg = Image(
                data_length=len(data),
                height=height,
                width=width,
                encoding="rgb8",
                is_bigendian=0,
                step=width * 3,
                data=data,
            )

            self.viz_image.publish(msg)
        except Exception as e:
            logger.error(f"Error publishing visualization: {e}")

    def check_target_stabilized(self) -> bool:
        """Check if the commanded poses have stabilized."""
        if len(self.reached_poses) < self.reached_poses.maxlen:
            return False

        # Extract positions
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.reached_poses]
        )

        # Calculate standard deviation for each axis
        std_devs = np.std(positions, axis=0)

        # Check if all axes are below threshold
        return np.all(std_devs < self.pose_stabilization_threshold)

    def get_place_target_pose(self) -> Optional[Pose]:
        """Get the place target pose with z-offset applied based on object height."""
        if self.place_target_position is None:
            return None

        # Create a copy of the place position
        place_pos = self.place_target_position.copy()

        # Apply z-offset if target object height is known
        if self.target_object_height is not None:
            z_offset = self.target_object_height / 2.0
            place_pos[2] += z_offset + 0.05
            logger.info(f"Applied z-offset of {z_offset:.3f}m to place position")

        # Create place pose
        place_center_pose = Pose(
            Point(place_pos[0], place_pos[1], place_pos[2]), Quaternion(0.0, 0.0, 0.0, 1.0)
        )

        # Get current EE pose
        ee_pose = self.arm.get_ee_pose()

        # Use update_target_grasp_pose with no grasp distance and current pitch angle
        place_pose = update_target_grasp_pose(
            place_center_pose,
            ee_pose,
            grasp_distance=0.0,  # No grasp distance for placing
            grasp_pitch_degrees=self.grasp_pitch_degrees,  # Use current grasp pitch
        )

        return place_pose

    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()
