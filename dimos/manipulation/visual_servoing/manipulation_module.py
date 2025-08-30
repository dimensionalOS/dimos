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
Manipulation module for robotic grasping with visual servoing and integrated 3D detection.
Handles object detection, grasping logic, state machine, and hardware coordination as a Dimos module.
Processes RGB-D data directly to reduce latency and publishes detection arrays.
"""

import cv2
import time
import threading
from typing import Optional, Any, Dict, Tuple
from enum import Enum
from collections import deque

import numpy as np

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import Vector3, Pose, Quaternion
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.std_msgs import String
from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.protocol.tf import TF
from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.perception.common.utils import find_clicked_detection
from dimos.manipulation.visual_servoing.utils import (
    create_manipulation_visualization,
    update_target_grasp_pose,
    is_target_reached,
    select_points_from_depth,
    transform_points_3d,
)
from dimos.utils.transform_utils import pose_to_matrix
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.visual_servoing.manipulation_module")


class GraspStage(Enum):
    """Enum for different grasp stages."""

    IDLE = "idle"
    PRE_GRASP = "pre_grasp"
    GRASP = "grasp"
    CLOSE_AND_RETRACT = "close_and_retract"
    PLACE = "place"
    RETRACT = "retract"


class Feedback:
    """Feedback data containing state information about the manipulation process."""

    def __init__(
        self,
        grasp_stage: GraspStage,
        target_tracked: bool,
        current_executed_pose: Optional[Pose] = None,
        current_ee_pose: Optional[Pose] = None,
        current_camera_pose: Optional[Pose] = None,
        target_pose: Optional[Pose] = None,
        waiting_for_reach: bool = False,
        success: Optional[bool] = None,
    ):
        self.grasp_stage = grasp_stage
        self.target_tracked = target_tracked
        self.current_executed_pose = current_executed_pose
        self.current_ee_pose = current_ee_pose
        self.current_camera_pose = current_camera_pose
        self.target_pose = target_pose
        self.waiting_for_reach = waiting_for_reach
        self.success = success


class ManipulationModule(Module):
    """
    Manipulation module with integrated 3D detection for visual servoing and grasping.

    Subscribes to:
        - RGB images (for detection and visualization)
        - Depth images (for 3D detection)
        - Camera info (for intrinsics)

    Publishes:
        - Detection3DArray (3D object detections in base frame)
        - Detection2DArray (2D object detections)
        - Visualization images
        - Grasp state

    RPC methods:
        - handle_keyboard_command: Process keyboard input
        - pick_and_place: Execute pick and place task with optional place location
        - get_single_rgb_frame: Get latest RGB frame
    """

    # LCM inputs
    rgb_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    # LCM outputs
    viz_image: Out[Image] = None
    grasp_state: Out[String] = None  # Publish grasp state
    detection3d_array: Out[Detection3DArray] = None  # Output 3D detections
    detection2d_array: Out[Detection2DArray] = None  # Output 2D detections

    def __init__(
        self,
        piper_arm_module=None,  # PiperArmModule instance
        min_confidence: float = 0.6,
        min_points: int = 30,
        max_depth: float = 1.0,
        max_object_size: float = 0.15,
        camera_frame_id: str = "camera_link",
        base_frame_id: str = "base_link",
        **kwargs,
    ):
        """
        Initialize manipulation module.

        Args:
            piper_arm_module: PiperArmModule instance for arm control
            min_confidence: Minimum detection confidence threshold
            min_points: Minimum 3D points required for valid detection
            max_depth: Maximum valid depth in meters
            max_object_size: Maximum object size to consider valid
            camera_frame_id: TF frame ID for camera
            base_frame_id: TF frame ID for robot base
        """
        super().__init__(**kwargs)

        # Store reference to PiperArmModule
        self.arm = piper_arm_module

        # Detection parameters
        self.min_confidence = min_confidence
        self.min_points = min_points
        self.max_depth = max_depth
        self.max_object_size = max_object_size
        self.camera_frame_id = camera_frame_id
        self.base_frame_id = base_frame_id

        # Initialize PBVS controller
        self.pbvs = PBVS()

        # Initialize TF listener
        self.tf = TF()

        # Detection processor (will be initialized when camera info is received)
        self.detector = None
        self.camera_intrinsics = None

        # Control state
        self.last_valid_target = None
        self.waiting_for_reach = False
        self.current_executed_pose = None  # Track the actual pose sent to arm
        self.target_updated = False
        self.waiting_start_time = None
        self.reach_pose_timeout = 20.0

        # Grasp parameters
        self.grasp_width_offset = 0.03
        self.pregrasp_distance = 0.25
        self.grasp_distance_range = 0.03
        self.grasp_close_delay = 2.0
        self.grasp_reached_time = None
        self.gripper_max_opening = 0.07

        # Workspace limits and dynamic pitch parameters
        self.workspace_min_radius = 0.2
        self.workspace_max_radius = 0.75
        self.min_grasp_pitch_degrees = 5.0
        self.max_grasp_pitch_degrees = 60.0

        # Grasp stage tracking
        self.grasp_stage = GraspStage.IDLE

        # Pose stabilization tracking
        self.pose_history_size = 4
        self.pose_stabilization_threshold = 0.01
        self.stabilization_timeout = 25.0
        self.stabilization_start_time = None
        self.reached_poses = deque(maxlen=self.pose_history_size)
        self.adjustment_count = 0

        # Pose reachability tracking
        self.ee_pose_history = deque(maxlen=20)  # Keep history of EE poses
        self.stuck_pose_threshold = 0.001  # 1mm movement threshold
        self.stuck_pose_adjustment_degrees = 5.0
        self.stuck_count = 0
        self.max_stuck_reattempts = 7

        # State for visualization
        self.current_visualization = None

        # Grasp result and task tracking
        self.pick_success = None
        self.final_pregrasp_pose = None
        self.task_failed = False
        self.overall_success = None

        # Task control
        self.task_running = False
        self.task_thread = None
        self.stop_event = threading.Event()

        # Latest sensor data
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.ee_frame_id = "ee_link"  # Frame ID for end-effector

        # Target selection
        self.target_click = None

        # Place target position and object info
        self.home_pose = Pose(
            position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        self.place_target_position = None
        self.target_object_height = None
        self.retract_distance = 0.12
        self.place_pose = None
        self.retract_pose = None

    @rpc
    def start(self):
        """Start the manipulation module."""
        # Subscribe to sensor inputs
        self.rgb_image.subscribe(self._on_rgb_image)
        self.depth_image.subscribe(self._on_depth_image)
        self.camera_info.subscribe(self._on_camera_info)

        # Go to observe position after start
        self.arm.goto_observe()

        logger.info("Manipulation module started")

    @rpc
    def stop(self):
        """Stop the manipulation module."""
        # Stop any running task
        self.stop_event.set()
        if self.task_thread and self.task_thread.is_alive():
            self.task_thread.join(timeout=5.0)

        self.reset_to_idle()
        logger.info("Manipulation module stopped")

    def _on_rgb_image(self, msg: Image):
        """Handle RGB image messages."""
        try:
            self.latest_rgb = msg.data
        except Exception as e:
            logger.error(f"Error processing RGB image: {e}")

    def _on_depth_image(self, msg: Image):
        """Handle depth image messages."""
        self.latest_depth = msg.data

    def _on_camera_info(self, msg: CameraInfo):
        """Handle camera info messages."""
        self.latest_camera_info = msg
        # Extract camera intrinsics
        intrinsics = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]
        # Initialize detector if not already done or intrinsics changed
        if self.detector is None or self.camera_intrinsics != intrinsics:
            self.camera_intrinsics = intrinsics
            self.detector = Detection3DProcessor(
                camera_intrinsics=self.camera_intrinsics,
                min_confidence=self.min_confidence,
                min_points=self.min_points,
                max_depth=self.max_depth,
                max_object_size=self.max_object_size,
            )
            logger.info(f"Initialized detector with intrinsics: {self.camera_intrinsics}")

    def _get_ee_pose(self) -> Optional[Pose]:
        """Get current end-effector pose from TF."""
        try:
            transform = self.tf.get(
                parent_frame=self.base_frame_id,
                child_frame=self.ee_frame_id,
                time_point=None,
                time_tolerance=1.0,
            )
            if transform:
                return transform.to_pose()
            else:
                logger.warning(
                    f"No transform available from {self.base_frame_id} to {self.ee_frame_id}"
                )
                return None
        except Exception as e:
            logger.error(f"Error getting EE pose from TF: {e}")
            return None

    def _process_detections(self):
        """Process current frame and generate detections."""
        if self.latest_rgb is None or self.latest_depth is None or self.detector is None:
            return

        try:
            # Get transform from camera to base frame
            transform = self.tf.get(
                parent_frame=self.base_frame_id,
                child_frame=self.camera_frame_id,
                time_point=None,
                time_tolerance=1.0,
            )

            # Process frame with detector, passing Transform object directly
            detection3d_array, detection2d_array = self.detector.process_frame(
                self.latest_rgb, self.latest_depth, transform
            )

            # Publish detections
            if self.detection3d_array:
                self.detection3d_array.publish(detection3d_array)
            if self.detection2d_array:
                self.detection2d_array.publish(detection2d_array)

            # Store for internal use
            self.last_detection_3d_array = detection3d_array
            self.last_detection_2d_array = detection2d_array

        except Exception as e:
            logger.error(f"Error processing detections: {e}")

    @rpc
    def get_single_rgb_frame(self) -> Optional[np.ndarray]:
        """
        get the latest rgb frame from the camera
        """
        return self.latest_rgb

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
            self.arm.soft_stop()
            self.stop_event.set()
            self.task_running = False
            return "stop"
        elif key_code == ord(" ") and self.pbvs and self.pbvs.target_grasp_pose:
            if self.grasp_stage == GraspStage.PRE_GRASP:
                self.set_grasp_stage(GraspStage.GRASP)
            logger.info("Executing target pose")
            return "execute"
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

        if target_x is not None and target_y is not None:
            self.target_click = (target_x, target_y)

        # Handle place location if provided
        if place_x is not None and place_y is not None and self.latest_depth is not None:
            points_3d_camera = select_points_from_depth(
                self.latest_depth,
                (place_x, place_y),
                self.camera_intrinsics,
                radius=10,
            )

            if points_3d_camera.size > 0:
                # Get camera transform from TF to transform points to world frame
                camera_transform_msg = self.tf.get(
                    parent_frame=self.base_frame_id,
                    child_frame=self.camera_frame_id,
                    time_point=None,
                    time_tolerance=1.0,
                )
                if camera_transform_msg:
                    camera_transform = pose_to_matrix(camera_transform_msg.to_pose())

                    points_3d_world = transform_points_3d(
                        points_3d_camera,
                        camera_transform,
                        to_robot=True,
                    )

                    place_position = np.mean(points_3d_world, axis=0)
                    self.place_target_position = place_position
                    logger.info(
                        f"Place target set at position: ({place_position[0]:.3f}, {place_position[1]:.3f}, {place_position[2]:.3f})"
                    )
                else:
                    logger.warning("No EE pose available for place location transformation")
                    self.place_target_position = None
            else:
                logger.warning("No valid depth points found at place location")
                self.place_target_position = None
        else:
            self.place_target_position = None

        self.task_failed = False
        self.stop_event.clear()

        if self.task_thread and self.task_thread.is_alive():
            self.stop_event.set()
            self.task_thread.join(timeout=1.0)
        self.task_thread = threading.Thread(target=self._run_pick_and_place, daemon=True)
        self.task_thread.start()

        return {"status": "started", "message": "Pick and place task started"}

    def _run_pick_and_place(self):
        """Run the pick and place task loop."""
        self.task_running = True
        logger.info("Starting pick and place task")

        try:
            while not self.stop_event.is_set():
                if self.task_failed:
                    logger.error("Task failed, terminating pick and place")
                    self.stop_event.set()
                    break

                feedback = self.update()
                if feedback is None:
                    time.sleep(0.01)
                    continue

                if feedback.success is not None:
                    if feedback.success:
                        logger.info("Pick and place completed successfully!")
                    else:
                        logger.warning("Pick and place failed")
                    self.reset_to_idle()
                    self.stop_event.set()
                    break

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
        # Publish state change
        if self.grasp_state:
            self.grasp_state.publish(String(data=stage.value))

    def calculate_dynamic_grasp_pitch(self, target_pose: Pose) -> float:
        """
        Calculate grasp pitch dynamically based on distance from robot base.
        Maps workspace radius to grasp pitch angle.

        Args:
            target_pose: Target pose

        Returns:
            Grasp pitch angle in degrees
        """
        # Calculate 3D distance from robot base (assumes robot at origin)
        position = target_pose.position
        distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)

        # Clamp distance to workspace limits
        distance = np.clip(distance, self.workspace_min_radius, self.workspace_max_radius)

        # Linear interpolation: min_radius -> max_pitch, max_radius -> min_pitch
        # Normalized distance (0 to 1)
        normalized_dist = (distance - self.workspace_min_radius) / (
            self.workspace_max_radius - self.workspace_min_radius
        )

        # Inverse mapping: closer objects need higher pitch
        pitch_degrees = self.max_grasp_pitch_degrees - (
            normalized_dist * (self.max_grasp_pitch_degrees - self.min_grasp_pitch_degrees)
        )

        return pitch_degrees

    def check_within_workspace(self, target_pose: Pose) -> bool:
        """
        Check if pose is within workspace limits and log error if not.

        Args:
            target_pose: Target pose to validate

        Returns:
            True if within workspace, False otherwise
        """
        # Calculate 3D distance from robot base
        position = target_pose.position
        distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)

        if not (self.workspace_min_radius <= distance <= self.workspace_max_radius):
            logger.error(
                f"Target outside workspace limits: distance {distance:.3f}m not in [{self.workspace_min_radius:.2f}, {self.workspace_max_radius:.2f}]"
            )
            return False

        return True

    def _check_reach_timeout(self) -> Tuple[bool, float]:
        """Check if robot has exceeded timeout while reaching pose.

        Returns:
            Tuple of (timed_out, time_elapsed)
        """
        if self.waiting_start_time:
            time_elapsed = time.time() - self.waiting_start_time
            if time_elapsed > self.reach_pose_timeout:
                logger.warning(
                    f"Robot failed to reach pose within {self.reach_pose_timeout}s timeout"
                )
                self.task_failed = True
                self.reset_to_idle()
                return True, time_elapsed
            return False, time_elapsed
        return False, 0.0

    def _check_if_stuck(self) -> bool:
        """
        Check if robot is stuck by analyzing pose history.

        Returns:
            Tuple of (is_stuck, max_std_dev_mm)
        """
        if len(self.ee_pose_history) < self.ee_pose_history.maxlen:
            return False

        # Extract positions from pose history
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.ee_pose_history]
        )

        # Calculate standard deviation of positions
        std_devs = np.std(positions, axis=0)
        # Check if all standard deviations are below stuck threshold
        is_stuck = np.all(std_devs < self.stuck_pose_threshold)

        return is_stuck

    def check_reach_and_adjust(self) -> bool:
        """
        Check if robot has reached the current executed pose while waiting.
        Handles timeout internally by failing the task.
        Also detects if the robot is stuck (not moving towards target).

        Returns:
            True if reached, False if still waiting or not in waiting state
        """
        if not self.waiting_for_reach or not self.current_executed_pose:
            return False

        # Get current end-effector pose from TF
        ee_pose = self._get_ee_pose()
        if not ee_pose:
            return False
        target_pose = self.current_executed_pose

        # Check for timeout - this will fail task and reset if timeout occurred
        timed_out, _ = self._check_reach_timeout()
        if timed_out:
            return False

        self.ee_pose_history.append(ee_pose)

        # Check if robot is stuck
        is_stuck = self._check_if_stuck()
        if is_stuck:
            if self.grasp_stage == GraspStage.RETRACT or self.grasp_stage == GraspStage.PLACE:
                self.waiting_for_reach = False
                self.waiting_start_time = None
                self.stuck_count = 0
                self.ee_pose_history.clear()
                return True
            self.stuck_count += 1
            pitch_degrees = self.calculate_dynamic_grasp_pitch(target_pose)
            if self.stuck_count % 2 == 0:
                pitch_degrees += self.stuck_pose_adjustment_degrees * (1 + self.stuck_count // 2)
            else:
                pitch_degrees -= self.stuck_pose_adjustment_degrees * (1 + self.stuck_count // 2)

            pitch_degrees = max(
                self.min_grasp_pitch_degrees, min(self.max_grasp_pitch_degrees, pitch_degrees)
            )
            updated_target_pose = update_target_grasp_pose(target_pose, ee_pose, 0.0, pitch_degrees)
            self.arm.goto_observe()
            time.sleep(1.5)
            self.arm.cmd_ee_pose(updated_target_pose)
            self.current_executed_pose = updated_target_pose
            self.ee_pose_history.clear()
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()
            return False

        if self.stuck_count >= self.max_stuck_reattempts:
            self.task_failed = True
            self.reset_to_idle()
            return False

        if is_target_reached(target_pose, ee_pose, self.pbvs.target_tolerance):
            self.waiting_for_reach = False
            self.waiting_start_time = None
            self.stuck_count = 0
            self.ee_pose_history.clear()
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
        self.ee_pose_history.clear()
        self.adjustment_count = 0
        self.waiting_for_reach = False
        self.current_executed_pose = None
        self.target_updated = False
        self.stabilization_start_time = None
        self.grasp_reached_time = None
        self.waiting_start_time = None
        self.pick_success = None
        self.final_pregrasp_pose = None
        self.overall_success = None
        self.place_pose = None
        self.retract_pose = None
        self.stuck_count = 0

        self.arm.goto_observe()

    def execute_idle(self):
        """Execute idle stage."""
        pass

    def execute_pre_grasp(self):
        """Execute pre-grasp stage: visual servoing to pre-grasp position."""
        if self.waiting_for_reach:
            if self.check_reach_and_adjust():
                self.reached_poses.append(self.current_executed_pose)
                self.target_updated = False
                time.sleep(0.2)
            return
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

        ee_pose = self._get_ee_pose()
        if not ee_pose:
            return
        dynamic_pitch = self.calculate_dynamic_grasp_pitch(self.pbvs.current_target.bbox.center)

        _, _, _, has_target, target_pose = self.pbvs.compute_control(
            ee_pose, self.pregrasp_distance, dynamic_pitch
        )
        if target_pose and has_target:
            # Validate target pose is within workspace
            if not self.check_within_workspace(target_pose):
                self.task_failed = True
                self.reset_to_idle()
                return

            if self.check_target_stabilized():
                logger.info("Target stabilized, transitioning to GRASP")
                self.final_pregrasp_pose = self.current_executed_pose
                self.grasp_stage = GraspStage.GRASP
                self.adjustment_count = 0
                self.waiting_for_reach = False
            elif not self.waiting_for_reach and self.target_updated:
                self.arm.cmd_ee_pose(target_pose)
                self.current_executed_pose = target_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
                self.target_updated = False
                self.adjustment_count += 1
                time.sleep(0.2)

    def execute_grasp(self):
        """Execute grasp stage: move to final grasp position."""
        if self.waiting_for_reach:
            if self.check_reach_and_adjust() and not self.grasp_reached_time:
                self.grasp_reached_time = time.time()
            return

        if self.grasp_reached_time:
            if (time.time() - self.grasp_reached_time) >= self.grasp_close_delay:
                logger.info("Grasp delay completed, closing gripper")
                self.grasp_stage = GraspStage.CLOSE_AND_RETRACT
            return

        if self.last_valid_target:
            # Calculate dynamic pitch for current target
            dynamic_pitch = self.calculate_dynamic_grasp_pitch(self.last_valid_target.bbox.center)
            normalized_pitch = dynamic_pitch / 90.0
            grasp_distance = -self.grasp_distance_range + (
                2 * self.grasp_distance_range * normalized_pitch
            )

            ee_pose = self._get_ee_pose()
            if not ee_pose:
                return
            _, _, _, has_target, target_pose = self.pbvs.compute_control(
                ee_pose, grasp_distance, dynamic_pitch
            )

            if target_pose and has_target:
                # Validate grasp pose is within workspace
                if not self.check_within_workspace(target_pose):
                    self.task_failed = True
                    self.reset_to_idle()
                    return

                object_width = self.last_valid_target.bbox.size.x
                gripper_opening = max(
                    0.005, min(object_width + self.grasp_width_offset, self.gripper_max_opening)
                )

                logger.info(f"Executing grasp: gripper={gripper_opening * 1000:.1f}mm")
                self.arm.cmd_gripper_ctrl(gripper_opening)
                self.arm.cmd_ee_pose(target_pose, line_mode=True)
                self.current_executed_pose = target_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()

    def execute_close_and_retract(self):
        """Execute the retraction sequence after gripper has been closed."""
        if self.waiting_for_reach and self.final_pregrasp_pose:
            if self.check_reach_and_adjust():
                logger.info("Reached pre-grasp retraction position")
                self.pick_success = self.arm.gripper_object_detected()
                if self.pick_success:
                    logger.info("Object successfully grasped!")
                    if self.place_target_position is not None:
                        logger.info("Transitioning to PLACE stage")
                        self.grasp_stage = GraspStage.PLACE
                    else:
                        self.overall_success = True
                else:
                    logger.warning("No object detected in gripper")
                    self.task_failed = True
                    self.overall_success = False
            return
        if not self.waiting_for_reach:
            logger.info("Retracting to pre-grasp position")
            self.arm.cmd_ee_pose(self.final_pregrasp_pose, line_mode=True)
            self.arm.close_gripper()
            self.current_executed_pose = self.final_pregrasp_pose
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()

    def execute_place(self):
        """Execute place stage: move to place position and release object."""
        if self.waiting_for_reach:
            # Use the already executed pose instead of recalculating
            if self.check_reach_and_adjust():
                logger.info("Reached place position, releasing gripper")
                self.arm.release_gripper()
                time.sleep(1.0)
                self.place_pose = self.current_executed_pose
                logger.info("Transitioning to RETRACT stage")
                self.grasp_stage = GraspStage.RETRACT
            return

        if not self.waiting_for_reach:
            place_pose = self.get_place_target_pose()
            if place_pose:
                logger.info("Moving to place position")
                self.arm.cmd_ee_pose(place_pose, line_mode=True)
                self.current_executed_pose = place_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
            else:
                logger.error("Failed to get place target pose")
                self.task_failed = True
                self.overall_success = False

    def execute_retract(self):
        """Execute retract stage: retract from place position."""
        if self.waiting_for_reach and self.retract_pose:
            if self.check_reach_and_adjust():
                logger.info("Reached retract position")
                logger.info("Returning to observe position")
                self.arm.goto_observe()
                self.arm.close_gripper()
                self.overall_success = True
                logger.info("Pick and place completed successfully!")
            return

        if not self.waiting_for_reach:
            if self.place_pose:
                pose_pitch = self.calculate_dynamic_grasp_pitch(self.place_pose)
                self.retract_pose = update_target_grasp_pose(
                    self.place_pose, self.home_pose, self.retract_distance, pose_pitch
                )
                logger.info("Retracting from place position")
                self.arm.cmd_ee_pose(self.retract_pose, line_mode=True)
                self.current_executed_pose = self.retract_pose
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()
            else:
                logger.error("No place pose stored for retraction")
                self.task_failed = True
                self.overall_success = False

    def pick_target(self, x: int, y: int) -> bool:
        """Select a target object at the given pixel coordinates."""
        if not self.last_detection_2d_array or not self.last_detection_3d_array:
            logger.warning("No detections available for target selection")
            return False

        clicked_3d = find_clicked_detection(
            (x, y), self.last_detection_2d_array.detections, self.last_detection_3d_array.detections
        )
        if clicked_3d and self.pbvs:
            # Validate workspace
            if not self.check_within_workspace(clicked_3d.bbox.center):
                self.task_failed = True
                return False

            self.pbvs.set_target(clicked_3d)

            if clicked_3d.bbox and clicked_3d.bbox.size:
                self.target_object_height = clicked_3d.bbox.size.z
                logger.info(f"Target object height: {self.target_object_height:.3f}m")

            position = clicked_3d.bbox.center.position
            logger.info(
                f"Target selected: ID={clicked_3d.id}, pos=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )
            self.grasp_stage = GraspStage.PRE_GRASP
            self.reached_poses.clear()
            self.adjustment_count = 0
            self.waiting_for_reach = False
            self.current_executed_pose = None
            self.stabilization_start_time = time.time()
            return True
        return False

    def update(self) -> Optional[Dict[str, Any]]:
        """Main update function that handles capture, processing, control, and visualization."""
        if self.latest_rgb is None:
            return None

        # Process detections in the update loop instead of callback
        self._process_detections()

        if self.target_click:
            x, y = self.target_click
            if self.pick_target(x, y):
                self.target_click = None

        if (
            self.last_detection_3d_array
            and self.grasp_stage in [GraspStage.PRE_GRASP, GraspStage.GRASP]
            and not self.waiting_for_reach
        ):
            self._update_tracking(self.last_detection_3d_array)
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

        target_tracked = self.pbvs.get_current_target() is not None if self.pbvs else False
        ee_pose = self._get_ee_pose()
        feedback = Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            current_executed_pose=self.current_executed_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=None,  # Not computed anymore with TF-based system
            target_pose=self.pbvs.target_grasp_pose if self.pbvs else None,
            waiting_for_reach=self.waiting_for_reach,
            success=self.overall_success,
        )

        if self.task_running:
            self.current_visualization = create_manipulation_visualization(
                self.latest_rgb,
                feedback,
                self.last_detection_3d_array,
                self.last_detection_2d_array,
            )

            if self.current_visualization is not None:
                self._publish_visualization(self.current_visualization)

        return feedback

    def _publish_visualization(self, viz_image: np.ndarray):
        """Publish visualization image to LCM."""
        try:
            viz_rgb = cv2.cvtColor(viz_image, cv2.COLOR_BGR2RGB)
            msg = Image.from_numpy(viz_rgb)
            self.viz_image.publish(msg)
        except Exception as e:
            logger.error(f"Error publishing visualization: {e}")

    def check_target_stabilized(self) -> bool:
        """Check if the commanded poses have stabilized."""
        if len(self.reached_poses) < self.reached_poses.maxlen:
            return False

        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.reached_poses]
        )
        std_devs = np.std(positions, axis=0)
        return np.all(std_devs < self.pose_stabilization_threshold)

    def get_place_target_pose(self) -> Optional[Pose]:
        """Get the place target pose with z-offset applied based on object height."""
        if self.place_target_position is None:
            return None

        place_pos = self.place_target_position.copy()
        if self.target_object_height is not None:
            z_offset = self.target_object_height / 2.0
            place_pos[2] += z_offset + 0.1

        place_center_pose = Pose(
            position=Vector3(place_pos[0], place_pos[1], place_pos[2]),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )

        ee_pose = self._get_ee_pose()
        if not ee_pose:
            return None

        # Calculate dynamic pitch for place position
        dynamic_pitch = self.calculate_dynamic_grasp_pitch(place_center_pose)

        place_pose = update_target_grasp_pose(
            place_center_pose,
            ee_pose,
            grasp_distance=0.0,
            grasp_pitch_degrees=dynamic_pitch,
        )

        return place_pose

    @rpc
    def cleanup(self):
        """Clean up resources on module destruction."""
        # Arm cleanup is handled by PiperArmModule
        pass
