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
from copy import deepcopy
from typing import Optional, Any, Dict, Union, Tuple
from enum import Enum
from collections import deque
import traceback

import numpy as np

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.geometry_msgs import Vector3, Pose, Quaternion, Transform, Twist
from dimos_lcm.vision_msgs import Detection3D, Detection3DArray, Detection2DArray
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
from dimos.utils.transform_utils import pose_to_matrix, apply_transform
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.manipulation.visual_servoing.manipulation_module")


class GraspStage(Enum):
    """Enum for different grasp stages."""

    IDLE = "idle"
    POSE = "pose"  # Mobile base positioning stage
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

    rgb_image: In[Image] = None
    depth_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    cmd_vel: Out[Twist] = None
    viz_image: Out[Image] = None
    grasp_state: Out[String] = None
    detection3d_array: Out[Detection3DArray] = None
    detection2d_array: Out[Detection2DArray] = None

    def __init__(
        self,
        piper_arm_module=None,
        min_confidence: float = 0.6,
        min_points: int = 30,
        max_depth: float = 1.0,
        max_object_size: float = 0.15,
        camera_frame_id: str = "camera_link",
        base_frame_id: str = "base_link",
        track_frame_id: str = "world",
        reach_timeout: float = 10.0,
        enable_mobile_base: bool = False,
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
            track_frame_id: TF frame ID for tracking frame (world for mobile base)
            reach_timeout: Timeout for reaching poses
            enable_mobile_base: Enable mobile base control for pose adjustment
        """
        super().__init__(**kwargs)

        self.arm = piper_arm_module
        self.min_confidence = min_confidence
        self.min_points = min_points
        self.max_depth = max_depth
        self.max_object_size = max_object_size
        self.camera_frame_id = camera_frame_id
        self.base_frame_id = base_frame_id
        self.track_frame_id = track_frame_id

        self.pbvs = PBVS()
        self.tf = TF()

        self.detector = None
        self.camera_intrinsics = None
        self.last_valid_target = None
        self.waiting_for_reach = False
        self.current_executed_pose = None
        self.waiting_start_time = None
        self.reach_timeout = reach_timeout
        self.grasp_width_offset = 0.03
        self.pregrasp_distance = 0.25
        self.grasp_distance_range = 0.03
        self.grasp_close_delay = 1.0
        self.grasp_reached_time = None
        self.gripper_max_opening = 0.07

        self.workspace_min_radius = 0.2
        self.workspace_max_radius = 0.75
        self.min_grasp_pitch_degrees = 5.0
        self.max_grasp_pitch_degrees = 60.0

        self.grasp_stage = GraspStage.IDLE
        self.pose_history_size = 4
        self.pose_stabilization_threshold = 0.01  # 1cm position stability
        self.reached_poses = deque(maxlen=self.pose_history_size)

        self.current_visualization = None
        self.pick_success = None
        self.final_pregrasp_pose = None
        self.task_failed = False
        self.overall_success = None
        self.task_running = False
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.last_detection_3d_array = None
        self.last_detection_2d_array = None
        self.last_target_update_time = (
            None  # Track when the target was last updated with new 2D detection
        )
        self.target_has_new_detection = False  # Flag for new detection for the specific target
        self.detection_timeout = 5.0  # Timeout in seconds for no new detections during pre-grasp
        self.ee_frame_id = "ee_link"

        self.target_click = None
        self.target_object = None
        self.home_pose = Pose(
            position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        self.place_target_position = None
        self.target_object_height = None
        self.retract_distance = 0.12
        self.place_pose = None
        self.retract_pose = None

        self.enable_mobile_base = enable_mobile_base
        self.pose_adjusted = False
        self.cmd_height = 0.0  # Height command for mobile base
        self.cmd_pitch = 0.0  # Pitch command for mobile base
        self.pose_publisher_thread = None
        self.pose_publisher_stop = threading.Event()

    @rpc
    def start(self):
        """Start the manipulation module."""
        self.rgb_image.subscribe(lambda msg: setattr(self, "latest_rgb", msg.data))
        self.depth_image.subscribe(lambda msg: self._process_depth(msg))
        self.camera_info.subscribe(lambda msg: self._setup_detector(msg))

        if self.enable_mobile_base and self.cmd_vel:
            self.pose_publisher_stop.clear()
            self.pose_publisher_thread = threading.Thread(
                target=self._pose_publisher_loop, daemon=True
            )
            self.pose_publisher_thread.start()

        self.arm.goto_observe()
        logger.info("Manipulation module started")

    @rpc
    def stop(self):
        """Stop the manipulation module."""
        self.task_running = False

        if self.pose_publisher_thread:
            self.pose_publisher_stop.set()
            self.pose_publisher_thread.join(timeout=1.0)
            self.pose_publisher_thread = None

        self.reset_to_idle()
        logger.info("Manipulation module stopped")

    def _process_depth(self, msg: Image):
        """Process depth image and convert format if needed."""
        if msg.format == ImageFormat.DEPTH16:
            self.latest_depth = msg.data.astype(np.float32) / 1000.0
        else:
            self.latest_depth = msg.data

    def _setup_detector(self, msg: CameraInfo):
        """Setup detector with camera intrinsics."""
        self.latest_camera_info = msg
        intrinsics = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]

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

    def _process_detections(self):
        """Process current frame and generate detections."""
        if self.latest_rgb is None or self.latest_depth is None or self.detector is None:
            return

        try:
            transform = self.tf.get(
                parent_frame=self.track_frame_id,
                child_frame=self.camera_frame_id,
                time_point=None,
                time_tolerance=0.2,
            )

            if not transform:
                logger.warning(
                    f"No transform available from {self.camera_frame_id} to {self.track_frame_id} frame"
                )
                return

            detection3d_array, detection2d_array = self.detector.process_frame(
                self.latest_rgb, self.latest_depth, transform
            )

            if self.detection3d_array:
                self.detection3d_array.publish(detection3d_array)
            if self.detection2d_array:
                self.detection2d_array.publish(detection2d_array)

            self.last_detection_3d_array = detection3d_array
            self.last_detection_2d_array = detection2d_array

        except Exception as e:
            logger.error(f"Error processing detections: {e}")

    def _pose_publisher_loop(self):
        """Continuously publish pose commands to maintain mobile base position."""
        while not self.pose_publisher_stop.is_set():
            if self.cmd_vel and (self.cmd_height != 0.0 or self.cmd_pitch != 0.0):
                pose_cmd = Twist(
                    linear=Vector3(0, 0, self.cmd_height), angular=Vector3(0, self.cmd_pitch, 0)
                )
                self.cmd_vel.publish(pose_cmd)
            time.sleep(0.1)

    @rpc
    def get_single_rgb_frame(self) -> Optional[np.ndarray]:
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
            self.task_running = False
            self.reset_to_idle()
            return "reset"
        elif key_code == ord("s"):
            logger.info("SOFT STOP - Emergency stopping robot!")
            self.arm.soft_stop()
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
        elif key_code == ord("1"):
            self.arm.goto_observe(pitch=90.0)
            return "observe"
        elif key_code == ord("2"):
            self.arm.goto_observe(pitch=120.0)
            return "observe"
        elif key_code == ord("3"):
            self.arm.goto_observe(pitch=150.0)
            return "observe"

        return ""

    @rpc
    def pick_and_place(
        self,
        pick_x: Union[int, Tuple[int, int], Detection3D] = None,
        pick_y: int = None,
        place_x: int = None,
        place_y: int = None,
    ) -> Dict[str, Any]:
        """
        Execute a pick and place task (blocking).

        Args:
            pick_x: X coordinate for pick location, or a tuple (x, y), or Detection3D object
            pick_y: Y coordinate for pick location (when pick_x is int)
            place_x: X coordinate for place location (optional)
            place_y: Y coordinate for place location (optional)

        Returns:
            Dict with success status and details
        """
        if self.task_running:
            return {"success": False, "error": "Task already running"}

        self.task_running = True
        self.task_failed = False

        try:
            # Handle pick target - support multiple input formats
            if pick_x is not None:
                if isinstance(pick_x, tuple) and len(pick_x) == 2:
                    # Tuple (x, y) provided
                    self.target_click = pick_x
                    self.target_object = None
                elif isinstance(pick_x, Detection3D):
                    # Detection3D object provided
                    self.target_object = pick_x
                    self.target_click = None
                elif isinstance(pick_x, int) and pick_y is not None:
                    # Individual coordinates provided
                    self.target_click = (pick_x, pick_y)
                    self.target_object = None

            # Handle place target
            if place_x is not None and place_y is not None and self.latest_depth is not None:
                self._set_place_target(place_x, place_y)
            else:
                self.place_target_position = None

            success = self._execute_pick_and_place()

            return {
                "success": success,
                "message": "Pick and place completed" if success else "Pick and place failed",
            }

        except Exception as e:
            logger.error(f"Error in pick and place: {e}")

            logger.error(f"Traceback: {traceback.format_exc()}")
            return {"success": False, "error": str(e)}
        finally:
            self.task_running = False
            self.reset_to_idle()

    def _set_place_target(self, place_x: int, place_y: int):
        """Set the place target position from pixel coordinates."""
        points_3d_camera = select_points_from_depth(
            self.latest_depth,
            (place_x, place_y),
            self.camera_intrinsics,
            radius=10,
        )

        if points_3d_camera.size > 0:
            transform = self.tf.get(
                parent_frame=self.track_frame_id,
                child_frame=self.camera_frame_id,
                time_point=None,
                time_tolerance=0.2,
            )
            if transform:
                camera_transform = pose_to_matrix(transform.to_pose())
                points_3d_world = transform_points_3d(
                    points_3d_camera,
                    camera_transform,
                )
                place_position = np.mean(points_3d_world, axis=0)
                self.place_target_position = place_position
                logger.info(
                    f"Place target in world frame: ({place_position[0]:.3f}, {place_position[1]:.3f}, {place_position[2]:.3f})"
                )
            else:
                logger.warning("No transform available for place location")
                self.place_target_position = None
        else:
            logger.warning("No valid depth points found at place location")
            self.place_target_position = None

    def _execute_pick_and_place(self) -> bool:
        """Execute the pick and place task synchronously."""
        logger.info("Executing pick and place task")

        self._process_detections()

        if self.target_click:
            if not self.pick_target(self.target_click):
                logger.error("Failed to select target")
                return False
            self.target_click = None

        if self.target_object:
            if not self.pick_target(self.target_object):
                logger.error("Failed to select target")
                return False
            self.target_object = None

        start_time = time.time()
        while time.time() - start_time < 60.0:
            if self.task_failed:
                logger.error("Task marked as failed")
                return False

            feedback = self.update()
            if feedback and feedback.success is not None:
                return feedback.success

            time.sleep(0.05)

        logger.error("Pick and place timed out")
        return False

    def set_grasp_stage(self, stage: GraspStage):
        """Set the grasp stage and publish state."""
        self.grasp_stage = stage
        logger.info(f"Grasp stage: {stage.value}")
        if self.grasp_state:
            self.grasp_state.publish(String(data=stage.value))

    def calculate_dynamic_grasp_pitch(self, target_pose: Pose) -> float:
        """Calculate grasp pitch based on distance from robot base."""
        position = target_pose.position
        distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)
        distance = np.clip(distance, self.workspace_min_radius, self.workspace_max_radius)

        normalized = (distance - self.workspace_min_radius) / (
            self.workspace_max_radius - self.workspace_min_radius
        )
        return self.max_grasp_pitch_degrees - (
            normalized * (self.max_grasp_pitch_degrees - self.min_grasp_pitch_degrees)
        )

    def check_within_workspace(self, target_pose: Pose) -> bool:
        """Check if pose is within workspace limits."""
        position = target_pose.position
        distance = np.sqrt(position.x**2 + position.y**2 + position.z**2)

        if not (self.workspace_min_radius <= distance <= self.workspace_max_radius):
            logger.error(
                f"Target outside workspace: {distance:.3f}m not in [{self.workspace_min_radius:.2f}, {self.workspace_max_radius:.2f}]"
            )
            return False
        return True

    def _wait_for_reach(self, timeout: float = None) -> bool:
        """
        Wait for robot to reach the target pose.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if reached, False if timeout or error
        """
        if not self.waiting_for_reach or not self.current_executed_pose:
            return True

        timeout = timeout or self.reach_timeout
        start_time = time.time()

        while time.time() - start_time < timeout:
            ee_transform = self.tf.get(
                parent_frame=self.base_frame_id,
                child_frame=self.ee_frame_id,
                time_point=None,
                time_tolerance=1.0,
            )
            if not ee_transform:
                time.sleep(0.1)
                continue
            ee_pose = ee_transform.to_pose()

            _, target_reached = is_target_reached(
                self.current_executed_pose, ee_pose, position_tolerance=self.pbvs.target_tolerance
            )

            if target_reached:
                self.waiting_for_reach = False
                self.waiting_start_time = None
                return True

            time.sleep(0.05)
        logger.error(f"Failed to reach target pose within {timeout}s")

        if self.grasp_stage == GraspStage.RETRACT or self.grasp_stage == GraspStage.PLACE:
            return True

        self.task_failed = True
        return False

    def _get_target_in_base_frame(self) -> Optional[Pose]:
        """Get the tracked target transformed to base_link frame for grasp generation."""
        if not self.pbvs or not self.pbvs.current_target:
            return None

        if self.track_frame_id == self.base_frame_id:
            return Pose(
                position=self.pbvs.current_target.bbox.center.position,
                orientation=self.pbvs.current_target.bbox.center.orientation,
            )

        base_to_target = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame="tracked_object",
            time_point=None,
            time_tolerance=0.2,
        )

        if not base_to_target:
            logger.warning("Cannot transform tracked object to base frame")
            return None

        target_pose = base_to_target.to_pose()

        return target_pose

    def _update_tracking(self, detection_3d_array: Optional[Detection3DArray]) -> bool:
        """Update tracking with new detections."""
        if not detection_3d_array or not self.pbvs:
            return False

        # Store the previous target to check if it changed
        previous_target = self.pbvs.current_target

        target_tracked = self.pbvs.update_tracking(detection_3d_array)
        if target_tracked:
            # Check if the target has been updated with new detection
            if self.pbvs.current_target != previous_target:
                # Target has been updated with new detection from process_frame
                self.target_has_new_detection = True
                self.last_target_update_time = time.time()
                logger.debug("Target updated with new 2D detection")

            self.last_valid_target = self.pbvs.current_target

            # Publish TF for tracked object
            if (
                self.last_valid_target
                and self.last_valid_target.bbox
                and self.last_valid_target.bbox.center
            ):
                self.tf.publish(
                    Transform(
                        translation=self.last_valid_target.bbox.center.position,
                        rotation=self.last_valid_target.bbox.center.orientation,
                        frame_id=self.track_frame_id,
                        child_frame_id="tracked_object",
                        ts=time.time(),
                    )
                )
                logger.info(
                    f"self.last_valid_target: {self.last_valid_target.results[0].pose.pose}"
                )
                self.tf.publish(
                    Transform(
                        translation=self.last_valid_target.results[0].pose.pose.position,
                        rotation=self.last_valid_target.results[0].pose.pose.orientation,
                        frame_id=self.camera_frame_id,
                        child_frame_id="tracked_object_optical",
                        ts=time.time(),
                    )
                )
        return target_tracked

    def reset_to_idle(self):
        """Reset the manipulation system to IDLE state."""
        if self.pbvs:
            self.pbvs.clear_target()
        self.grasp_stage = GraspStage.IDLE
        self.waiting_for_reach = False
        self.current_executed_pose = None
        self.grasp_reached_time = None
        self.waiting_start_time = None
        self.pick_success = None
        self.final_pregrasp_pose = None
        self.cmd_height = 0.0
        self.cmd_pitch = 0.0
        self.pose_adjusted = False
        self.overall_success = None
        self.place_pose = None
        self.retract_pose = None
        self.place_target_position = None
        self.target_object_height = None
        self.reached_poses.clear()  # Clear pose history
        self.pose_adjusted = False
        self.target_has_new_detection = False  # Reset target detection flag
        self.last_target_update_time = None

        if self.enable_mobile_base and self.cmd_vel:
            stop_cmd = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            self.cmd_vel.publish(stop_cmd)
            time.sleep(0.2)
            self.cmd_vel.publish(stop_cmd)

        # Return arm to observe position
        self.arm.goto_observe()

    def execute_pose(self):
        """Adjust mobile base height and pitch to optimize for grasping, continuously sending commands."""
        if not self.enable_mobile_base or not self.cmd_vel:
            # Skip directly to pre-grasp if mobile base control not enabled
            self.set_grasp_stage(GraspStage.PRE_GRASP)
            return

        if not self.pbvs or not self.pbvs.current_target:
            return

        base_to_target = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame="tracked_object",
            time_point=None,
            time_tolerance=0.2,
        )

        if not base_to_target:
            logger.warning("Cannot get tracked_object transform in base_link frame")
            return

        target_pose = base_to_target.to_pose()
        target_height = target_pose.position.z

        self.cmd_height = target_height

        if target_height < 0:
            self.cmd_pitch = -target_height * 0.8
        else:
            self.cmd_pitch = 0.0

        if not self.pose_adjusted:
            self.pose_adjusted = True
            logger.info(
                f"Adjusting mobile base pose: height={target_height:.3f}m, pitch={self.cmd_pitch:.3f}rad"
            )

            time.sleep(2.0)

            logger.info("Mobile base pose adjustment complete, transitioning to PRE_GRASP")
            self.set_grasp_stage(GraspStage.PRE_GRASP)

    def execute_pre_grasp(self):
        """Execute pre-grasp stage: visual servoing to pre-grasp position."""

        if self.waiting_for_reach:
            if self._wait_for_reach():
                time.sleep(0.5)
                self.reached_poses.append(self.current_executed_pose)
                self.waiting_for_reach = False
            return

        if not self.pbvs.current_target:
            return

        if self.check_target_stabilized():
            logger.info("Target stabilized, transitioning to GRASP")
            self.final_pregrasp_pose = self.reached_poses[-1]
            self.set_grasp_stage(GraspStage.GRASP)
            self.reached_poses.clear()
            return

        # Check for detection timeout during pre-grasp
        if self.last_target_update_time is not None:
            time_since_last_detection = time.time() - self.last_target_update_time
            if time_since_last_detection > self.detection_timeout:
                logger.error(
                    f"No new detection for {self.detection_timeout:.1f} seconds during pre-grasp, failing task"
                )
                self.task_failed = True
                return

        # Only compute and execute new grasp pose if target has been updated with new detection
        if not self.target_has_new_detection:
            logger.debug("Target has no new detection update, skipping pre-grasp pose update")
            return

        ee_transform = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame=self.ee_frame_id,
            time_point=None,
            time_tolerance=1.0,
        )
        if not ee_transform:
            return
        ee_pose = ee_transform.to_pose()

        target_in_base = self._get_target_in_base_frame()
        if not target_in_base:
            logger.warning("Cannot get target in base frame for grasp")
            return

        dynamic_pitch = self.calculate_dynamic_grasp_pitch(target_in_base)
        target_pose = self.pbvs.compute_control(
            target_in_base, ee_pose, self.pregrasp_distance, dynamic_pitch
        )

        if not target_pose:
            return

        if not self.check_within_workspace(target_pose):
            logger.error("Target pose outside workspace")
            self.task_failed = True
            return

        logger.info(
            f"Moving to pre-grasp position (sample {len(self.reached_poses) + 1}/{self.pose_history_size})"
        )
        self.arm.cmd_ee_pose(target_pose)
        self.current_executed_pose = target_pose
        self.waiting_for_reach = True
        self.waiting_start_time = time.time()
        # Clear the target detection flag after using it
        self.target_has_new_detection = False

    def execute_grasp(self):
        """Execute grasp stage: move to final grasp position."""
        if self.waiting_for_reach:
            if self._wait_for_reach():
                self.grasp_reached_time = time.time()
            return

        if self.grasp_reached_time:
            if (time.time() - self.grasp_reached_time) >= self.grasp_close_delay:
                logger.info("Closing gripper")
                self.set_grasp_stage(GraspStage.CLOSE_AND_RETRACT)
            return

        if not self.last_valid_target:
            return

        # Only compute and execute new grasp pose if target has been updated with new detection
        if not self.target_has_new_detection:
            logger.debug("Target has no new detection update, skipping final grasp pose update")
            return

        target_in_base = self._get_target_in_base_frame()
        if not target_in_base:
            logger.warning("Cannot get target in base frame for final grasp")
            return

        dynamic_pitch = self.calculate_dynamic_grasp_pitch(target_in_base)
        normalized_pitch = dynamic_pitch / 90.0
        grasp_distance = -self.grasp_distance_range + (
            2 * self.grasp_distance_range * normalized_pitch
        )

        # Get end-effector pose
        ee_transform = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame=self.ee_frame_id,
            time_point=None,
            time_tolerance=1.0,
        )
        if not ee_transform:
            return
        ee_pose = ee_transform.to_pose()

        target_pose = self.pbvs.compute_control(
            target_in_base, ee_pose, grasp_distance, dynamic_pitch
        )
        if target_pose:
            if not self.check_within_workspace(target_pose):
                logger.error("Grasp pose outside workspace")
                self.task_failed = True
                return

            object_width = self.pbvs.current_target.bbox.size.x
            gripper_opening = max(
                0.005, min(object_width + self.grasp_width_offset, self.gripper_max_opening)
            )

            logger.info(f"Moving to grasp position, gripper={gripper_opening * 1000:.1f}mm")
            self.arm.cmd_gripper_ctrl(gripper_opening)
            self.arm.cmd_ee_pose(target_pose, line_mode=True)
            self.current_executed_pose = target_pose
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()
            # Clear the target detection flag after using it
            self.target_has_new_detection = False

    def execute_close_and_retract(self):
        """Execute the retraction sequence after gripper has been closed."""
        if self.waiting_for_reach:
            if self._wait_for_reach():
                self.pick_success = self.arm.gripper_object_detected()
                if self.pick_success:
                    logger.info("Object successfully grasped")
                    if self.place_target_position is not None:
                        self.set_grasp_stage(GraspStage.PLACE)
                    else:
                        self.overall_success = True
                else:
                    logger.error("No object detected in gripper")
                    self.task_failed = True
                    self.overall_success = False
            return

        if self.final_pregrasp_pose:
            logger.info("Closing gripper and retracting")
            self.arm.close_gripper()
            self.arm.cmd_ee_pose(self.final_pregrasp_pose, line_mode=True)
            self.current_executed_pose = self.final_pregrasp_pose
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()

    def execute_place(self):
        """Execute place stage: move to place position and release object."""
        if self.waiting_for_reach:
            if self._wait_for_reach():
                logger.info("Releasing object")
                self.arm.release_gripper()
                time.sleep(1.0)
                self.place_pose = self.current_executed_pose
                self.set_grasp_stage(GraspStage.RETRACT)
            return

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

    def execute_retract(self):
        """Execute retract stage: retract from place position."""
        if self.waiting_for_reach:
            if self._wait_for_reach():
                logger.info("Retraction complete")
                self.arm.goto_observe()
                self.arm.close_gripper()
                self.overall_success = True
            return

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
            logger.error("No place pose for retraction")
            self.task_failed = True

    def pick_target(self, target: Union[Tuple[int, int], Detection3D]) -> bool:
        """
        Select a target object either from pixel coordinates or directly from a Detection3D object.

        Args:
            target: Either a tuple of (x, y) pixel coordinates or a Detection3D object

        Returns:
            bool: True if target was successfully selected, False otherwise
        """
        self._process_detections()

        if isinstance(target, tuple):
            if not self.last_detection_2d_array or not self.last_detection_3d_array:
                logger.warning("No detections available for pixel selection")
                return False

            target_detection = find_clicked_detection(
                target,
                self.last_detection_2d_array.detections,
                self.last_detection_3d_array.detections,
            )

            if not target_detection:
                logger.warning(f"No object found at pixel coordinates ({target[0]}, {target[1]})")
                return False

        elif isinstance(target, Detection3D):
            target_detection = target

        self.pbvs.set_target(target_detection)
        self.last_valid_target = target_detection
        # Mark that we have a new target with fresh detection
        self.target_has_new_detection = True
        self.last_target_update_time = time.time()
        if target_detection and target_detection.bbox and target_detection.bbox.center:
            self.tf.publish(
                Transform(
                    translation=target_detection.bbox.center.position,
                    rotation=target_detection.bbox.center.orientation,
                    frame_id=self.track_frame_id,
                    child_frame_id="tracked_object",
                    ts=time.time(),
                )
            )

        if target_detection.bbox and target_detection.bbox.size:
            self.target_object_height = target_detection.bbox.size.z

        position = target_detection.bbox.center.position
        logger.info(f"Target selected: pos=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")

        if self.last_detection_3d_array:
            self._update_tracking(self.last_detection_3d_array)

        if self.enable_mobile_base:
            self.set_grasp_stage(GraspStage.POSE)
            self.pose_adjusted = False
        else:
            self.set_grasp_stage(GraspStage.PRE_GRASP)

        self.waiting_for_reach = False
        self.current_executed_pose = None
        return True

    def update(self) -> Optional[Feedback]:
        """Main update function that handles capture, processing, control, and visualization."""
        if self.latest_rgb is None:
            return None

        self._process_detections()

        if self.target_click:
            if self.pick_target(self.target_click):
                self.target_click = None

        if self.last_detection_3d_array and self.grasp_stage in [
            GraspStage.POSE,  # Also track during pose adjustment
            GraspStage.PRE_GRASP,
            GraspStage.GRASP,
        ]:
            self._update_tracking(self.last_detection_3d_array)
        if self.grasp_stage == GraspStage.POSE:
            self.execute_pose()
        elif self.grasp_stage == GraspStage.PRE_GRASP:
            self.execute_pre_grasp()
        elif self.grasp_stage == GraspStage.GRASP:
            self.execute_grasp()
        elif self.grasp_stage == GraspStage.CLOSE_AND_RETRACT:
            self.execute_close_and_retract()
        elif self.grasp_stage == GraspStage.PLACE:
            self.execute_place()
        elif self.grasp_stage == GraspStage.RETRACT:
            self.execute_retract()

        target_tracked = self.pbvs.current_target is not None if self.pbvs else False
        ee_transform = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame=self.ee_frame_id,
            time_point=None,
            time_tolerance=1.0,
        )
        ee_pose = ee_transform.to_pose() if ee_transform else None

        feedback = Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            current_executed_pose=self.current_executed_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=None,
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

            if self.current_visualization is not None and self.viz_image:
                try:
                    viz_rgb = cv2.cvtColor(self.current_visualization, cv2.COLOR_BGR2RGB)
                    self.viz_image.publish(Image.from_numpy(viz_rgb))
                except Exception as e:
                    logger.error(f"Error publishing visualization: {e}")

        return feedback

    def get_place_target_pose(self) -> Optional[Pose]:
        """Get the place target pose with z-offset applied based on object height."""
        if self.place_target_position is None:
            return None

        # Place position is stored in world frame
        place_pos_world = self.place_target_position.copy()
        if self.target_object_height is not None:
            z_offset = self.target_object_height / 2.0
            place_pos_world[2] += z_offset + 0.1

        place_pose_world = Pose(
            position=Vector3(place_pos_world[0], place_pos_world[1], place_pos_world[2]),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        # Transform from world to base frame if needed
        if self.track_frame_id != self.base_frame_id:
            # Create a transform for the place target in world frame
            place_transform = Transform(
                translation=place_pose_world.position,
                rotation=place_pose_world.orientation,
                frame_id=self.track_frame_id,
                child_frame_id="place_target",
                ts=time.time(),
            )
            self.tf.publish(place_transform)

            # Get the place target in base frame
            place_in_base = self.tf.get(
                parent_frame=self.base_frame_id,
                child_frame="place_target",
                time_point=None,
                time_tolerance=0.2,
            )
            if not place_in_base:
                logger.warning("Cannot transform place target to base frame")
                return None

            place_center_pose = place_in_base.to_pose()
        else:
            place_center_pose = place_pose_world

        # Get end-effector pose
        ee_transform = self.tf.get(
            parent_frame=self.base_frame_id,
            child_frame=self.ee_frame_id,
            time_point=None,
            time_tolerance=1.0,
        )
        if not ee_transform:
            return None
        ee_pose = ee_transform.to_pose()

        dynamic_pitch = self.calculate_dynamic_grasp_pitch(place_center_pose)

        place_pose = update_target_grasp_pose(
            place_center_pose,
            ee_pose,
            grasp_distance=0.0,
            grasp_pitch_degrees=dynamic_pitch,
        )

        return place_pose

    def check_target_stabilized(self) -> bool:
        """Check if the commanded poses have stabilized."""
        if len(self.reached_poses) < self.reached_poses.maxlen:
            return False

        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.reached_poses]
        )
        std_devs = np.std(positions, axis=0)
        is_stable = np.all(std_devs < self.pose_stabilization_threshold)

        return is_stable

    @rpc
    def cleanup(self):
        """Clean up resources on module destruction."""
        self.detector.cleanup()
