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

from collections import deque
from enum import Enum
import threading
import time
from typing import Any

import cv2
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import numpy as np
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.hardware.manipulators.piper.piper_arm import PiperArm
from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.manipulation.visual_servoing.utils import (
    create_manipulation_visualization,
    is_target_reached,
    select_points_from_depth,
    transform_points_3d,
    update_target_grasp_pose,
)
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.common.utils import find_clicked_detection
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    compose_transforms,
    create_transform_from_6dof,
    matrix_to_pose,
    pose_to_matrix,
)

logger = setup_logger()


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
        current_executed_pose: Pose | None = None,
        current_ee_pose: Pose | None = None,
        current_camera_pose: Pose | None = None,
        target_pose: Pose | None = None,
        waiting_for_reach: bool = False,
        success: bool | None = None,
    ) -> None:
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
    rgb_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]

    # LCM outputs
    viz_image: Out[Image]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        ee_to_camera_6dof: list | None = None,  # type: ignore[type-arg]
        **kwargs,
    ) -> None:
        """
        Initialize manipulation module.

        Args:
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
            workspace_min_radius: Minimum workspace radius in meters
            workspace_max_radius: Maximum workspace radius in meters
            min_grasp_pitch_degrees: Minimum grasp pitch angle (at max radius)
            max_grasp_pitch_degrees: Maximum grasp pitch angle (at min radius)
        """
        super().__init__(**kwargs)

        self.arm = PiperArm()

        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [-0.065, 0.03, -0.095, 0.0, -1.57, 0.0]
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        self.camera_intrinsics = None
        self.detector = None
        self.pbvs = None

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
        self.reached_poses = deque(maxlen=self.pose_history_size)  # type: ignore[var-annotated]
        self.adjustment_count = 0

        # Pose reachability tracking
        self.ee_pose_history = deque(maxlen=20)  # type: ignore[var-annotated]  # Keep history of EE poses
        self.stuck_pose_threshold = 0.001  # 1mm movement threshold
        self.stuck_pose_adjustment_degrees = 5.0
        self.stuck_count = 0
        self.max_stuck_reattempts = 7

        # State for visualization
        self.current_visualization = None
        self.last_detection_3d_array = None
        self.last_detection_2d_array = None

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
        self.arm.gotoObserve()

    @rpc
    def start(self) -> None:
        """Start the manipulation module."""

        unsub = self.rgb_image.subscribe(self._on_rgb_image)
        self._disposables.add(Disposable(unsub))

        unsub = self.depth_image.subscribe(self._on_depth_image)
        self._disposables.add(Disposable(unsub))

        unsub = self.camera_info.subscribe(self._on_camera_info)
        self._disposables.add(Disposable(unsub))

        logger.info("Manipulation module started")

    @rpc
    def stop(self) -> None:
        """Stop the manipulation module."""
        # Stop any running task
        self.stop_event.set()
        if self.task_thread and self.task_thread.is_alive():
            self.task_thread.join(timeout=5.0)

        self.reset_to_idle()

        if self.detector and hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        self.arm.disable()

        logger.info("Manipulation module stopped")

    def _on_rgb_image(self, msg: Image) -> None:
        """Handle RGB image messages."""
        try:
            self.latest_rgb = msg.data
        except Exception as e:
            logger.error(f"Error processing RGB image: {e}")

    def _on_depth_image(self, msg: Image) -> None:
        """Handle depth image messages."""
        try:
            self.latest_depth = msg.data
        except Exception as e:
            logger.error(f"Error processing depth image: {e}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Handle camera info messages."""
        try:
            self.camera_intrinsics = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]  # type: ignore[assignment]

            if self.detector is None:
                self.detector = Detection3DProcessor(self.camera_intrinsics)  # type: ignore[arg-type, assignment]
                self.pbvs = PBVS()  # type: ignore[assignment]
                logger.info("Initialized detection and PBVS processors")

            self.latest_camera_info = msg
        except Exception as e:
            logger.error(f"Error processing camera info: {e}")

    @rpc
    def get_single_rgb_frame(self) -> np.ndarray | None:  # type: ignore[type-arg]
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
            self.arm.softStop()
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
        self,
        target_x: int | None = None,
        target_y: int | None = None,
        place_x: int | None = None,
        place_y: int | None = None,
    ) -> dict[str, Any]:
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

        if target_x is not None and target_y is not None:
            self.target_click = (target_x, target_y)
        if place_x is not None and self.latest_depth is not None:
            points_3d_camera = select_points_from_depth(
                self.latest_depth,
                (place_x, place_y),
                self.camera_intrinsics,
                radius=10,
            )

            if points_3d_camera.size > 0:
                ee_pose = self.arm.get_ee_pose()
                ee_transform = pose_to_matrix(ee_pose)
                camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)

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

    def _run_pick_and_place(self) -> None:
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

                if feedback.success is not None:  # type: ignore[attr-defined]
                    if feedback.success:  # type: ignore[attr-defined]
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

    def set_grasp_stage(self, stage: GraspStage) -> None:
        """Set the grasp stage."""
        self.grasp_stage = stage
        logger.info(f"Grasp stage: {stage.value}")

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

        return pitch_degrees  # type: ignore[no-any-return]

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

    def _check_reach_timeout(self) -> tuple[bool, float]:
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
        if len(self.ee_pose_history) < self.ee_pose_history.maxlen:  # type: ignore[operator]
            return False

        # Extract positions from pose history
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.ee_pose_history]
        )

        # Calculate standard deviation of positions
        std_devs = np.std(positions, axis=0)
        # Check if all standard deviations are below stuck threshold
        is_stuck = np.all(std_devs < self.stuck_pose_threshold)

        return is_stuck  # type: ignore[return-value]

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

        # Get current end-effector pose
        ee_pose = self.arm.get_ee_pose()
        target_pose = self.current_executed_pose

        # Check for timeout - this will fail task and reset if timeout occurred
        timed_out, _time_elapsed = self._check_reach_timeout()
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

    def _update_tracking(self, detection_3d_array: Detection3DArray | None) -> bool:
        """Update tracking with new detections."""
        if not detection_3d_array or not self.pbvs:
            return False

        target_tracked = self.pbvs.update_tracking(detection_3d_array)
        if target_tracked:
            self.target_updated = True
            self.last_valid_target = self.pbvs.get_current_target()
        return target_tracked

    def reset_to_idle(self) -> None:
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

        self.arm.gotoObserve()

    def execute_idle(self) -> None:
        """Execute idle stage."""
        pass

    def execute_pre_grasp(self) -> None:
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

        ee_pose = self.arm.get_ee_pose()  # type: ignore[no-untyped-call]
        dynamic_pitch = self.calculate_dynamic_grasp_pitch(self.pbvs.current_target.bbox.center)  # type: ignore[attr-defined]

        _, _, _, has_target, target_pose = self.pbvs.compute_control(  # type: ignore[attr-defined]
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
                self.waiting_start_time = time.time()  # type: ignore[assignment]
                self.target_updated = False
                self.adjustment_count += 1
                time.sleep(0.2)

    def execute_grasp(self) -> None:
        """Execute grasp stage: move to final grasp position."""
        if self.waiting_for_reach:
            if self.check_reach_and_adjust() and not self.grasp_reached_time:
                self.grasp_reached_time = time.time()  # type: ignore[assignment]
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

            ee_pose = self.arm.get_ee_pose()
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

    def execute_close_and_retract(self) -> None:
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
            self.current_executed_pose = self.final_pregrasp_pose
            self.arm.close_gripper()
            self.waiting_for_reach = True
            self.waiting_start_time = time.time()  # type: ignore[assignment]

    def execute_place(self) -> None:
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
                self.current_executed_pose = place_pose  # type: ignore[assignment]
                self.waiting_for_reach = True
                self.waiting_start_time = time.time()  # type: ignore[assignment]
            else:
                logger.error("Failed to get place target pose")
                self.task_failed = True
                self.overall_success = False  # type: ignore[assignment]

    def execute_retract(self) -> None:
        """Execute retract stage: retract from place position."""
        if self.waiting_for_reach and self.retract_pose:
            if self.check_reach_and_adjust():
                logger.info("Reached retract position")
                logger.info("Returning to observe position")
                self.arm.gotoObserve()
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
                self.overall_success = False  # type: ignore[assignment]

    def capture_and_process(
        self,
    ) -> tuple[np.ndarray | None, Detection3DArray | None, Detection2DArray | None, Pose | None]:  # type: ignore[type-arg]
        """Capture frame from camera data and process detections."""
        if self.latest_rgb is None or self.latest_depth is None or self.detector is None:
            return None, None, None, None

        ee_pose = self.arm.get_ee_pose()
        ee_transform = pose_to_matrix(ee_pose)
        camera_transform = compose_transforms(ee_transform, self.T_ee_to_camera)
        camera_pose = matrix_to_pose(camera_transform)
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

    def update(self) -> dict[str, Any] | None:
        """Main update function that handles capture, processing, control, and visualization."""
        rgb, detection_3d_array, detection_2d_array, camera_pose = self.capture_and_process()
        if rgb is None:
            return None

        self.last_detection_3d_array = detection_3d_array  # type: ignore[assignment]
        self.last_detection_2d_array = detection_2d_array  # type: ignore[assignment]
        if self.target_click:
            x, y = self.target_click
            if self.pick_target(x, y):
                self.target_click = None

        if (
            detection_3d_array
            and self.grasp_stage in [GraspStage.PRE_GRASP, GraspStage.GRASP]
            and not self.waiting_for_reach
        ):
            self._update_tracking(detection_3d_array)
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
        ee_pose = self.arm.get_ee_pose()  # type: ignore[no-untyped-call]
        feedback = Feedback(
            grasp_stage=self.grasp_stage,
            target_tracked=target_tracked,
            current_executed_pose=self.current_executed_pose,
            current_ee_pose=ee_pose,
            current_camera_pose=camera_pose,
            target_pose=self.pbvs.target_grasp_pose if self.pbvs else None,
            waiting_for_reach=self.waiting_for_reach,
            success=self.overall_success,
        )

        if self.task_running:
            self.current_visualization = create_manipulation_visualization(  # type: ignore[assignment]
                rgb, feedback, detection_3d_array, detection_2d_array
            )

            if self.current_visualization is not None:
                self._publish_visualization(self.current_visualization)

        return feedback  # type: ignore[return-value]

    def _publish_visualization(self, viz_image: np.ndarray) -> None:  # type: ignore[type-arg]
        """Publish visualization image to LCM."""
        try:
            viz_rgb = cv2.cvtColor(viz_image, cv2.COLOR_BGR2RGB)
            msg = Image.from_numpy(viz_rgb)
            self.viz_image.publish(msg)
        except Exception as e:
            logger.error(f"Error publishing visualization: {e}")

    def check_target_stabilized(self) -> bool:
        """Check if the commanded poses have stabilized."""
        if len(self.reached_poses) < self.reached_poses.maxlen:  # type: ignore[operator]
            return False

        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in self.reached_poses]
        )
        std_devs = np.std(positions, axis=0)
        return np.all(std_devs < self.pose_stabilization_threshold)  # type: ignore[return-value]

    def get_place_target_pose(self) -> Pose | None:
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

        ee_pose = self.arm.get_ee_pose()

        # Calculate dynamic pitch for place position
        dynamic_pitch = self.calculate_dynamic_grasp_pitch(place_center_pose)

        place_pose = update_target_grasp_pose(
            place_center_pose,
            ee_pose,
            grasp_distance=0.0,
            grasp_pitch_degrees=dynamic_pitch,
        )

        return place_pose
