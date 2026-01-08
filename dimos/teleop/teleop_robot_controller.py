# Copyright 2025-2026 Dimensional Inc.
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
Teleop Robot Controller

Receives DELTA poses from Quest3TeleopModule and applies them to the robot.
Auto-calibrates robot on first delta received.

Architecture:
- Subscribes to left_controller_delta and right_controller_delta from Quest3TeleopModule
- On first delta received: gets robot's initial end-effector pose via RPC
- Calculates: target_pose = initial_robot_pose + delta
- Publishes cartesian_command (Pose) to driver
"""

from dataclasses import dataclass
import threading
import time
import traceback
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Vector3
from dimos.msgs.std_msgs import Bool, Float32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class TeleopRobotControllerConfig(ModuleConfig):
    """Configuration for Teleop Robot Controller."""

    # Driver settings
    driver_module_name: str = "RobotDriver"  # Name of the driver module to get robot pose from
    dummy_driver: bool = False  # If True, skip RPC calls and use zeros for initial pose

    # Control settings
    control_frequency: float = 50.0  # Hz - control loop frequency
    enable_left_arm: bool = True
    enable_right_arm: bool = True

    # Safety settings
    delta_timeout: float = 1  # Seconds - stop publishing if no new delta received


class TeleopRobotController(Module):
    """Teleop Robot Controller - applies delta poses to robot.

    This controller:
    1. Receives DELTA poses (PoseStamped) from Quest3TeleopModule
    2. On first delta: gets initial robot end-effector pose via RPC (auto-calibration)
    3. Applies delta to initial robot pose: target = initial_robot + delta
    4. Publishes cartesian commands (Pose) to manipulator driver
    5. Optionally applies workspace limits

    Works with any manipulator driver that provides:
    - get_cartesian_state RPC method
    - cartesian_command input topic (Pose)
    """

    default_config = TeleopRobotControllerConfig
    config: TeleopRobotControllerConfig

    # Input topics - receiving DELTA poses as PoseStamped
    left_controller_delta: In[PoseStamped] = None  # type: ignore[assignment]
    right_controller_delta: In[PoseStamped] = None  # type: ignore[assignment]
    left_trigger: In[Float32] = None  # type: ignore[assignment]  # Gripper value 0.0-1.0
    right_trigger: In[Float32] = None  # type: ignore[assignment]  # Gripper value 0.0-1.0

    # Output topics (Pose for commands)
    left_cartesian_command: Out[Pose] = None  # type: ignore[assignment]
    right_cartesian_command: Out[Pose] = None  # type: ignore[assignment]
    left_gripper_command: Out[Bool] = None  # type: ignore[assignment]
    right_gripper_command: Out[Bool] = None  # type: ignore[assignment]

    # RPC dependencies (dynamically set based on config)
    rpc_calls: list[str] = []

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize the teleop robot controller."""
        super().__init__(*args, **kwargs)

        # Set RPC calls dynamically based on driver name
        driver_name = self.config.driver_module_name
        self.rpc_calls = [
            f"{driver_name}.get_state",
        ]

        # Latest delta data
        self._left_delta: PoseStamped | None = None
        self._right_delta: PoseStamped | None = None
        self._left_gripper_value: float = 0.0  # 0.0 to 1.0
        self._right_gripper_value: float = 0.0  # 0.0 to 1.0

        # Timestamps for delta timeout detection
        self._left_delta_timestamp: float | None = None
        self._right_delta_timestamp: float | None = None

        # Robot initial state (auto-calibrated on first delta)
        self._left_robot_initial_position: Vector3 | None = None
        self._left_robot_initial_rpy: Vector3 | None = None
        self._right_robot_initial_position: Vector3 | None = None
        self._right_robot_initial_rpy: Vector3 | None = None
        self._robot_calibrated = False

        # State for unwrapped logging
        self._last_logged_rpy: dict[str, NDArray[np.float64]] = {}

        # Control loop
        self._control_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # State lock
        self._state_lock = threading.Lock()

        logger.info("TeleopRobotController initialized - waiting for delta poses")

    # =========================================================================
    # Module Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the teleop robot controller."""
        logger.info("Starting TeleopRobotController...")
        super().start()

        # Subscribe to input topics (delta poses)
        if self.left_controller_delta and hasattr(self.left_controller_delta, "subscribe"):
            self.left_controller_delta.subscribe(self._on_left_delta)
            logger.debug("Subscribed to left_controller_delta")

        if self.right_controller_delta and hasattr(self.right_controller_delta, "subscribe"):
            self.right_controller_delta.subscribe(self._on_right_delta)
            logger.debug("Subscribed to right_controller_delta")

        if self.left_trigger and hasattr(self.left_trigger, "subscribe"):
            self.left_trigger.subscribe(self._on_left_trigger)
            logger.debug("Subscribed to left_trigger")

        if self.right_trigger and hasattr(self.right_trigger, "subscribe"):
            self.right_trigger.subscribe(self._on_right_trigger)
            logger.debug("Subscribed to right_trigger")

        # Start control loop
        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="TeleopRobotController"
        )
        self._control_thread.start()

        logger.info("TeleopRobotController started - waiting for delta poses to auto-calibrate")

    @rpc
    def stop(self) -> None:
        """Stop the teleop Robot controller."""
        logger.info("Stopping TeleopRobotController")

        # Stop control loop
        self._stop_event.set()
        if self._control_thread:
            self._control_thread.join(timeout=2.0)

        super().stop()
        logger.info("TeleopRobotController stopped")

    # =========================================================================
    # Input Callbacks
    # =========================================================================

    def _on_left_delta(self, delta: PoseStamped) -> None:
        """Callback for left controller delta pose.

        On first delta, auto-calibrates robot.
        """
        if not self.config.enable_left_arm:
            return

        # Auto-calibrate robot on first delta received
        if not self._robot_calibrated:
            logger.info("First delta received - auto-calibrating robot...")
            self._calibrate_robot()

        with self._state_lock:
            self._left_delta = delta
            self._left_delta_timestamp = time.time()

            # Log first few deltas
            if not hasattr(self, "_left_delta_count"):
                self._left_delta_count = 0
            self._left_delta_count += 1
            if self._left_delta_count <= 5:
                logger.info(
                    f"Received left delta #{self._left_delta_count}: "
                    f"pos=[{delta.position.x:.3f}, {delta.position.y:.3f}, {delta.position.z:.3f}], "
                    f"frame_id={delta.frame_id}"
                )

    def _on_right_delta(self, delta: PoseStamped) -> None:
        """Callback for right controller delta pose.

        On first delta, auto-calibrates robot.
        """
        if not self.config.enable_right_arm:
            return

        # Auto-calibrate robot on first delta received
        if not self._robot_calibrated:
            logger.info("First delta received - auto-calibrating robot...")
            self._calibrate_robot()

        with self._state_lock:
            self._right_delta = delta
            self._right_delta_timestamp = time.time()

            if not hasattr(self, "_right_delta_count"):
                self._right_delta_count = 0
            self._right_delta_count += 1
            if self._right_delta_count <= 5:
                logger.info(
                    f"Received right delta #{self._right_delta_count}: "
                    f"pos=[{delta.position.x:.3f}, {delta.position.y:.3f}, {delta.position.z:.3f}], "
                    f"frame_id={delta.frame_id}"
                )

    def _on_left_trigger(self, gripper: Float32) -> None:
        """Callback for left gripper value (0.0-1.0)."""
        with self._state_lock:
            self._left_gripper_value = float(gripper.data)

    def _on_right_trigger(self, gripper: Float32) -> None:
        """Callback for right gripper value (0.0-1.0)."""
        with self._state_lock:
            self._right_gripper_value = float(gripper.data)

    # =========================================================================
    # Robot Calibration (Auto-triggered on first delta)
    # =========================================================================

    def _calibrate_robot(self) -> bool:
        """Calibrate robot by getting its initial pose via RPC.

        Called automatically when first delta pose is received.
        If dummy_driver=True, uses zeros instead of making RPC calls.

        Returns:
            True if calibration successful, False otherwise
        """
        logger.info("Calibrating robot (getting initial pose)...")

        # If dummy_driver is enabled, use zeros for initial pose
        if self.config.dummy_driver:
            logger.info("Dummy driver mode - using zeros for initial pose")
            if self.config.enable_left_arm:
                self._left_robot_initial_position = Vector3(0.0, 0.0, 0.0)
                self._left_robot_initial_rpy = Vector3(0.0, 0.0, 0.0)
                logger.info("Left arm initial pose: pos=[0, 0, 0], rpy=[0, 0, 0]")

            if self.config.enable_right_arm:
                self._right_robot_initial_position = Vector3(0.0, 0.0, 0.0)
                self._right_robot_initial_rpy = Vector3(0.0, 0.0, 0.0)
                logger.info("Right arm initial pose: pos=[0, 0, 0], rpy=[0, 0, 0]")

            self._robot_calibrated = True
            logger.info("Robot calibration complete (dummy mode) - control active!")
            return True

        try:
            driver_name = self.config.driver_module_name
            rpc_method_name = f"{driver_name}.get_state"

            get_state = self.get_rpc_calls(rpc_method_name)

            if get_state is None:
                logger.error("RPC callable is None - check blueprint wiring")
                return False

            result = get_state()

            if result and result.get("success"):
                pose_data = result.get("pose", {})

                # Store robot initial state
                if self.config.enable_left_arm:
                    self._left_robot_initial_position = Vector3(
                        pose_data.get("x", 0.0), pose_data.get("y", 0.0), pose_data.get("z", 0.0)
                    )
                    self._left_robot_initial_rpy = Vector3(
                        pose_data.get("roll", 0.0),
                        pose_data.get("pitch", 0.0),
                        pose_data.get("yaw", 0.0),
                    )
                    logger.info(
                        f"Robot initial pose: "
                        f"pos=[{self._left_robot_initial_position.x:.3f}, "
                        f"{self._left_robot_initial_position.y:.3f}, "
                        f"{self._left_robot_initial_position.z:.3f}], "
                        f"rpy=[{self._left_robot_initial_rpy.x:.3f}, "
                        f"{self._left_robot_initial_rpy.y:.3f}, "
                        f"{self._left_robot_initial_rpy.z:.3f}]"
                    )

                if self.config.enable_right_arm:
                    self._right_robot_initial_position = Vector3(
                        pose_data.get("x", 0.0), pose_data.get("y", 0.0), pose_data.get("z", 0.0)
                    )
                    self._right_robot_initial_rpy = Vector3(
                        pose_data.get("roll", 0.0),
                        pose_data.get("pitch", 0.0),
                        pose_data.get("yaw", 0.0),
                    )

                self._robot_calibrated = True
                logger.info("Robot calibration complete - control active!")
                return True
            else:
                error_msg = f"Failed to get robot cartesian state: {result}"
                logger.error(error_msg)
                return False

        except Exception as e:
            logger.error(f"Robot calibration failed: {e}", exc_info=True)
            traceback.print_exc()
            return False

    @rpc
    def recalibrate_robot(self) -> dict[str, Any]:
        """Manually recalibrate robot (get new initial pose).

        Returns:
            Dict with 'success' and optional 'message' or 'error'
        """
        self._robot_calibrated = False
        success = self._calibrate_robot()

        if success:
            return {"success": True, "message": "Robot recalibrated"}
        else:
            return {"success": False, "error": "Recalibration failed"}

    @rpc
    def is_robot_calibrated(self) -> bool:
        """Check if robot is calibrated.

        Returns:
            True if calibrated, False otherwise
        """
        return self._robot_calibrated

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get controller status.

        Returns:
            Dict with status information
        """
        return {
            "robot_calibrated": self._robot_calibrated,
            "has_left_delta": self._left_delta is not None,
            "has_right_delta": self._right_delta is not None,
            "left_arm_enabled": self.config.enable_left_arm,
            "right_arm_enabled": self.config.enable_right_arm,
        }

    # =========================================================================
    # Control Loop
    # =========================================================================

    def _control_loop(self) -> None:
        """Main control loop - applies deltas to robot initial pose."""
        logger.info("Control loop started")
        period = 1.0 / self.config.control_frequency
        next_time = time.perf_counter() + period

        loop_count = 0
        while not self._stop_event.is_set():
            try:
                loop_count += 1

                # Get latest state
                current_time = time.time()
                with self._state_lock:
                    left_delta = self._left_delta
                    right_delta = self._right_delta
                    robot_calibrated = self._robot_calibrated
                    left_delta_time = self._left_delta_timestamp
                    right_delta_time = self._right_delta_timestamp

                # Check for stale deltas (timeout)
                delta_timeout = self.config.delta_timeout
                if left_delta_time is not None:
                    if current_time - left_delta_time > delta_timeout:
                        logger.debug("Left delta timed out - clearing")
                        with self._state_lock:
                            self._left_delta = None
                            self._left_delta_timestamp = None
                            self._left_gripper_value = 0.0  # Clear gripper value too
                        left_delta = None

                if right_delta_time is not None:
                    if current_time - right_delta_time > delta_timeout:
                        logger.debug("Right delta timed out - clearing")
                        with self._state_lock:
                            self._right_delta = None
                            self._right_delta_timestamp = None
                            self._right_gripper_value = 0.0  # Clear gripper value too
                        right_delta = None

                # Log state periodically
                if loop_count <= 10 or loop_count % 100 == 0:
                    logger.debug(
                        f"Control loop #{loop_count}: robot_calibrated={robot_calibrated}, "
                        f"left_delta={left_delta is not None}, right_delta={right_delta is not None}"
                    )

                # Only process if robot is calibrated
                if robot_calibrated:
                    # Process left arm
                    if self.config.enable_left_arm and left_delta is not None:
                        self._apply_delta(left_delta, "left")

                    # Process right arm
                    if self.config.enable_right_arm and right_delta is not None:
                        self._apply_delta(right_delta, "right")

                    # Publish gripper commands (only when calibrated)
                    # Convert float (0.0-1.0) to Bool (threshold at 0.5)
                    with self._state_lock:
                        left_gripper_val = self._left_gripper_value
                        right_gripper_val = self._right_gripper_value

                    if self.config.enable_left_arm:
                        if self.left_gripper_command and hasattr(
                            self.left_gripper_command, "publish"
                        ):
                            try:
                                # Convert float to bool: > 0.5 = closed
                                left_gripper_closed = left_gripper_val > 0.5
                                self.left_gripper_command.publish(Bool(data=left_gripper_closed))
                            except Exception as e:
                                logger.debug(f"Failed to publish left gripper command: {e}")

                    if self.config.enable_right_arm:
                        if self.right_gripper_command and hasattr(
                            self.right_gripper_command, "publish"
                        ):
                            try:
                                # Convert float to bool: > 0.5 = closed
                                right_gripper_closed = right_gripper_val > 0.5
                                self.right_gripper_command.publish(Bool(data=right_gripper_closed))
                            except Exception as e:
                                logger.debug(f"Failed to publish right gripper command: {e}")

            except Exception as e:
                logger.error(f"Error in control loop: {e}", exc_info=True)

            # Rate control
            next_time += period
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.perf_counter() + period

        logger.info("Control loop stopped")

    def _apply_delta(self, delta: PoseStamped, arm_side: str) -> None:
        """Apply delta pose to robot initial pose and publish command.

        Calculates: target_pose = initial_robot_pose + delta

        Args:
            delta: Delta pose (PoseStamped) from Quest3TeleopModule
            arm_side: "left" or "right"
        """
        try:
            # Track command count for logging
            if not hasattr(self, "_command_count"):
                self._command_count = 0
            self._command_count += 1

            # Get robot initial poses for this arm
            if arm_side == "left":
                robot_initial_pos = self._left_robot_initial_position
                robot_initial_rpy = self._left_robot_initial_rpy
            else:
                robot_initial_pos = self._right_robot_initial_position
                robot_initial_rpy = self._right_robot_initial_rpy

            if robot_initial_pos is None or robot_initial_rpy is None:
                logger.debug(f"{arm_side.capitalize()} arm not calibrated, skipping")
                return

            # Calculate target position: initial_robot + delta
            target_x = robot_initial_pos.x + delta.position.x
            target_y = robot_initial_pos.y + delta.position.y
            target_z = robot_initial_pos.z + delta.position.z

            # Calculate target orientation using RPY addition:
            # Convert delta quaternion to Euler, then add to robot initial RPY
            delta_euler = delta.orientation.to_euler()

            target_roll = robot_initial_rpy.x + delta_euler.x
            target_pitch = robot_initial_rpy.y + delta_euler.y
            target_yaw = robot_initial_rpy.z + delta_euler.z

            # Wrap angles to [-π, π]
            def wrap_angle(angle: float) -> float:
                return float(np.arctan2(np.sin(angle), np.cos(angle)))

            target_roll = wrap_angle(target_roll)
            target_pitch = wrap_angle(target_pitch)
            target_yaw = wrap_angle(target_yaw)

            # Create target orientation quaternion from final RPY
            target_rpy = Vector3(target_roll, target_pitch, target_yaw)
            target_orientation = Quaternion.from_euler(target_rpy)

            # Create target pose
            target_pose = Pose(
                position=Vector3(target_x, target_y, target_z),
                orientation=target_orientation,
            )

            # Log and publish
            if arm_side == "left":
                # Unwrap Euler angles for smooth logging (handles +pi and -pi discontinuities)
                current_rpy = np.array([target_pose.roll, target_pose.pitch, target_pose.yaw])
                if arm_side in self._last_logged_rpy:
                    prev_rpy = self._last_logged_rpy[arm_side]
                    diff = current_rpy - prev_rpy
                    diff = (diff + np.pi) % (2 * np.pi) - np.pi
                    current_rpy = prev_rpy + diff
                self._last_logged_rpy[arm_side] = current_rpy

                # Publish to robot
                if self.left_cartesian_command and hasattr(self.left_cartesian_command, "publish"):
                    try:
                        self.left_cartesian_command.publish(target_pose)
                    except Exception as e:
                        logger.error(f"Failed to publish left cartesian command: {e}")

            elif arm_side == "right":
                # Unwrap Euler angles for smooth logging (handles +pi and -pi discontinuities)
                current_rpy = np.array([target_pose.roll, target_pose.pitch, target_pose.yaw])
                if arm_side in self._last_logged_rpy:
                    prev_rpy = self._last_logged_rpy[arm_side]
                    diff = current_rpy - prev_rpy
                    diff = (diff + np.pi) % (2 * np.pi) - np.pi
                    current_rpy = prev_rpy + diff
                self._last_logged_rpy[arm_side] = current_rpy

                # Publish to robot
                if self.right_cartesian_command and hasattr(
                    self.right_cartesian_command, "publish"
                ):
                    try:
                        self.right_cartesian_command.publish(target_pose)
                    except Exception as e:
                        logger.error(f"Failed to publish right cartesian command: {e}")

        except Exception as e:
            logger.error(f"Error applying {arm_side} delta: {e}", exc_info=True)


# Expose blueprint for declarative composition
teleop_robot_controller = TeleopRobotController.blueprint
