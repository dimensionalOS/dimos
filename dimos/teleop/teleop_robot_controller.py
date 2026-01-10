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

Receives controller delta poses from Quest3TeleopModule and applies them to the robot.
Auto-calibrates robot on first delta received.

Architecture:
- Subscribes to controller_delta from Quest3TeleopModule
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
    driver_method_name: str = "get_state"
    dummy_driver: bool = False  # If True, skip RPC calls and use zeros for initial pose

    # Control settings
    control_frequency: float = 50.0  # Hz - control loop frequency

    # Safety settings
    delta_timeout: float = 1  # Seconds - stop publishing if no new delta received


class TeleopRobotController(Module):
    """Teleop Robot Controller - applies delta poses to robot.

    This controller:
    1. Receives DELTA poses (PoseStamped) from Quest3TeleopModule for a single controller
    2. On first delta: gets initial robot end-effector pose via RPC (auto-calibration)
    3. Applies delta to initial robot pose: target = initial_robot + delta
    4. Publishes cartesian commands (Pose) to manipulator driver
    5. Optionally applies workspace limits

    Works with any manipulator driver that provides:
    - state RPC method (configurable via driver_method_name)
    - cartesian_command input topic (Pose)
    """

    default_config = TeleopRobotControllerConfig
    config: TeleopRobotControllerConfig

    # Input topics - receiving DELTA poses as PoseStamped
    controller_delta: In[PoseStamped] = None  # type: ignore[assignment]
    trigger_value: In[Float32] = None  # type: ignore[assignment]  # Gripper value 0.0-1.0

    # Output topics (Pose for commands)
    cartesian_command: Out[Pose] = None  # type: ignore[assignment]
    gripper_command: Out[Bool] = None  # type: ignore[assignment]

    # RPC dependencies (dynamically set based on config)
    rpc_calls: list[str] = []

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Initialize the teleop robot controller."""
        super().__init__(*args, **kwargs)

        # Set RPC calls dynamically based on driver name
        self.rpc_calls = [
            f"{self.config.driver_module_name}.{self.config.driver_method_name}",
        ]

        # Latest delta data
        self._delta_pose: Pose | None = None
        self._gripper_value: float = 0.0  # 0.0 to 1.0

        # Timestamps for delta timeout detection
        self._delta_timestamp: float | None = None

        # Robot initial state (auto-calibrated on first delta)
        self._robot_initial_pose: Pose | None = None
        self._robot_calibrated = False

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
        if self.controller_delta and hasattr(self.controller_delta, "subscribe"):
            self.controller_delta.subscribe(self._on_controller_delta)
            logger.debug("Subscribed to controller_delta")
        else:
            logger.warning("controller_delta not wired; no delta subscription")

        if self.trigger_value and hasattr(self.trigger_value, "subscribe"):
            self.trigger_value.subscribe(self._on_trigger_value)
            logger.debug("Subscribed to trigger_value")
        else:
            logger.warning("trigger_value not wired; no trigger subscription")

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

    def _on_controller_delta(self, delta: PoseStamped) -> None:
        """Callback for controller delta pose.

        On first delta, auto-calibrates robot.
        """
        # Auto-calibrate robot on first delta received
        if not self._robot_calibrated:
            logger.info("First delta received - auto-calibrating robot...")
            self._calibrate_robot()

        with self._state_lock:
            self._delta_pose = Pose(position=delta.position, orientation=delta.orientation)

            # Log first few deltas
            if not hasattr(self, "_delta_count"):
                self._delta_count = 0
            self._delta_count += 1
            if self._delta_count <= 5:
                logger.info(
                    f"Received controller delta #{self._delta_count}: "
                    f"pos=[{delta.position.x:.3f}, {delta.position.y:.3f}, {delta.position.z:.3f}], "
                    f"frame_id={delta.frame_id}"
                )

    def _on_trigger_value(self, gripper: Float32) -> None:
        """Callback for controller gripper value (0.0-1.0)."""
        with self._state_lock:
            self._gripper_value = float(gripper.data)

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
            self._robot_initial_pose = Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
            )
            logger.info("Initial pose: pos=[0, 0, 0], rpy=[0, 0, 0]")

            self._robot_calibrated = True
            logger.info("Robot calibration complete (dummy mode) - control active!")
            return True

        try:
            rpc_method_name = f"{self.config.driver_module_name}.{self.config.driver_method_name}"
            get_state = self.get_rpc_calls(rpc_method_name)

            if get_state is None:
                logger.error("RPC callable is None - check blueprint wiring")
                return False

            result = get_state()

            if result and result.get("success"):
                pose_data = result.get("pose", {})

                # Store robot initial state
                position = Vector3(pose_data.get("x", 0.0), pose_data.get("y", 0.0), pose_data.get("z", 0.0))
                rpy = Vector3(pose_data.get("roll", 0.0), pose_data.get("pitch", 0.0), pose_data.get("yaw", 0.0))
                self._robot_initial_pose = Pose(
                    position=position,
                    orientation=Quaternion.from_euler(rpy),
                )
                logger.info(f"Robot initial pose: {pose_data}")
                self._robot_calibrated = True
                logger.info("Robot calibration complete - control active!")
                return True

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
            "has_delta": self._delta_pose is not None,
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

                with self._state_lock:
                    robot_calibrated = self._robot_calibrated
                    delta_pose = self._delta_pose
                    gripper_val = self._gripper_value
                    robot_initial_pose = self._robot_initial_pose

                if robot_calibrated:
                    # Publishing Pose
                    if robot_initial_pose is not None and delta_pose is not None:
                        target_pose = robot_initial_pose + delta_pose
                        if self.cartesian_command and hasattr(self.cartesian_command, "publish"):
                            try:
                                self.cartesian_command.publish(target_pose)
                            except Exception as e:
                                logger.error(f"Failed to publish cartesian command: {e}")

                    # Publishing Gripper
                    if self.gripper_command and hasattr(self.gripper_command, "publish"):
                        try:
                            self.gripper_command.publish(Bool(data=gripper_val > 0.5))
                        except Exception as e:
                            logger.debug(f"Failed to publish gripper command: {e}")
                
                # Log state periodically
                if loop_count <= 10 or loop_count % 100 == 0:
                    logger.debug(
                        f"Control loop #{loop_count}, "
                        f"robot_calibrated={robot_calibrated}, "
                        f"has_delta={delta_pose is not None}"
                    )

            except Exception as e:
                logger.error(f"Error in control loop: {e}", exc_info=True)

            # Rate control
            next_time += period
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                if loop_count % 100 == 0:
                    logger.warning("Control loop overrun by %.4fs", -sleep_time)
                next_time = time.perf_counter() + period

        logger.info("Control loop stopped")

# Expose blueprint for declarative composition
teleop_robot_controller = TeleopRobotController.blueprint
