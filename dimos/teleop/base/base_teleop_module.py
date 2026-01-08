#!/usr/bin/env python3
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
Base Teleoperation Module

Abstract base class for all teleoperation devices that provides:
- Dual-arm controller calibration (left/right)
- Delta pose computation (current - initial)
- LCM topic publishing (delta poses + trigger states)
- Rerun visualization
- Standard RPC interface

Device-specific modules inherit from this and implement their connection logic.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
import time
from typing import TYPE_CHECKING, Any

from dimos.core import Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Vector3
from dimos.msgs.std_msgs import Bool, Float32
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

logger = setup_logger()

try:
    import rerun as rr

    from dimos.dashboard.rerun_init import connect_rerun

    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False


@dataclass
class BaseTeleopConfig(ModuleConfig):
    """Base configuration for teleoperation modules."""

    # Control settings
    position_scale: float = 1.0  # Scale factor for positions
    enable_left_arm: bool = True
    enable_right_arm: bool = True

    # Visualization settings
    visualize_in_rerun: bool = True  # Visualize controller poses in Rerun

    # Safety limits
    safety_limits: bool = True
    max_velocity: float = 0.5  # m/s
    workspace_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "x": (-1.0, 1.0),
            "y": (-0.8, 0.8),
            "z": (0.1, 2.0),
        }
    )


class BaseTeleopModule(Module, ABC):
    """Base class for teleoperation modules.

    Provides common functionality for dual-arm teleoperation:
    - Calibration: capture initial controller poses
    - Delta computation: current - initial
    - Publishing: delta poses and trigger states to LCM
    - Visualization: Rerun integration
    - RPC interface: standard methods for all devices

    Subclasses implement device-specific connection logic and call
    `update_controller_poses()` when new tracking data arrives.

    ## LCM Topics (Output):
    - left_controller_delta: Out[PoseStamped] - Left controller delta pose
    - right_controller_delta: Out[PoseStamped] - Right controller delta pose
    - left_trigger: Out[Float32] - Left trigger/gripper value (0.0-1.0)
    - right_trigger: Out[Float32] - Right trigger/gripper value (0.0-1.0)

    ## RPC Methods:
    - calibrate_vr() -> dict: Calibrate by capturing initial poses
    - reset_calibration() -> dict: Reset calibration
    - is_vr_calibrated() -> bool: Check calibration status
    - get_status() -> dict: Get teleoperation status
    """

    default_config = BaseTeleopConfig

    # LCM Output topics
    left_controller_delta: Out[PoseStamped] = None  # type: ignore[assignment]
    right_controller_delta: Out[PoseStamped] = None  # type: ignore[assignment]
    left_trigger: Out[Float32] = None  # type: ignore[assignment]
    right_trigger: Out[Float32] = None  # type: ignore[assignment]

    def __init__(self, global_config: GlobalConfig | None = None, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # Get global config for Rerun connection
        self._global_config = global_config or GlobalConfig()

        # No RPC dependencies - data-driven activation
        self.rpc_calls = []

        # Calibration state
        self._is_calibrated = False
        self._left_controller_initial: Pose | None = None
        self._right_controller_initial: Pose | None = None

        # Latest controller data (absolute poses)
        self._left_pose: NDArray[np.float32] | None = None
        self._right_pose: NDArray[np.float32] | None = None
        self._left_gripper_value: float = 0.0  # 0.0 to 1.0
        self._right_gripper_value: float = 0.0  # 0.0 to 1.0

        # Tracking counters
        self._tracking_msg_count = 0
        self._publish_count = 0

        # Connection state (for subclasses to update)
        self._connected_clients = 0

        logger.info(f"{self.__class__.__name__} base initialized")

    # =========================================================================
    # Module Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the teleoperation module."""
        logger.info(f"Starting {self.__class__.__name__}...")
        super().start()

        # Connect to Rerun for visualization if enabled
        if (
            self.config.visualize_in_rerun
            and RERUN_AVAILABLE
            and self._global_config.viewer_backend.startswith("rerun")
        ):
            connect_rerun(global_config=self._global_config)
            logger.info("Connected to Rerun for controller visualization")

    # =========================================================================
    # Calibration
    # =========================================================================

    @rpc
    def calibrate_vr(self) -> dict[str, Any]:
        """Calibrate by capturing initial controller poses.

        This is typically called when a calibration button is pressed.
        Captures the current controller poses as the "zero" reference.
        After calibration, delta poses are published.

        Returns:
            Dict with 'success' and optional 'message' or 'error'
        """
        logger.info("Calibrating controllers...")

        try:
            # Check if we have controller data
            if self._left_pose is None and self._right_pose is None:
                return {
                    "success": False,
                    "error": "No controller data received yet. Move controllers and try again.",
                }

            # Capture left controller initial pose
            if self.config.enable_left_arm and self._left_pose is not None:
                left_pose_obj = matrix_to_pose(self._left_pose)

                # Check if pose is valid (not all zeros)
                pose_magnitude = (
                    left_pose_obj.x**2 + left_pose_obj.y**2 + left_pose_obj.z**2
                ) ** 0.5
                if pose_magnitude < 0.001:
                    return {
                        "success": False,
                        "error": "Left controller pose is invalid (all zeros)",
                    }

                self._left_controller_initial = left_pose_obj
                logger.info(
                    f"Captured left controller initial: "
                    f"pos=[{left_pose_obj.x:.3f}, {left_pose_obj.y:.3f}, {left_pose_obj.z:.3f}], "
                    f"rpy=[{left_pose_obj.roll:.3f}, {left_pose_obj.pitch:.3f}, {left_pose_obj.yaw:.3f}]"
                )

            # Capture right controller initial pose
            if self.config.enable_right_arm and self._right_pose is not None:
                right_pose_obj = matrix_to_pose(self._right_pose)

                # Check if pose is valid (not all zeros)
                pose_magnitude = (
                    right_pose_obj.x**2 + right_pose_obj.y**2 + right_pose_obj.z**2
                ) ** 0.5
                if pose_magnitude < 0.001:
                    return {
                        "success": False,
                        "error": "Right controller pose is invalid (all zeros)",
                    }

                self._right_controller_initial = right_pose_obj
                logger.info(
                    f"Captured right controller initial: "
                    f"pos=[{right_pose_obj.x:.3f}, {right_pose_obj.y:.3f}, {right_pose_obj.z:.3f}], "
                    f"rpy=[{right_pose_obj.roll:.3f}, {right_pose_obj.pitch:.3f}, {right_pose_obj.yaw:.3f}]"
                )

            self._is_calibrated = True

            logger.info("Calibration complete. Now streaming delta poses...")
            return {"success": True, "message": "Calibrated - move controllers to control robot"}

        except Exception as e:
            logger.error(f"Calibration failed: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @rpc
    def reset_calibration(self) -> dict[str, Any]:
        """Reset calibration. Stops streaming until recalibrated.

        Returns:
            Dict with 'success' and 'message'
        """
        self._is_calibrated = False
        self._left_controller_initial = None
        self._right_controller_initial = None

        logger.info("Calibration reset. Recalibrate to resume teleoperation...")
        return {"success": True, "message": "Calibration reset - recalibrate to resume"}

    @rpc
    def is_vr_calibrated(self) -> bool:
        """Check if calibrated.

        Returns:
            True if calibrated and streaming deltas
        """
        return self._is_calibrated

    @rpc
    def get_status(self) -> dict:
        """Get current teleoperation status.

        Returns:
            Dictionary with status information
        """
        return {
            "is_calibrated": self._is_calibrated,
            "connected_clients": self._connected_clients,
            "left_arm_enabled": self.config.enable_left_arm,
            "right_arm_enabled": self.config.enable_right_arm,
            "has_left_data": self._left_pose is not None,
            "has_right_data": self._right_pose is not None,
            "left_gripper_value": self._left_gripper_value,
            "right_gripper_value": self._right_gripper_value,
        }

    # =========================================================================
    # Controller Data Processing (called by subclasses)
    # =========================================================================

    def update_controller_poses(
        self,
        left_pose: NDArray[np.float32],
        right_pose: NDArray[np.float32],
        left_gripper: float,
        right_gripper: float,
    ) -> None:
        """Update controller poses and publish deltas if calibrated.

        This method should be called by subclasses when new tracking data arrives
        from the device-specific connection.

        Args:
            left_pose: 4x4 transformation matrix for left controller
            right_pose: 4x4 transformation matrix for right controller
            left_gripper: Left gripper value (0.0-1.0)
            right_gripper: Right gripper value (0.0-1.0)
        """
        # Track how many tracking messages we've received
        self._tracking_msg_count += 1

        # Log first few tracking messages to confirm data is arriving
        if self._tracking_msg_count <= 3:
            logger.info(f"Received tracking data #{self._tracking_msg_count}")

        # Store absolute poses
        self._left_pose = left_pose
        self._right_pose = right_pose

        # Store gripper values (0.0 to 1.0)
        self._left_gripper_value = float(left_gripper)
        self._right_gripper_value = float(right_gripper)

        # Only stream deltas if calibrated
        if not self._is_calibrated:
            # Only log this warning once every 100 messages to avoid spam
            if self._tracking_msg_count <= 1 or self._tracking_msg_count % 100 == 0:
                logger.warning("Not calibrated. Calibrate to start teleoperation.")
            return

        # Compute and publish deltas
        self._compute_and_publish_deltas()

    def _compute_and_publish_deltas(self) -> None:
        """Compute and publish delta poses (current - initial)."""
        # Track publish count for logging
        self._publish_count += 1

        try:
            current_time = time.time()

            # Left controller delta
            if self.config.enable_left_arm and self._left_controller_initial is not None:
                if self.left_controller_delta and hasattr(self.left_controller_delta, "publish"):
                    if self._left_pose is not None:
                        left_pose_obj = matrix_to_pose(self._left_pose)
                        delta_pose_stamped = self._compute_delta(
                            left_pose_obj,
                            self._left_controller_initial,
                            current_time,
                            f"{self.__class__.__name__.lower()}_left_controller_delta",
                        )

                        try:
                            self.left_controller_delta.publish(delta_pose_stamped)

                            # Visualize in Rerun (absolute pose)
                            self._visualize_controller_in_rerun(left_pose_obj, "left")

                            # Log periodically
                            if self._publish_count <= 5 or self._publish_count % 100 == 0:
                                logger.info(
                                    f"Published left delta #{self._publish_count}: "
                                    f"pos=[{delta_pose_stamped.position.x:.3f}, {delta_pose_stamped.position.y:.3f}, {delta_pose_stamped.position.z:.3f}], "
                                    f"rpy=[{delta_pose_stamped.roll:.3f}, {delta_pose_stamped.pitch:.3f}, {delta_pose_stamped.yaw:.3f}], "
                                    f"frame_id={delta_pose_stamped.frame_id}"
                                )
                        except Exception as e:
                            logger.error(f"Failed to publish left delta: {e}")

            # Right controller delta
            if self.config.enable_right_arm and self._right_controller_initial is not None:
                if self.right_controller_delta and hasattr(self.right_controller_delta, "publish"):
                    if self._right_pose is not None:
                        right_pose_obj = matrix_to_pose(self._right_pose)
                        delta_pose_stamped = self._compute_delta(
                            right_pose_obj,
                            self._right_controller_initial,
                            current_time,
                            f"{self.__class__.__name__.lower()}_right_controller_delta",
                        )

                        try:
                            self.right_controller_delta.publish(delta_pose_stamped)

                            # Visualize in Rerun (absolute pose)
                            self._visualize_controller_in_rerun(right_pose_obj, "right")

                            if self._publish_count <= 5 or self._publish_count % 100 == 0:
                                logger.info(
                                    f"Published right delta #{self._publish_count}: "
                                    f"pos=[{delta_pose_stamped.position.x:.3f}, {delta_pose_stamped.position.y:.3f}, {delta_pose_stamped.position.z:.3f}], "
                                    f"frame_id={delta_pose_stamped.frame_id}"
                                )
                        except Exception as e:
                            logger.error(f"Failed to publish right delta: {e}")

            # Publish gripper values (0.0 to 1.0)
            if self.left_trigger and hasattr(self.left_trigger, "publish"):
                try:
                    self.left_trigger.publish(Float32(data=self._left_gripper_value))
                    # Visualize gripper value in Rerun
                    self._visualize_gripper_in_rerun(self._left_gripper_value, "left")
                except Exception as e:
                    logger.debug(f"Failed to publish left gripper: {e}")

            if self.right_trigger and hasattr(self.right_trigger, "publish"):
                try:
                    self.right_trigger.publish(Float32(data=self._right_gripper_value))
                    # Visualize gripper value in Rerun
                    self._visualize_gripper_in_rerun(self._right_gripper_value, "right")
                except Exception as e:
                    logger.debug(f"Failed to publish right gripper: {e}")

        except Exception as e:
            logger.error(f"Error computing/publishing delta poses: {e}")

    def _compute_delta(
        self, current: Pose, initial: Pose, timestamp: float, frame_id: str
    ) -> PoseStamped:
        """Compute delta pose: current - initial.

        For position: simple subtraction
        For orientation: delta_quat = current * inverse(initial)

        Args:
            current: Current controller pose
            initial: Initial controller pose (reference)
            timestamp: Timestamp for the delta pose
            frame_id: Frame ID for the delta pose

        Returns:
            Delta pose as PoseStamped (position delta + orientation delta)
        """
        # Position delta
        delta_x = current.x - initial.x
        delta_y = current.y - initial.y
        delta_z = current.z - initial.z

        # Orientation delta: delta_quat = current * inverse(initial)
        delta_quat = current.orientation * initial.orientation.inverse()

        delta_pose = Pose(
            position=Vector3(delta_x, delta_y, delta_z),
            orientation=delta_quat,
        )

        return PoseStamped(
            ts=timestamp,
            frame_id=frame_id,
            position=delta_pose.position,
            orientation=delta_pose.orientation,
        )

    # =========================================================================
    # Rerun Visualization
    # =========================================================================

    def _visualize_controller_in_rerun(self, controller_pose: Pose, arm_side: str) -> None:
        """Visualize controller absolute pose in Rerun.

        Args:
            controller_pose: Absolute controller pose in robot space
            arm_side: "left" or "right"
        """
        if not (
            self.config.visualize_in_rerun
            and RERUN_AVAILABLE
            and self._global_config.viewer_backend.startswith("rerun")
        ):
            return

        try:
            # Convert to PoseStamped for Rerun visualization
            controller_pose_stamped = PoseStamped(
                ts=time.time(),
                frame_id=f"world/teleop/{arm_side}_controller",
                position=controller_pose.position,
                orientation=controller_pose.orientation,
            )
            # Log absolute controller pose transform
            rr.log(
                f"world/teleop/{arm_side}_controller",
                controller_pose_stamped.to_rerun(),
            )
            # Log coordinate frame axes to visualize the transform
            rr.log(
                f"world/teleop/{arm_side}_controller",
                rr.Arrows3D(  # type: ignore[attr-defined]
                    vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                    origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
            )
        except Exception as e:
            logger.debug(f"Failed to log {arm_side} controller to Rerun: {e}")

    def _visualize_gripper_in_rerun(self, gripper_value: float, arm_side: str) -> None:
        """Visualize gripper/trigger value in Rerun as a scalar time series.

        Args:
            gripper_value: Gripper value (0.0-1.0) from VR controller
            arm_side: "left" or "right"
        """
        if not (
            self.config.visualize_in_rerun
            and RERUN_AVAILABLE
            and self._global_config.viewer_backend.startswith("rerun")
        ):
            return

        try:
            # Log gripper value as scalar time series
            rr.log(
                f"world/teleop/{arm_side}_controller/gripper",
                rr.Scalars(gripper_value),  # type: ignore[attr-defined]
            )
        except Exception as e:
            logger.debug(f"Failed to log {arm_side} gripper to Rerun: {e}")
