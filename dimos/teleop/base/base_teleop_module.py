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
- Multi-controller calibration
- Delta pose computation (current - initial)
- LCM topic publishing (delta poses + trigger states)
- Rerun visualization
- Standard RPC interface

Device-specific modules inherit from this and implement their connection logic.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
import time
from typing import TYPE_CHECKING, Any

from dimos.core import Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.msgs.std_msgs import Float32
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
    num_inputs: int = 1  # Number of inputs (controllers)
    enable_inputs: list[bool] = field(default_factory=list)
    input_labels: list[str] = field(default_factory=list)

    # Visualization settings
    visualize_in_rerun: bool = True  # Visualize controller poses in Rerun
    log_input_data: bool = False  # Log input pose/gripper data periodically
    log_input_data_interval: int = 100  # Log every N publishes when enabled

    # Safety limits
    safety_limits: bool = True
    position_scale: float = 1.0  # Scale factor for positions TODO: Implement proportional scaling
    max_velocity: float = 0.5  # m/s
    workspace_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "x": (-1.0, 1.0),
            "y": (-0.8, 0.8),
            "z": (0.1, 2.0),
        }
    )


class TeleopStatusKey(str, Enum):
    """Status dictionary keys (controller_* entries are indexed by controller number)."""

    IS_CALIBRATED = "is_calibrated"
    CONTROLLER_ENABLED = "controller_{index}_enabled"
    CONTROLLER_HAS_DATA = "controller_{index}_has_data"
    CONTROLLER_GRIPPER_VALUE = "controller_{index}_gripper_value"
    CONTROLLER_LABEL = "controller_{index}_label"

class BaseTeleopModule(Module, ABC):
    """Base class for teleoperation modules.

    Provides common functionality for multi-controller teleoperation:
    - Calibration: capture initial controller poses
    - Delta computation: current - initial
    - Publishing: delta poses and trigger states to LCM
    - Visualization: Rerun integration
    - RPC interface: standard methods for all devices

    Subclasses implement device-specific connection logic and call
    `update_controller_poses()` when new tracking data arrives.

    ## LCM Topics (Output):
    - controller_delta_{i}: Out[PoseStamped] - Controller i delta pose
    - trigger_value_{i}: Out[Float32] - Controller i trigger/gripper value (0.0-1.0)

    ## RPC Methods:
    - calibrate() -> dict: Calibrate by capturing initial poses
    - reset_calibration() -> dict: Reset calibration
    - is_calibrated() -> bool: Check if calibrated
    - get_status() -> dict: Get teleoperation status
    """

    default_config = BaseTeleopConfig
    config: BaseTeleopConfig

    # LCM Output topics
    controller_delta_0: Out[PoseStamped] = None  # type: ignore
    trigger_value_0: Out[Float32] = None  # type: ignore

    def __init__(
        self, global_config: GlobalConfig | None = None, *args: Any, **kwargs: Any
    ) -> None:
        super().__init__(*args, **kwargs)

        # Get global config for Rerun connection
        self._global_config = global_config or GlobalConfig()

        # Calibration state
        self._is_calibrated = False
        self._initial_poses: list[Pose | None] = [None] * self.config.num_inputs

        # Latest controller data (absolute poses)
        self._all_poses: list[NDArray[np.float32] | None] = [None] * self.config.num_inputs
        self._all_gripper_values: list[float] = [0.0] * self.config.num_inputs
        self._tracking_msg_count = 0
        self._publish_count = 0

        # Set default values for enable_inputs and input_labels if not provided
        if not self.config.enable_inputs:
            self.config.enable_inputs = [True] * self.config.num_inputs
        if not self.config.input_labels:
            self.config.input_labels = [f"controller_{i}" for i in range(self.config.num_inputs)]

        # check for mismatches between num_inputs and enable_inputs or input_labels
        if len(self.config.enable_inputs) != self.config.num_inputs:
            raise ValueError(f"Number of enable_inputs ({len(self.config.enable_inputs)}) does not match num_inputs ({self.config.num_inputs})")
        if len(self.config.input_labels) != self.config.num_inputs:
            raise ValueError(f"Number of input_labels ({len(self.config.input_labels)}) does not match num_inputs ({self.config.num_inputs})")

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

    @rpc
    def stop(self) -> None:
        """Stop the teleoperation module."""
        logger.info(f"Stopping {self.__class__.__name__}...")
        super().stop()

    # =========================================================================
    # Calibration
    # =========================================================================

    @rpc
    def calibrate(self) -> dict[str, Any]:
        """Calibrate by capturing initial controller poses.

        This is typically called when a calibration button is pressed.
        Captures the current controller poses as the "zero" reference.
        After calibration, delta poses are published.

        Returns:
            Dict with 'success' and optional 'message' or 'error'
        """
        logger.info("Calibrating controllers...")

        try:
            # Check if we have controller data for enabled inputs
            enabled_indices = [i for i, enabled in enumerate(self.config.enable_inputs) if enabled]
            if not enabled_indices:
                return {
                    "success": False,
                    "error": "No controllers are enabled. Enable at least one controller and try again.",
                }

            # Capture controller initial poses
            for i in enabled_indices:
                if self._all_poses[i] is not None:
                    pose_obj = matrix_to_pose(self._all_poses[i])
                    self._initial_poses[i] = pose_obj
                    logger.info(
                        f"Captured controller initial: "
                        f"pos=[{pose_obj.x:.3f}, {pose_obj.y:.3f}, {pose_obj.z:.3f}], "
                        f"rpy=[{pose_obj.roll:.3f}, {pose_obj.pitch:.3f}, {pose_obj.yaw:.3f}]"
                    )
                else:
                    return {
                        "success": False,
                        "error": f"Controller {self.config.input_labels[i]} data is None. Move controller and try again.",
                    }

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
        self._initial_poses = [None] * self.config.num_inputs

        logger.info("Calibration reset. Recalibrate to resume teleoperation...")
        return {"success": True, "message": "Calibration reset - recalibrate to resume"}

    @rpc
    def is_calibrated(self) -> bool:
        """Check if calibrated.

        Returns:
            True if calibrated and streaming deltas
        """
        return self._is_calibrated

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get current teleoperation status.

        Returns:
            Dictionary with status information keyed by TeleopStatusKey templates.
        """
        status: dict[str, Any] = {
            TeleopStatusKey.IS_CALIBRATED.value: self._is_calibrated,
        }
        for i in range(self.config.num_inputs):
            status[TeleopStatusKey.CONTROLLER_ENABLED.value.format(index=i)] = self.config.enable_inputs[i]
            status[TeleopStatusKey.CONTROLLER_HAS_DATA.value.format(index=i)] = self._all_poses[i] is not None
            status[TeleopStatusKey.CONTROLLER_GRIPPER_VALUE.value.format(index=i)] = self._all_gripper_values[i]
            status[TeleopStatusKey.CONTROLLER_LABEL.value.format(index=i)] = self.config.input_labels[i]

        return status

    # =========================================================================
    # Controller Data Processing (called by subclasses)
    # =========================================================================

    def update_controller_poses(
        self,
        controller_poses: list[NDArray[np.float32]],
        controller_gripper_values: list[float],
    ) -> None:
        """Update controller poses and publish deltas if calibrated.

        This method should be called by subclasses when new tracking data arrives
        from the device-specific connection.

        Args:
            controller_poses: List of 4x4 transformation matrices for all controllers
            controller_gripper_values: List of gripper values (0.0-1.0) for all controllers
        """
        # Track how many tracking messages we've received
        self._tracking_msg_count += 1

        # Log first few tracking messages to confirm data is arriving
        if self._tracking_msg_count <= 3:
            logger.info(f"Received tracking data #{self._tracking_msg_count}")

        # Store absolute poses
        self._all_poses = controller_poses
        self._all_gripper_values = controller_gripper_values

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

            for i in range(self.config.num_inputs):
                if not self.config.enable_inputs[i]:
                    continue

                current_time = time.time()
                pose_obj = matrix_to_pose(self._all_poses[i])
                input_label = self.config.input_labels[i]
                delta_pose = pose_obj - self._initial_poses[i]
                delta_pose_stamped = PoseStamped(
                    ts=current_time,
                    frame_id=f"{self.__class__.__name__.lower()}_{input_label}_controller_delta",
                    position=delta_pose.position,
                    orientation=delta_pose.orientation,
                )

                controller_output = getattr(self, f"controller_delta_{i}", None)
                if controller_output and hasattr(controller_output, "publish"):
                    try:
                        controller_output.publish(delta_pose_stamped)
                        self._visualize_controller_in_rerun(pose_obj, input_label)
                    except Exception as e:
                        logger.error(f"Failed to publish {input_label} delta: {e}")

                trigger_output = getattr(self, f"trigger_value_{i}", None)
                if trigger_output and hasattr(trigger_output, "publish"):
                    try:
                        trigger_output.publish(Float32(data=self._all_gripper_values[i]))
                        self._visualize_trigger_in_rerun(self._all_gripper_values[i], input_label)
                    except Exception as e:
                        logger.debug(f"Failed to publish {input_label} gripper: {e}")


                if self.config.log_input_data and (
                    self._publish_count <= 5
                    or self._publish_count % self.config.log_input_data_interval == 0
                ):
                    logger.info(
                        f"Published {input_label} delta #{self._publish_count}: "
                        f"pos=[{delta_pose_stamped.position.x:.3f}, {delta_pose_stamped.position.y:.3f}, {delta_pose_stamped.position.z:.3f}], "
                        f"rpy=[{delta_pose_stamped.roll:.3f}, {delta_pose_stamped.pitch:.3f}, {delta_pose_stamped.yaw:.3f}], "
                        f"frame_id={delta_pose_stamped.frame_id}, "
                        f"trigger_value={self._all_gripper_values[i]:.3f}"
                    )
        except Exception as e:
            logger.error(f"Error computing/publishing delta poses: {e}")

    # =========================================================================
    # Rerun Visualization
    # =========================================================================

    def _visualize_controller_in_rerun(
        self, controller_pose: Pose, controller_label: str
    ) -> None:
        """Visualize controller absolute pose in Rerun.

        Args:
            controller_pose: Absolute controller pose in robot space
            controller_label: Controller label from input_labels
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
                frame_id=f"world/teleop/{controller_label}_controller",
                position=controller_pose.position,
                orientation=controller_pose.orientation,
            )
            # Log absolute controller pose transform
            rr.log(
                f"world/teleop/{controller_label}_controller",
                controller_pose_stamped.to_rerun(),  # type: ignore[no-untyped-call]
            )
            # Log coordinate frame axes to visualize the transform
            rr.log(
                f"world/teleop/{controller_label}_controller",
                rr.Arrows3D(  # type: ignore[attr-defined]
                    vectors=[[0.30, 0, 0], [0, 0.30, 0], [0, 0, 0.30]],
                    origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
            )
        except Exception as e:
            logger.debug(f"Failed to log {controller_label} controller to Rerun: {e}")

    def _visualize_trigger_in_rerun(self, trigger_value: float, controller_label: str) -> None:
        """Visualize trigger value in Rerun as a scalar time series.

        Args:
            trigger_value: Trigger value (0.0-1.0) from controller
            controller_label: Controller label from input_labels
        """
        if not (
            self.config.visualize_in_rerun
            and RERUN_AVAILABLE
            and self._global_config.viewer_backend.startswith("rerun")
        ):
            return

        try:
            # Log trigger value as scalar time series
            rr.log(
                f"world/teleop/{controller_label}_controller/trigger",
                rr.Scalars(trigger_value),  # type: ignore[attr-defined]
            )
        except Exception as e:
            logger.debug(f"Failed to log {controller_label} trigger to Rerun: {e}")
