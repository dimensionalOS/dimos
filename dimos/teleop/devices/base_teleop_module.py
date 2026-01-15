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
Base Teleoperation Module.

Provides calibration, delta computation, and command publishing.
Subclasses implement device-specific input handling.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from dimos.core import Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Twist
from dimos.msgs.std_msgs import Bool
from dimos.utils.logging_config import setup_logger
from dimos.utils.teleop_transforms import compute_active_indices, parse_pose_from_dict
from dimos.utils.teleop_visualization import (
    init_rerun_visualization,
    visualize_controller_pose,
    visualize_trigger_value,
)
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


logger = setup_logger()


@dataclass
class BaseTeleopConfig(ModuleConfig):
    """Base configuration for teleoperation modules.

    output_types determines active indices: PoseStamped→0,1, Twist→2,3
    """

    output_types: list[type] = field(default_factory=lambda: [PoseStamped, PoseStamped])
    input_labels: list[str] = field(default_factory=lambda: ["left", "right"])
    robot_pose_rpc_methods: list[str | None] = field(default_factory=list)

    visualize_in_rerun: bool = True

    linear_scale: float = 1.0
    angular_scale: float = 1.0
    max_linear_velocity: float = 0.5
    max_angular_velocity: float = 1.0
    gripper_threshold: float = 0.5


class TeleopStatusKey(str, Enum):
    """Status dictionary keys (controller_* entries are indexed by controller number)."""

    IS_CALIBRATED = "is_calibrated"
    CONTROLLER_HAS_DATA = "controller_{index}_has_data"
    CONTROLLER_TRIGGER_VALUE = "controller_{index}_trigger_value"
    CONTROLLER_LABEL = "controller_{index}_label"


class BaseTeleopModule(Module, ABC):
    """Base class for teleoperation modules.

    Handles calibration, delta computation, and command publishing.
    Subclasses implement device-specific input handling.
    """

    default_config = BaseTeleopConfig
    config: BaseTeleopConfig

    # Output topics: PoseStamped for arms (0,1), Twist for locomotion (2,3)
    controller_delta_0: Out[PoseStamped] = None  # type: ignore
    trigger_value_0: Out[Bool] = None  # type: ignore
    controller_delta_1: Out[PoseStamped] = None  # type: ignore
    trigger_value_1: Out[Bool] = None  # type: ignore
    controller_delta_2: Out[Twist] = None  # type: ignore
    trigger_value_2: Out[Bool] = None  # type: ignore
    controller_delta_3: Out[Twist] = None  # type: ignore
    trigger_value_3: Out[Bool] = None  # type: ignore

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._is_calibrated = False
        self._tracking_msg_count = 0
        self._publish_count = 0

        if len(self.config.input_labels) != len(self.config.output_types):
            raise ValueError(
                f"input_labels length ({len(self.config.input_labels)}) must match "
                f"output_types length ({len(self.config.output_types)})"
            )

        self._active_indices = compute_active_indices(self.config.output_types)

        self._initial_poses: dict[int, Pose | None] = {i: None for i in self._active_indices}
        self._initial_robot_poses: dict[int, Pose | None] = {i: None for i in self._active_indices}
        self._all_poses: dict[int, NDArray[np.float64] | None] = {
            i: None for i in self._active_indices
        }
        self._all_trigger_values: dict[int, float] = {i: 0.0 for i in self._active_indices}

        logger.info(
            f"{self.__class__.__name__} initialized: indices={self._active_indices}, "
            f"types={[t.__name__ for t in self.config.output_types]}"
        )

    def _get_label(self, index: int) -> str:
        """Get label for an active index."""
        try:
            i = self._active_indices.index(index)
            return self.config.input_labels[i]
        except (ValueError, IndexError):
            return f"controller_{index}"

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(f"Starting {self.__class__.__name__}...")
        if self.config.visualize_in_rerun:
            init_rerun_visualization()

    @rpc
    def stop(self) -> None:
        logger.info(f"Stopping {self.__class__.__name__}...")
        super().stop()

    @rpc
    def calibrate(self) -> dict[str, Any]:
        """Capture current controller poses as the zero reference."""
        logger.info("Calibrating controllers...")

        try:
            if not self._active_indices:
                return {"success": False, "error": "No controllers are enabled"}

            for i, idx in enumerate(self._active_indices):
                pose_matrix = self._all_poses.get(idx)
                if pose_matrix is not None:
                    pose_obj = matrix_to_pose(pose_matrix)
                    self._initial_poses[idx] = pose_obj
                    logger.info(f"Captured controller {self._get_label(idx)} initial: {pose_obj}")
                else:
                    return {
                        "success": False,
                        "error": f"Controller {self._get_label(idx)} data is None",
                    }

                # Get initial robot pose via RPC if configured
                output_type = self.config.output_types[i]
                if i < len(self.config.robot_pose_rpc_methods):
                    rpc_method = self.config.robot_pose_rpc_methods[i]
                    if rpc_method is not None:
                        self._initial_robot_poses[idx] = self._get_robot_initial_pose(
                            output_type, rpc_method
                        )
                    else:
                        self._initial_robot_poses[idx] = parse_pose_from_dict(output_type)
                else:
                    self._initial_robot_poses[idx] = parse_pose_from_dict(output_type)

            self._is_calibrated = True
            logger.info("Calibration complete. Now streaming delta poses...")
            return {"success": True, "message": "Calibrated"}

        except Exception as e:
            logger.error(f"Calibration failed: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    def _get_robot_initial_pose(self, output_type: type, rpc_method: str) -> Pose | None:
        """Get initial robot pose via RPC call."""
        try:
            rpc_call = self.get_rpc_calls(rpc_method)
            rpc_result = rpc_call()
            pose = parse_pose_from_dict(output_type, rpc_result)
            logger.info(f"Got robot initial pose via {rpc_method}: {pose}")
            return pose
        except Exception as e:
            logger.warning(f"RPC call {rpc_method} failed: {e}, using zero pose")
            return parse_pose_from_dict(output_type)

    @rpc
    def reset_calibration(self) -> dict[str, Any]:
        """Reset calibration. Stops streaming until recalibrated."""
        self._is_calibrated = False
        self._initial_poses = {i: None for i in self._active_indices}
        self._initial_robot_poses = {i: None for i in self._active_indices}
        logger.info("Calibration reset.")
        return {"success": True, "message": "Calibration reset"}

    @rpc
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get current teleoperation status."""
        status: dict[str, Any] = {
            TeleopStatusKey.IS_CALIBRATED.value: self._is_calibrated,
            "active_indices": self._active_indices,
        }
        for i in self._active_indices:
            status[TeleopStatusKey.CONTROLLER_HAS_DATA.value.format(index=i)] = (
                self._all_poses.get(i) is not None
            )
            status[TeleopStatusKey.CONTROLLER_TRIGGER_VALUE.value.format(index=i)] = (
                self._all_trigger_values.get(i, 0.0)
            )
            status[TeleopStatusKey.CONTROLLER_LABEL.value.format(index=i)] = self._get_label(i)
        return status

    def compute_deltas(
        self,
        controller_poses: dict[int, NDArray[np.float64] | None],
        controller_trigger_values: dict[int, float],
    ) -> dict[int, Pose | None]:
        """Compute delta = current_pose - initial_pose for each controller."""
        self._tracking_msg_count += 1
        self._all_poses = controller_poses
        self._all_trigger_values = controller_trigger_values

        if not self._is_calibrated:
            if self._tracking_msg_count <= 1 or self._tracking_msg_count % 100 == 0:
                logger.warning("Not calibrated. Calibrate to start teleoperation.")
            return {i: None for i in self._active_indices}

        self._publish_count += 1
        deltas: dict[int, Pose | None] = {}

        for i in self._active_indices:
            pose_matrix = self._all_poses.get(i)
            if pose_matrix is None:
                deltas[i] = None
                continue

            initial_pose = self._initial_poses.get(i)
            if initial_pose is None:
                deltas[i] = None
                continue

            pose_obj = matrix_to_pose(pose_matrix)
            delta_pose = pose_obj - initial_pose

            if self.config.visualize_in_rerun:
                label = self._get_label(i)
                visualize_controller_pose(pose_obj, label)
                visualize_trigger_value(self._all_trigger_values.get(i, 0.0), label)

            deltas[i] = delta_pose

        return deltas

    def publish_command(self, index: int, command: Any, aux_command: Any = None) -> None:
        """Publish command to controller_delta_{index} and trigger_value_{index}."""
        if command is not None:
            controller_output = getattr(self, f"controller_delta_{index}", None)
            if controller_output and hasattr(controller_output, "publish"):
                try:
                    controller_output.publish(command)
                except Exception as e:
                    logger.error(f"Failed to publish command for index {index}: {e}")

        if aux_command is not None:
            trigger_output = getattr(self, f"trigger_value_{index}", None)
            if trigger_output and hasattr(trigger_output, "publish"):
                try:
                    trigger_output.publish(aux_command)
                except Exception as e:
                    logger.debug(f"Failed to publish aux command for index {index}: {e}")
