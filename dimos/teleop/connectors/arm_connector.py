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
Arm Teleop Connector

Connector for robotic arms that transforms delta poses into PoseStamped commands.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Vector3
from dimos.msgs.std_msgs import Bool
from dimos.teleop.connectors.base_connector import BaseTeleopConnector, ConnectorConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ArmConnectorConfig(ConnectorConfig):
    """Configuration for arm connector."""

    driver_module_name: str = "ArmDriver"
    driver_method_name: str = "get_state"
    dummy_driver: bool = False
    gripper_threshold: float = 0.5  # Trigger value above this activates gripper


class ArmConnector(BaseTeleopConnector):
    """Connector for robotic arms.

    Transforms delta poses into PoseStamped commands for arm drivers.
    Auto-calibrates on first delta by getting robot's initial end-effector pose via RPC.

    Output: PoseStamped (use with controller_delta_0 or controller_delta_1)
    """

    config: ArmConnectorConfig

    def __init__(self, config: ArmConnectorConfig | None = None) -> None:
        super().__init__(config or ArmConnectorConfig())
        self.config: ArmConnectorConfig = config or ArmConnectorConfig()

    def set_initial_pos(self, rpc_result: dict | None = None) -> bool:
        """Set initial robot pose from RPC result.

        Args:
            rpc_result: RPC result dict with 'success' and 'pose' keys. If None, uses zeros.

        Returns:
            True if successful, False otherwise.
        """
        if rpc_result is None or not rpc_result.get("success"):
            logger.info("Using zeros for initial pose")
            self._initial_robot_pose = Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
            )
        else:
            pose_data = rpc_result.get("pose", {})
            position = Vector3(
                pose_data.get("x", 0.0),
                pose_data.get("y", 0.0),
                pose_data.get("z", 0.0),
            )
            rpy = Vector3(
                pose_data.get("roll", 0.0),
                pose_data.get("pitch", 0.0),
                pose_data.get("yaw", 0.0),
            )
            self._initial_robot_pose = Pose(
                position=position,
                orientation=Quaternion.from_euler(rpy),
            )
            logger.info(f"Arm connector initial position set: {pose_data}")

        self._has_initial_pos = True
        return True

    def transform_delta(
        self,
        delta_pose: Pose,
        trigger_value: float,
    ) -> tuple[PoseStamped | None, Bool | None]:
        """Transform delta pose to target arm pose.

        Computes: target_pose = initial_robot_pose + delta

        Args:
            delta_pose: Delta from teleop device (current - initial).
            trigger_value: Trigger/gripper value (0.0-1.0).

        Returns:
            Tuple of (PoseStamped command, Bool gripper command).
        """
        if not self._has_initial_pos:
            if not self.set_initial_pos():
                return None, None

        if self._initial_robot_pose is None:
            return None, None

        target_pose = self._initial_robot_pose + delta_pose

        command = PoseStamped(
            position=target_pose.position,
            orientation=target_pose.orientation,
            frame_id="arm_command",
        )

        gripper_command = Bool(data=trigger_value > self.config.gripper_threshold)

        return command, gripper_command
