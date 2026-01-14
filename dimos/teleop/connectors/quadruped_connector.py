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
Quadruped Teleop Connector

Connector for quadruped robots that transforms delta poses into Twist velocity commands.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.msgs.geometry_msgs import Pose, Twist, Vector3
from dimos.msgs.std_msgs import Bool
from dimos.teleop.connectors.base_connector import BaseTeleopConnector, ConnectorConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class QuadrupedConnectorConfig(ConnectorConfig):
    """Configuration for quadruped connector."""

    driver_module_name: str = "QuadrupedDriver"
    driver_method_name: str = "get_state"
    dummy_driver: bool = False

    linear_scale: float = 1.0  # Scale factor for linear velocity
    angular_scale: float = 1.0  # Scale factor for angular velocity
    max_linear_velocity: float = 0.5  # m/s
    max_angular_velocity: float = 1.0  # rad/s


class QuadrupedConnector(BaseTeleopConnector):
    """Connector for quadruped robots.

    Transforms delta poses into Twist commands for quadruped locomotion.
    Maps position deltas to linear velocity and orientation deltas to angular velocity.

    Output: Twist (use with controller_delta_2 or controller_delta_3)
    """

    config: QuadrupedConnectorConfig

    def __init__(self, config: QuadrupedConnectorConfig | None = None) -> None:
        super().__init__(config or QuadrupedConnectorConfig())
        self.config: QuadrupedConnectorConfig = config or QuadrupedConnectorConfig()

    def set_initial_pos(self, rpc_result: dict | None = None) -> bool:
        """Quadruped connector doesn't need initial position from robot.

        Args:
            rpc_result: Ignored for quadruped.

        Returns:
            Always True.
        """
        logger.info("Quadruped connector ready (no robot state needed)")
        self._has_initial_pos = True
        return True

    def transform_delta(
        self,
        delta_pose: Pose,
        trigger_value: float,
    ) -> tuple[Twist | None, Bool | None]:
        """Transform delta pose to velocity command.

        Maps:
        - delta position -> linear velocity (x, y, z)
        - delta orientation (euler) -> angular velocity (roll, pitch, yaw rates)

        Args:
            delta_pose: Delta from teleop device (current - initial).
            trigger_value: Trigger value (0.0-1.0).

        Returns:
            Tuple of (Twist command, Bool trigger command).
        """
        if not self._has_initial_pos:
            self.set_initial_pos()

        # Scale and clamp linear velocity from position delta
        linear = Vector3(
            self._clamp(
                delta_pose.position.x * self.config.linear_scale,
                self.config.max_linear_velocity,
            ),
            self._clamp(
                delta_pose.position.y * self.config.linear_scale,
                self.config.max_linear_velocity,
            ),
            self._clamp(
                delta_pose.position.z * self.config.linear_scale,
                self.config.max_linear_velocity,
            ),
        )

        # Scale and clamp angular velocity from orientation delta
        # Convert quaternion to euler for angular velocity mapping
        euler = delta_pose.orientation.to_euler()
        angular = Vector3(
            self._clamp(
                euler.x * self.config.angular_scale,
                self.config.max_angular_velocity,
            ),
            self._clamp(
                euler.y * self.config.angular_scale,
                self.config.max_angular_velocity,
            ),
            self._clamp(
                euler.z * self.config.angular_scale,
                self.config.max_angular_velocity,
            ),
        )

        command = Twist(linear=linear, angular=angular)
        trigger_command = Bool(data=trigger_value > 0.5)

        return command, trigger_command

    def _clamp(self, value: float, limit: float) -> float:
        """Clamp value to [-limit, limit]."""
        return max(-limit, min(limit, value))
