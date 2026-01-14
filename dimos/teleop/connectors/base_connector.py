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
Base Teleop Connector

Abstract base class for injectable teleop connectors.
Connectors transform delta poses into robot commands.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from dimos.msgs.geometry_msgs import Pose


@dataclass
class ConnectorConfig:
    """Base configuration for teleop connectors."""

    driver_module_name: str = "driver_name"
    driver_method_name: str = "get_state"
    dummy_driver: bool = False


class BaseTeleopConnector(ABC):
    """Abstract base class for teleop connectors.

    Connectors are injectable classes (not Modules) that:
    1. Receive delta poses from the teleop module
    2. Transform deltas into robot commands
    3. Optionally make RPC calls to robot drivers
    4. Return commands to be published by the teleop module
    """

    config: ConnectorConfig

    def __init__(self, config: ConnectorConfig | None = None) -> None:
        self.config = config or ConnectorConfig()
        self._has_initial_pos = False
        self._initial_robot_pose: Pose | None = None

    @abstractmethod
    def set_initial_pos(self, rpc_result: dict | None = None) -> bool:
        """Set initial robot pose for the connector.

        Args:
            rpc_result: RPC result from robot driver (optional, some connectors don't need it).

        Returns:
            True if successful, False otherwise.
        """
        pass

    @abstractmethod
    def transform_delta(
        self,
        delta_pose: Pose,
        trigger_value: float,
    ) -> tuple[Any, Any]:
        """Transform a delta pose into a robot command.

        Args:
            delta_pose: The delta pose (current - initial) from teleop device.
            trigger_value: Trigger/gripper value (0.0-1.0).

        Returns:
            Tuple of (command to publish, optional gripper/aux command).
        """
        pass

    @property
    def has_initial_pos(self) -> bool:
        """Check if connector has initial position."""
        return self._has_initial_pos

    def reset_initial_pos(self) -> None:
        """Reset connector initial position."""
        self._has_initial_pos = False
        self._initial_robot_pose = None
