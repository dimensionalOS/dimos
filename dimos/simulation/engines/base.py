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

"""Base interfaces for simulator engines."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.msgs.sensor_msgs import JointState


class SimulationEngine(ABC):
    """Abstract base class for a simulator engine instance."""

    def __init__(self, config_path: Path, headless: bool) -> None:
        self._config_path = config_path
        self._headless = headless

    @property
    def config_path(self) -> Path:
        return self._config_path

    @property
    def headless(self) -> bool:
        return self._headless

    @abstractmethod
    def connect(self) -> bool:
        """Connect to simulation and start the engine."""

    @abstractmethod
    def disconnect(self) -> bool:
        """Disconnect from simulation and stop the engine."""

    @property
    @abstractmethod
    def connected(self) -> bool:
        """Whether the engine is connected."""

    @property
    @abstractmethod
    def num_joints(self) -> int:
        """Number of joints for the loaded robot."""

    @property
    @abstractmethod
    def joint_names(self) -> list[str]:
        """Joint names for the loaded robot."""

    @abstractmethod
    def read_joint_positions(self) -> list[float]:
        """Read joint positions in radians."""

    @abstractmethod
    def read_joint_velocities(self) -> list[float]:
        """Read joint velocities in rad/s."""

    @abstractmethod
    def read_joint_efforts(self) -> list[float]:
        """Read joint efforts in Nm."""

    @abstractmethod
    def write_joint_command(self, command: JointState) -> None:
        """Command joints using a JointState message."""

    @abstractmethod
    def hold_current_position(self) -> None:
        """Hold current joint positions."""
