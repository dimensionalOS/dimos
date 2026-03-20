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

"""Mock twist base adapter for testing - no hardware required.

Usage:
    >>> from dimos.hardware.drive_trains.mock import MockTwistBaseAdapter
    >>> adapter = MockTwistBaseAdapter(dof=3)
    >>> adapter.connect()
    True
    >>> adapter.write_velocities([0.5, 0.0, 0.1])
    True
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry


class MockTwistBaseAdapter:
    """Fake twist base adapter for unit tests.

    Implements TwistBaseAdapter protocol with in-memory state.
    Useful for:
    - Unit testing coordinator logic without hardware
    - Integration testing with predictable behavior
    - Development without a physical base
    """

    def __init__(self, dof: int = 3, **_: object) -> None:
        self._dof = dof
        self._velocities = [0.0] * dof
        self._odometry: list[float] | None = [0.0] * dof
        self._enabled = False
        self._connected = False

    def connect(self) -> bool:
        """Simulate connection."""
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Simulate disconnection."""
        self._connected = False

    def is_connected(self) -> bool:
        """Check mock connection status."""
        return self._connected

    def get_dof(self) -> int:
        """Return DOF."""
        return self._dof

    def read_velocities(self) -> list[float]:
        """Return mock velocities."""
        return self._velocities.copy()

    def read_odometry(self) -> list[float] | None:
        """Return mock odometry."""
        if self._odometry is None:
            return None
        return self._odometry.copy()

    def write_velocities(self, velocities: list[float]) -> bool:
        """Set mock velocities."""
        if len(velocities) != self._dof:
            return False
        self._velocities = list(velocities)
        return True

    def write_stop(self) -> bool:
        """Stop mock motion."""
        self._velocities = [0.0] * self._dof
        return True

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable mock platform."""
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        """Check mock enable state."""
        return self._enabled

    def set_odometry(self, odometry: list[float] | None) -> None:
        """Set odometry directly for testing."""
        self._odometry = list(odometry) if odometry is not None else None

    def set_velocities_directly(self, velocities: list[float]) -> None:
        """Set velocities directly for testing (bypasses DOF check)."""
        self._velocities = list(velocities)


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("mock_twist_base", MockTwistBaseAdapter)


__all__ = ["MockTwistBaseAdapter"]
