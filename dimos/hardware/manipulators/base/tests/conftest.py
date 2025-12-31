# Copyright 2025 Dimensional Inc.
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

"""Pytest fixtures and mocks for manipulator driver tests.

This module contains MockSDK which implements BaseManipulatorSDK with controllable
behavior for testing driver logic without requiring hardware.

Features:
- Configurable initial state (positions, DOF, vendor, model)
- Call tracking for verification
- Configurable error injection
- Simulated behavior (e.g., position updates)
"""

from dataclasses import dataclass, field
import math

import pytest

from ..sdk_interface import BaseManipulatorSDK, ManipulatorInfo


@dataclass
class MockSDKConfig:
    """Configuration for MockSDK behavior."""

    dof: int = 6
    vendor: str = "Mock"
    model: str = "TestArm"
    initial_positions: list[float] | None = None
    initial_velocities: list[float] | None = None
    initial_efforts: list[float] | None = None

    # Error injection
    connect_fails: bool = False
    enable_fails: bool = False
    motion_fails: bool = False
    error_code: int = 0

    # Behavior options
    simulate_motion: bool = False  # If True, set_joint_positions updates internal state


@dataclass
class CallRecord:
    """Record of a method call for verification."""

    method: str
    args: tuple = field(default_factory=tuple)
    kwargs: dict = field(default_factory=dict)


class MockSDK(BaseManipulatorSDK):
    """Mock SDK for unit testing. Implements BaseManipulatorSDK interface.

    Usage:
        # Basic usage
        mock = MockSDK()
        driver = create_driver_with_sdk(mock)
        driver.enable_servo()
        assert mock.enable_servos_called

        # With custom config
        config = MockSDKConfig(dof=7, connect_fails=True)
        mock = MockSDK(config=config)

        # With initial positions
        mock = MockSDK(positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        # Verify calls
        mock.set_joint_positions([0.1] * 6)
        assert mock.was_called("set_joint_positions")
        assert mock.call_count("set_joint_positions") == 1
    """

    def __init__(
        self,
        config: MockSDKConfig | None = None,
        *,
        dof: int = 6,
        vendor: str = "Mock",
        model: str = "TestArm",
        positions: list[float] | None = None,
    ):
        """Initialize MockSDK.

        Args:
            config: Full configuration object (takes precedence)
            dof: Degrees of freedom (ignored if config provided)
            vendor: Vendor name (ignored if config provided)
            model: Model name (ignored if config provided)
            positions: Initial joint positions (ignored if config provided)
        """
        if config is None:
            config = MockSDKConfig(
                dof=dof,
                vendor=vendor,
                model=model,
                initial_positions=positions,
            )

        self._config = config
        self._dof = config.dof
        self._vendor = config.vendor
        self._model = config.model

        # State
        self._connected = False
        self._servos_enabled = False
        self._positions = list(config.initial_positions or [0.0] * self._dof)
        self._velocities = list(config.initial_velocities or [0.0] * self._dof)
        self._efforts = list(config.initial_efforts or [0.0] * self._dof)
        self._mode = 0
        self._state = 0
        self._error_code = config.error_code

        # Call tracking
        self._calls: list[CallRecord] = []

        # Convenience flags for simple assertions
        self.connect_called = False
        self.disconnect_called = False
        self.enable_servos_called = False
        self.disable_servos_called = False
        self.set_joint_positions_called = False
        self.set_joint_velocities_called = False
        self.stop_motion_called = False
        self.emergency_stop_called = False
        self.clear_errors_called = False

    def _record_call(self, method: str, *args, **kwargs):
        """Record a method call."""
        self._calls.append(CallRecord(method=method, args=args, kwargs=kwargs))

    def was_called(self, method: str) -> bool:
        """Check if a method was called."""
        return any(c.method == method for c in self._calls)

    def call_count(self, method: str) -> int:
        """Get the number of times a method was called."""
        return sum(1 for c in self._calls if c.method == method)

    def get_calls(self, method: str) -> list[CallRecord]:
        """Get all calls to a specific method."""
        return [c for c in self._calls if c.method == method]

    def get_last_call(self, method: str) -> CallRecord | None:
        """Get the last call to a specific method."""
        calls = self.get_calls(method)
        return calls[-1] if calls else None

    def reset_calls(self):
        """Reset call tracking."""
        self._calls.clear()
        self.connect_called = False
        self.disconnect_called = False
        self.enable_servos_called = False
        self.disable_servos_called = False
        self.set_joint_positions_called = False
        self.set_joint_velocities_called = False
        self.stop_motion_called = False
        self.emergency_stop_called = False
        self.clear_errors_called = False

    # ============= State Manipulation (for test setup) =============

    def set_positions(self, positions: list[float]):
        """Set internal positions (test helper)."""
        self._positions = list(positions)

    def set_error(self, code: int, message: str = ""):
        """Inject an error state (test helper)."""
        self._error_code = code

    def set_enabled(self, enabled: bool):
        """Set servo enabled state (test helper)."""
        self._servos_enabled = enabled

    # ============= BaseManipulatorSDK Implementation =============

    def connect(self, config: dict) -> bool:
        self._record_call("connect", config)
        self.connect_called = True

        if self._config.connect_fails:
            return False

        self._connected = True
        return True

    def disconnect(self) -> None:
        self._record_call("disconnect")
        self.disconnect_called = True
        self._connected = False

    def is_connected(self) -> bool:
        self._record_call("is_connected")
        return self._connected

    def get_joint_positions(self) -> list[float]:
        self._record_call("get_joint_positions")
        return self._positions.copy()

    def get_joint_velocities(self) -> list[float]:
        self._record_call("get_joint_velocities")
        return self._velocities.copy()

    def get_joint_efforts(self) -> list[float]:
        self._record_call("get_joint_efforts")
        return self._efforts.copy()

    def set_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        self._record_call(
            "set_joint_positions",
            positions,
            velocity=velocity,
            acceleration=acceleration,
            wait=wait,
        )
        self.set_joint_positions_called = True

        if self._config.motion_fails:
            return False

        if not self._servos_enabled:
            return False

        if self._config.simulate_motion:
            self._positions = list(positions)

        return True

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        self._record_call("set_joint_velocities", velocities)
        self.set_joint_velocities_called = True

        if self._config.motion_fails:
            return False

        if not self._servos_enabled:
            return False

        self._velocities = list(velocities)
        return True

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        self._record_call("set_joint_efforts", efforts)
        return False  # Not supported in mock

    def stop_motion(self) -> bool:
        self._record_call("stop_motion")
        self.stop_motion_called = True
        self._velocities = [0.0] * self._dof
        return True

    def enable_servos(self) -> bool:
        self._record_call("enable_servos")
        self.enable_servos_called = True

        if self._config.enable_fails:
            return False

        self._servos_enabled = True
        return True

    def disable_servos(self) -> bool:
        self._record_call("disable_servos")
        self.disable_servos_called = True
        self._servos_enabled = False
        return True

    def are_servos_enabled(self) -> bool:
        self._record_call("are_servos_enabled")
        return self._servos_enabled

    def get_robot_state(self) -> dict:
        self._record_call("get_robot_state")
        return {
            "state": self._state,
            "mode": self._mode,
            "error_code": self._error_code,
            "is_moving": any(v != 0 for v in self._velocities),
        }

    def get_error_code(self) -> int:
        self._record_call("get_error_code")
        return self._error_code

    def get_error_message(self) -> str:
        self._record_call("get_error_message")
        return "" if self._error_code == 0 else f"Mock error {self._error_code}"

    def clear_errors(self) -> bool:
        self._record_call("clear_errors")
        self.clear_errors_called = True
        self._error_code = 0
        return True

    def emergency_stop(self) -> bool:
        self._record_call("emergency_stop")
        self.emergency_stop_called = True
        self._velocities = [0.0] * self._dof
        self._servos_enabled = False
        return True

    def get_info(self) -> ManipulatorInfo:
        self._record_call("get_info")
        return ManipulatorInfo(
            vendor=self._vendor,
            model=f"{self._model} (Mock)",
            dof=self._dof,
            firmware_version="mock-1.0.0",
            serial_number="MOCK-001",
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        self._record_call("get_joint_limits")
        lower = [-2 * math.pi] * self._dof
        upper = [2 * math.pi] * self._dof
        return lower, upper

    def get_velocity_limits(self) -> list[float]:
        self._record_call("get_velocity_limits")
        return [math.pi] * self._dof

    def get_acceleration_limits(self) -> list[float]:
        self._record_call("get_acceleration_limits")
        return [math.pi * 2] * self._dof


# ============= Pytest Fixtures =============


@pytest.fixture
def mock_sdk():
    """Create a basic MockSDK."""
    return MockSDK(dof=6)


@pytest.fixture
def mock_sdk_with_positions():
    """Create MockSDK with initial positions."""
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    return MockSDK(positions=positions)
