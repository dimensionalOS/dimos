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

"""Hardware interface for the ControlOrchestrator.

Wraps ManipulatorBackend with orchestrator-specific features:
- Namespaced joint names (e.g., "left_joint1")
- Unified read/write interface
- Hold-last-value for partial commands
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorBackend

logger = logging.getLogger(__name__)


@runtime_checkable
class HardwareInterface(Protocol):
    """Protocol for hardware that the orchestrator can control.

    This wraps ManipulatorBackend with orchestrator-specific features:
    - Namespaced joint names (e.g., "left_arm_joint1")
    - Unified read/write interface
    - State caching
    """

    @property
    def hardware_id(self) -> str:
        """Unique ID for this hardware (e.g., 'left_arm')."""
        ...

    @property
    def joint_names(self) -> list[str]:
        """Ordered list of fully-qualified joint names this hardware controls."""
        ...

    def read_state(self) -> dict[str, tuple[float, float, float]]:
        """Read current state.

        Returns:
            Dict of joint_name -> (position, velocity, effort)
        """
        ...

    def write_command(self, commands: dict[str, float], mode: ControlMode) -> bool:
        """Write commands to hardware.

        IMPORTANT: Accepts partial joint sets. Missing joints hold last value.

        Args:
            commands: {joint_name: value} - can be partial
            mode: Control mode (POSITION, VELOCITY, TORQUE)

        Returns:
            True if command was sent successfully
        """
        ...

    def disconnect(self) -> None:
        """Disconnect the underlying hardware."""
        ...


class BackendHardwareInterface:
    """Concrete implementation wrapping a ManipulatorBackend.

    Features:
    - Generates namespaced joint names (prefix_joint1, prefix_joint2, ...)
    - Holds last commanded value for partial commands
    - On first tick, reads current position from hardware for missing joints
    """

    def __init__(
        self,
        backend: ManipulatorBackend,
        hardware_id: str,
        joint_prefix: str | None = None,
    ) -> None:
        """Initialize hardware interface.

        Args:
            backend: ManipulatorBackend instance (XArmBackend, PiperBackend, etc.)
            hardware_id: Unique identifier for this hardware
            joint_prefix: Prefix for joint names (defaults to hardware_id)
        """
        if not isinstance(backend, ManipulatorBackend):
            raise TypeError("backend must implement ManipulatorBackend")

        self._backend = backend
        self._hardware_id = hardware_id
        self._prefix = joint_prefix or hardware_id
        self._dof = backend.get_dof()

        # Generate joint names: prefix_joint1, prefix_joint2, ...
        self._joint_names = [f"{self._prefix}_joint{i + 1}" for i in range(self._dof)]

        # Track last commanded values for hold-last behavior
        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def hardware_id(self) -> str:
        """Unique ID for this hardware."""
        return self._hardware_id

    @property
    def joint_names(self) -> list[str]:
        """Ordered list of joint names."""
        return self._joint_names

    @property
    def dof(self) -> int:
        """Degrees of freedom."""
        return self._dof

    def disconnect(self) -> None:
        """Disconnect the underlying backend."""
        self._backend.disconnect()

    def read_state(self) -> dict[str, tuple[float, float, float]]:
        """Read state as {joint_name: (position, velocity, effort)}.

        Returns:
            Dict mapping joint name to (position, velocity, effort) tuple
        """
        positions = self._backend.read_joint_positions()
        velocities = self._backend.read_joint_velocities()
        efforts = self._backend.read_joint_efforts()

        return {
            name: (positions[i], velocities[i], efforts[i])
            for i, name in enumerate(self._joint_names)
        }

    def write_command(self, commands: dict[str, float], mode: ControlMode) -> bool:
        """Write commands - allows partial joint sets, holds last for missing.

        This is critical for:
        - Partial WBC overrides
        - Safety controllers
        - Mixed task ownership

        Args:
            commands: {joint_name: value} - can be partial
            mode: Control mode

        Returns:
            True if command was sent successfully
        """
        # Initialize on first write if needed
        if not self._initialized:
            self._initialize_last_commanded()

        # Update last commanded for joints we received
        for joint_name, value in commands.items():
            if joint_name in self._joint_names:
                self._last_commanded[joint_name] = value
            elif joint_name not in self._warned_unknown_joints:
                logger.warning(
                    f"Hardware {self._hardware_id} received command for unknown joint "
                    f"{joint_name}. Valid joints: {self._joint_names}"
                )
                self._warned_unknown_joints.add(joint_name)

        # Build ordered list for backend
        ordered = self._build_ordered_command()

        # Switch control mode if needed
        if mode != self._current_mode:
            if not self._backend.set_control_mode(mode):
                logger.warning(f"Hardware {self._hardware_id} failed to switch to {mode.name}")
                return False
            self._current_mode = mode

        # Send to backend
        match mode:
            case ControlMode.POSITION | ControlMode.SERVO_POSITION:
                return self._backend.write_joint_positions(ordered)
            case ControlMode.VELOCITY:
                return self._backend.write_joint_velocities(ordered)
            case ControlMode.TORQUE:
                logger.warning(f"Hardware {self._hardware_id} does not support torque mode")
                return False
            case _:
                return False

    def _initialize_last_commanded(self) -> None:
        """Initialize last_commanded with current hardware positions."""
        for _ in range(10):
            try:
                current = self._backend.read_joint_positions()
                for i, name in enumerate(self._joint_names):
                    self._last_commanded[name] = current[i]
                self._initialized = True
                return
            except Exception:
                time.sleep(0.01)

        raise RuntimeError(
            f"Hardware {self._hardware_id} failed to read initial positions after retries"
        )

    def _build_ordered_command(self) -> list[float]:
        """Build ordered command list from last_commanded dict."""
        return [self._last_commanded[name] for name in self._joint_names]


__all__ = [
    "BackendHardwareInterface",
    "HardwareInterface",
]
