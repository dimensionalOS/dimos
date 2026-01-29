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

"""Connected hardware for the ControlCoordinator.

Wraps ManipulatorAdapter with coordinator-specific features:
- Namespaced joint names (e.g., "left_joint1")
- Unified read/write interface
- Hold-last-value for partial commands
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter

if TYPE_CHECKING:
    from dimos.control.components import HardwareComponent, HardwareId, JointName, JointState

logger = logging.getLogger(__name__)


class ConnectedHardware:
    """Runtime wrapper for hardware connected to the coordinator.

    Wraps a ManipulatorAdapter with coordinator-specific features:
    - Joint names from HardwareComponent config
    - Hold-last-value for partial commands
    - Converts between joint names and array indices

    Created when hardware is added to the coordinator. One instance
    per physical hardware device.
    """

    def __init__(
        self,
        adapter: ManipulatorAdapter,
        component: HardwareComponent,
    ) -> None:
        """Initialize hardware interface.

        Args:
            adapter: ManipulatorAdapter instance (XArmAdapter, PiperAdapter, etc.)
            component: Hardware component with joints config
        """
        if not isinstance(adapter, ManipulatorAdapter):
            raise TypeError("adapter must implement ManipulatorAdapter")

        self._adapter = adapter
        self._component = component
        self._joint_names = component.joints

        # Track last commanded values for hold-last behavior
        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def hardware_id(self) -> HardwareId:
        """Unique ID for this hardware."""
        return self._component.hardware_id

    @property
    def joint_names(self) -> list[JointName]:
        """Ordered list of joint names."""
        return self._joint_names

    @property
    def component(self) -> HardwareComponent:
        """The hardware component config."""
        return self._component

    @property
    def dof(self) -> int:
        """Degrees of freedom."""
        return len(self._joint_names)

    def disconnect(self) -> None:
        """Disconnect the underlying adapter."""
        self._adapter.disconnect()

    def read_state(self) -> dict[JointName, JointState]:
        """Read state as {joint_name: JointState}.

        Returns:
            Dict mapping joint name to JointState with position, velocity, effort
        """
        from dimos.control.components import JointState

        positions = self._adapter.read_joint_positions()
        velocities = self._adapter.read_joint_velocities()
        efforts = self._adapter.read_joint_efforts()

        return {
            name: JointState(
                position=positions[i],
                velocity=velocities[i],
                effort=efforts[i],
            )
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
                    f"Hardware {self.hardware_id} received command for unknown joint "
                    f"{joint_name}. Valid joints: {self._joint_names}"
                )
                self._warned_unknown_joints.add(joint_name)

        # Build ordered list for adapter
        ordered = self._build_ordered_command()

        # Switch control mode if needed
        if mode != self._current_mode:
            if not self._adapter.set_control_mode(mode):
                logger.warning(f"Hardware {self.hardware_id} failed to switch to {mode.name}")
                return False
            self._current_mode = mode

        # Send to adapter
        match mode:
            case ControlMode.POSITION | ControlMode.SERVO_POSITION:
                return self._adapter.write_joint_positions(ordered)
            case ControlMode.VELOCITY:
                return self._adapter.write_joint_velocities(ordered)
            case ControlMode.TORQUE:
                logger.warning(f"Hardware {self.hardware_id} does not support torque mode")
                return False
            case _:
                return False

    def _initialize_last_commanded(self) -> None:
        """Initialize last_commanded with current hardware positions."""
        for _ in range(10):
            try:
                current = self._adapter.read_joint_positions()
                for i, name in enumerate(self._joint_names):
                    self._last_commanded[name] = current[i]
                self._initialized = True
                return
            except Exception:
                time.sleep(0.01)

        raise RuntimeError(
            f"Hardware {self.hardware_id} failed to read initial positions after retries"
        )

    def _build_ordered_command(self) -> list[float]:
        """Build ordered command list from last_commanded dict."""
        return [self._last_commanded[name] for name in self._joint_names]


__all__ = [
    "ConnectedHardware",
]
