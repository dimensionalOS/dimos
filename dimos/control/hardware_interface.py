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

Provides two wrapper types:
- ConnectedHardware: Wraps ManipulatorAdapter for joint-controlled arms
- ConnectedTwistBase: Wraps TwistBaseAdapter for velocity-commanded platforms

Both share the same duck-type interface (read_state, write_command, etc.)
so the tick loop treats them uniformly.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.components import HardwareComponent, HardwareId, JointName, JointState
    from dimos.hardware.drive_trains.spec import TwistBaseAdapter
    from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter

logger = setup_logger()


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
        self._adapter = adapter
        self._component = component
        self._arm_joint_names: list[JointName] = list(component.joints)
        self._gripper_joints: list[JointName] = list(component.gripper_joints)
        self._joint_names: list[JointName] = component.all_joints

        # Track last commanded values for hold-last behavior
        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> ManipulatorAdapter:
        """The underlying hardware adapter."""
        return self._adapter

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

        result: dict[JointName, JointState] = {
            name: JointState(
                position=positions[i],
                velocity=velocities[i],
                effort=efforts[i],
            )
            for i, name in enumerate(self._arm_joint_names)
        }

        # Append gripper joint(s) via adapter gripper method
        if self._gripper_joints:
            gripper_pos = self._adapter.read_gripper_position()
            for gj in self._gripper_joints:
                result[gj] = JointState(
                    position=gripper_pos if gripper_pos is not None else 0.0,
                    velocity=0.0,
                    effort=0.0,
                )

        return result

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

        # Build ordered list for arm joints only
        arm_ordered = [self._last_commanded[name] for name in self._arm_joint_names]

        # Switch control mode if needed
        if mode != self._current_mode:
            if not self._adapter.set_control_mode(mode):
                logger.warning(f"Hardware {self.hardware_id} failed to switch to {mode.name}")
                return False
            self._current_mode = mode

        # Send arm joints to adapter
        arm_ok: bool
        match mode:
            case ControlMode.POSITION | ControlMode.SERVO_POSITION:
                arm_ok = self._adapter.write_joint_positions(arm_ordered)
            case ControlMode.VELOCITY:
                arm_ok = self._adapter.write_joint_velocities(arm_ordered)
            case ControlMode.TORQUE:
                logger.warning(f"Hardware {self.hardware_id} does not support torque mode")
                arm_ok = False
            case _:
                arm_ok = False

        # Send gripper joints via adapter gripper method
        gripper_ok = True
        for gj in self._gripper_joints:
            if gj in self._last_commanded:
                gripper_ok = (
                    self._adapter.write_gripper_position(self._last_commanded[gj]) and gripper_ok
                )

        return arm_ok and gripper_ok

    def _initialize_last_commanded(self) -> None:
        """Initialize last_commanded with current hardware positions."""
        for _ in range(10):
            try:
                current = self._adapter.read_joint_positions()
                for i, name in enumerate(self._arm_joint_names):
                    self._last_commanded[name] = current[i]

                # Initialize gripper joint(s) from adapter
                if self._gripper_joints:
                    gripper_pos = self._adapter.read_gripper_position()
                    for gj in self._gripper_joints:
                        self._last_commanded[gj] = gripper_pos if gripper_pos is not None else 0.0

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


class ConnectedTwistBase(ConnectedHardware):
    """Runtime wrapper for a twist base connected to the coordinator.

    Inherits from ConnectedHardware and overrides behavior for
    velocity-commanded platforms (holonomic bases, drones, quadrupeds, etc.).

    Key differences from ConnectedHardware:
    - Positions come from odometry (or zeros if unavailable)
    - Efforts are always zero
    - write_command always sends velocities regardless of mode
    - No retry loop for initialization (twist bases start at zero velocity)
    """

    _twist_adapter: TwistBaseAdapter

    def __init__(
        self,
        adapter: TwistBaseAdapter,
        component: HardwareComponent,
    ) -> None:
        self._twist_adapter = adapter
        self._component = component
        self._joint_names = component.joints

        # Twist bases start at zero velocity — no need to read from hardware
        self._last_commanded: dict[str, float] = {name: 0.0 for name in self._joint_names}
        self._initialized = True
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> TwistBaseAdapter:  # type: ignore[override]
        """The underlying twist base adapter."""
        return self._twist_adapter

    def disconnect(self) -> None:
        """Disconnect the underlying adapter."""
        self._twist_adapter.disconnect()

    def read_state(self) -> dict[JointName, JointState]:
        """Read state as {joint_name: JointState}.

        Positions come from odometry (zeros if unavailable).
        Velocities from adapter. Efforts are always zero.
        """
        from dimos.control.components import JointState

        velocities = self._twist_adapter.read_velocities()
        odometry = self._twist_adapter.read_odometry()
        positions = odometry if odometry is not None else [0.0] * self.dof

        return {
            name: JointState(
                position=positions[i],
                velocity=velocities[i],
                effort=0.0,
            )
            for i, name in enumerate(self._joint_names)
        }

    def write_command(self, commands: dict[str, float], _mode: ControlMode) -> bool:
        """Write velocity commands — always sends velocities regardless of mode.

        Args:
            commands: {joint_name: velocity} - can be partial
            _mode: Control mode (ignored — twist bases always use velocity)

        Returns:
            True if command was sent successfully
        """
        # Update last commanded for joints we received
        for joint_name, value in commands.items():
            if joint_name in self._last_commanded:
                self._last_commanded[joint_name] = value
            elif joint_name not in self._warned_unknown_joints:
                logger.warning(
                    f"TwistBase {self.hardware_id} received command for unknown joint "
                    f"{joint_name}. Valid joints: {self._joint_names}"
                )
                self._warned_unknown_joints.add(joint_name)

        # Build ordered velocity list and send
        ordered = self._build_ordered_command()
        return self._twist_adapter.write_velocities(ordered)


_DEFAULT_KP: float = 40.0
_DEFAULT_KD: float = 3.0


class ConnectedWholeBody(ConnectedHardware):
    """Runtime wrapper for a whole-body motor platform connected to the coordinator.

    Wraps a WholeBodyAdapter for joint-level motor control (any DOF count).

    Key differences from ConnectedHardware:
    - Reads joint state from MotorState (q, dq, tau)
    - write_command converts position commands to MotorCommand with PD gains
    - write_motor_commands provides direct pass-through to adapter
    """

    _wb_adapter: WholeBodyAdapter

    def __init__(
        self,
        adapter: WholeBodyAdapter,
        component: HardwareComponent,
    ) -> None:
        self._wb_adapter = adapter
        self._component = component
        self._joint_names = component.joints

        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> WholeBodyAdapter:  # type: ignore[override]
        """The underlying whole-body adapter."""
        return self._wb_adapter

    def disconnect(self) -> None:
        """Disconnect the underlying adapter."""
        self._wb_adapter.disconnect()

    def read_state(self) -> dict[JointName, JointState]:
        """Read motor states as {joint_name: JointState}."""
        from dimos.control.components import JointState

        motor_states = self._wb_adapter.read_motor_states()
        return {
            name: JointState(
                position=motor_states[i].q,
                velocity=motor_states[i].dq,
                effort=motor_states[i].tau,
            )
            for i, name in enumerate(self._joint_names)
        }

    def write_command(self, commands: dict[str, float], mode: ControlMode) -> bool:
        """Write position commands — converts to MotorCommand with PD gains.

        Only POSITION / SERVO_POSITION are supported; other modes are warned
        and dropped (matches ConnectedHardware's warn-and-skip pattern).
        """
        from dimos.hardware.whole_body.spec import MotorCommand

        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            logger.warning(
                f"WholeBody {self.hardware_id} only supports POSITION/SERVO_POSITION; "
                f"got {mode.name} — skipping"
            )
            return False

        if not self._initialized and not self._try_initialize_last_commanded():
            return False

        for joint_name, value in commands.items():
            if joint_name in self._joint_names:
                self._last_commanded[joint_name] = value
            elif joint_name not in self._warned_unknown_joints:
                logger.warning(
                    f"WholeBody {self.hardware_id} received command for unknown joint "
                    f"{joint_name}. Valid joints: {self._joint_names}"
                )
                self._warned_unknown_joints.add(joint_name)

        motor_cmds = [
            MotorCommand(
                q=self._last_commanded[name],
                dq=0.0,
                kp=_DEFAULT_KP,
                kd=_DEFAULT_KD,
                tau=0.0,
            )
            for name in self._joint_names
        ]
        return self._wb_adapter.write_motor_commands(motor_cmds)

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        """Direct pass-through to adapter for full MotorCommand control."""
        return self._wb_adapter.write_motor_commands(commands)

    def _try_initialize_last_commanded(self) -> bool:
        """Non-blocking init. Returns True once motor_states is cached."""
        if not self._wb_adapter.has_motor_states():
            return False
        states = self._wb_adapter.read_motor_states()
        for i, name in enumerate(self._joint_names):
            self._last_commanded[name] = states[i].q
        self._initialized = True
        return True


__all__ = [
    "ConnectedHardware",
    "ConnectedTwistBase",
    "ConnectedWholeBody",
]
