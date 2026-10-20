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

Provides runtime wrappers for coordinator-managed hardware:
- ConnectedHardware: Wraps ManipulatorAdapter for joint-controlled arms
- ConnectedTwistBase: Wraps TwistBaseAdapter for velocity-commanded platforms
- ConnectedWholeBody: Wraps WholeBodyAdapter for full-body motor control

They share the same duck-type interface (read_state, write_command, etc.)
so the tick loop treats them uniformly.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.components import HardwareComponent, HardwareId, JointName, JointState
    from dimos.hardware.drive_trains.spec import TwistBaseAdapter
    from dimos.hardware.spec import JointLimits
    from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter

logger = setup_logger()


class ConnectedHardware:
    """Runtime wrapper for hardware connected to the coordinator.

    Wraps a ManipulatorAdapter with coordinator-specific features:
    - Joint names from HardwareComponent config
    - Receives complete commands prepared by the coordinator
    - Converts between joint names and array indices

    Gripper joints are the trailing entries of ``component.joints`` and
    ride the same array as every other joint; no unit conversion happens here.

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
        self._joint_names: list[JointName] = list(component.joints)

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

    def get_limits(self) -> JointLimits | None:
        """Return configured limits, or limits reported by the adapter."""
        if self._component.limits is not None:
            return self._component.limits
        return self._adapter.get_limits()

    def ready_for_control(self) -> bool:
        """Whether state is available and control commands may be sent.

        Adapters whose state can go stale opt in with ``has_live_state()``.
        """
        has_live_state = getattr(self.adapter, "has_live_state", None)
        return has_live_state is None or bool(has_live_state())

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
        """Write a complete coordinator-prepared command.

        Args:
            commands: Complete {joint_name: value} in native adapter units
            mode: Control mode

        Returns:
            True if command was sent successfully
        """
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION, ControlMode.VELOCITY):
            logger.warning(f"Hardware {self.hardware_id} does not support {mode.name}")
            return False
        ordered = [commands[name] for name in self._joint_names]

        # Switch control mode if needed
        if mode != self._current_mode:
            if not self._adapter.set_control_mode(mode):
                logger.warning(f"Hardware {self.hardware_id} failed to switch to {mode.name}")
                return False
            self._current_mode = mode

        if mode == ControlMode.VELOCITY:
            return self._adapter.write_joint_velocities(ordered)
        return self._adapter.write_joint_positions(ordered)


class ConnectedTwistBase(ConnectedHardware):
    """Runtime wrapper for a twist base connected to the coordinator.

    Inherits from ConnectedHardware and overrides behavior for
    velocity-commanded platforms (holonomic bases, drones, quadrupeds, etc.).

    Key differences from ConnectedHardware:
    - Positions come from odometry (or zeros if unavailable)
    - Efforts are always zero
    - write_command always sends velocities regardless of mode
    - Initial zero velocities and partial holds are prepared in the coordinator
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

    @property
    def adapter(self) -> TwistBaseAdapter:  # type: ignore[override]
        """The underlying twist base adapter."""
        return self._twist_adapter

    def disconnect(self) -> None:
        """Disconnect the underlying adapter."""
        self._twist_adapter.disconnect()

    def get_limits(self) -> JointLimits | None:
        """Return configured limits without requiring base adapter support."""
        return self._component.limits

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
            commands: Complete {joint_name: velocity} in native adapter units
            _mode: Control mode (ignored — twist bases always use velocity)

        Returns:
            True if command was sent successfully
        """
        ordered = [commands[name] for name in self._joint_names]
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

        # Resolve per-joint PD gains once at wire-up time.  Gains live on
        # the WB-specific sub-config; fall back to _DEFAULT_KP/_DEFAULT_KD
        # if the blueprint didn't supply a wb_config.
        n = len(self._joint_names)
        wb = component.wb_config
        kp_in = wb.kp if wb is not None else None
        kd_in = wb.kd if wb is not None else None
        if kp_in is not None:
            if len(kp_in) != n:
                raise ValueError(
                    f"HardwareComponent '{component.hardware_id}': wb_config.kp length "
                    f"{len(kp_in)} does not match joints length {n}"
                )
            self._kp = list(kp_in)
        else:
            self._kp = [_DEFAULT_KP] * n
        if kd_in is not None:
            if len(kd_in) != n:
                raise ValueError(
                    f"HardwareComponent '{component.hardware_id}': wb_config.kd length "
                    f"{len(kd_in)} does not match joints length {n}"
                )
            self._kd = list(kd_in)
        else:
            self._kd = [_DEFAULT_KD] * n
        self._kp_by_name = dict(zip(self._joint_names, self._kp, strict=False))
        self._kd_by_name = dict(zip(self._joint_names, self._kd, strict=False))

    @property
    def adapter(self) -> WholeBodyAdapter:  # type: ignore[override]
        """The underlying whole-body adapter."""
        return self._wb_adapter

    def disconnect(self) -> None:
        """Disconnect the underlying adapter."""
        self._wb_adapter.disconnect()

    def get_limits(self) -> JointLimits | None:
        """Return configured limits, or limits reported by the adapter."""
        if self._component.limits is not None:
            return self._component.limits
        return self._wb_adapter.get_limits()

    def ready_for_control(self) -> bool:
        """Wait for real motor feedback before exposing state or accepting commands."""
        return self._wb_adapter.has_motor_states()

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
        """Write position commands — converts to MotorCommand with per-joint PD gains.

        Only POSITION / SERVO_POSITION are supported; other modes are warned
        and dropped (matches ConnectedHardware's warn-and-skip pattern).
        Per-joint kp/kd come from ``component.wb_config`` (resolved in
        ``__init__``); fall back to ``_DEFAULT_KP``/``_DEFAULT_KD`` when
        the blueprint didn't supply gains.
        """
        from dimos.hardware.whole_body.spec import MotorCommand

        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            logger.warning(
                f"WholeBody {self.hardware_id} only supports POSITION/SERVO_POSITION; "
                f"got {mode.name} — skipping"
            )
            return False

        motor_cmds = [
            MotorCommand(
                q=commands[name],
                dq=0.0,
                kp=self._kp_by_name[name],
                kd=self._kd_by_name[name],
                tau=0.0,
            )
            for name in self._joint_names
        ]
        return self._wb_adapter.write_motor_commands(motor_cmds)

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        """Direct pass-through to adapter for full MotorCommand control."""
        return self._wb_adapter.write_motor_commands(commands)
