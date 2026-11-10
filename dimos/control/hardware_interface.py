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


from __future__ import annotations

import time
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
    def __init__(
        self,
        adapter: ManipulatorAdapter,
        component: HardwareComponent,
    ) -> None:
        self._adapter = adapter
        self._component = component
        self._joint_names: list[JointName] = list(component.joints)

        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> ManipulatorAdapter:
        return self._adapter

    @property
    def hardware_id(self) -> HardwareId:
        return self._component.hardware_id

    @property
    def joint_names(self) -> list[JointName]:
        return self._joint_names

    @property
    def component(self) -> HardwareComponent:
        return self._component

    @property
    def dof(self) -> int:
        return len(self._joint_names)

    def disconnect(self) -> None:
        self._adapter.disconnect()

    def get_limits(self) -> JointLimits | None:
        if self._component.limits is not None:
            return self._component.limits
        return self._adapter.get_limits()

    def ready_for_control(self) -> bool:
        has_live_state = getattr(self.adapter, "has_live_state", None)
        return has_live_state is None or bool(has_live_state())

    def read_state(self) -> dict[JointName, JointState]:
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
        if mode != ControlMode.VELOCITY and not self._initialized:
            self._initialize_last_commanded()

        unknown = set(commands) - set(self._joint_names)
        for joint_name in unknown - self._warned_unknown_joints:
            logger.warning(
                "Hardware received command for unknown joint",
                hardware_id=self.hardware_id,
                joint_name=joint_name,
                valid_joints=self._joint_names,
            )
        self._warned_unknown_joints.update(unknown)

        if mode == ControlMode.VELOCITY:
            ordered = [commands.get(name, 0.0) for name in self._joint_names]
        else:
            for joint_name, value in commands.items():
                if joint_name in self._joint_names:
                    self._last_commanded[joint_name] = value
            ordered = self._build_ordered_command()

        if mode != self._current_mode:
            if not self._adapter.set_control_mode(mode):
                logger.warning(f"Hardware {self.hardware_id} failed to switch to {mode.name}")
                return False
            self._current_mode = mode

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
        return [self._last_commanded[name] for name in self._joint_names]


class ConnectedTwistBase(ConnectedHardware):
    _twist_adapter: TwistBaseAdapter

    def __init__(
        self,
        adapter: TwistBaseAdapter,
        component: HardwareComponent,
    ) -> None:
        self._twist_adapter = adapter
        self._component = component
        self._joint_names = component.joints

        self._last_commanded: dict[str, float] = {name: 0.0 for name in self._joint_names}
        self._initialized = True
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> TwistBaseAdapter:  # type: ignore[override]
        return self._twist_adapter

    def disconnect(self) -> None:
        self._twist_adapter.disconnect()

    def get_limits(self) -> JointLimits | None:
        return self._component.limits

    def read_state(self) -> dict[JointName, JointState]:
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
        for joint_name, value in commands.items():
            if joint_name in self._last_commanded:
                self._last_commanded[joint_name] = value
            elif joint_name not in self._warned_unknown_joints:
                logger.warning(
                    f"TwistBase {self.hardware_id} received command for unknown joint "
                    f"{joint_name}. Valid joints: {self._joint_names}"
                )
                self._warned_unknown_joints.add(joint_name)

        ordered = self._build_ordered_command()
        return self._twist_adapter.write_velocities(ordered)


_DEFAULT_KP: float = 40.0
_DEFAULT_KD: float = 3.0


class ConnectedWholeBody(ConnectedHardware):
    _wb_adapter: WholeBodyAdapter

    def __init__(
        self,
        adapter: WholeBodyAdapter,
        component: HardwareComponent,
    ) -> None:
        self._wb_adapter = adapter
        self._component = component
        self._joint_names = component.joints

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

        self._last_commanded: dict[str, float] = {}
        self._initialized = False
        self._warned_unknown_joints: set[str] = set()
        self._current_mode: ControlMode | None = None

    @property
    def adapter(self) -> WholeBodyAdapter:  # type: ignore[override]
        return self._wb_adapter

    def disconnect(self) -> None:
        self._wb_adapter.disconnect()

    def get_limits(self) -> JointLimits | None:
        if self._component.limits is not None:
            return self._component.limits
        return self._wb_adapter.get_limits()

    def ready_for_control(self) -> bool:
        return self._wb_adapter.has_motor_states()

    def read_state_ts(self) -> float | None:
        getter = getattr(self._wb_adapter, "motor_states_ts", None)
        return getter() if callable(getter) else None

    def read_state(self) -> dict[JointName, JointState]:
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
                kp=self._kp_by_name[name],
                kd=self._kd_by_name[name],
                tau=0.0,
            )
            for name in self._joint_names
        ]
        return self._wb_adapter.write_motor_commands(motor_cmds)

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        return self._wb_adapter.write_motor_commands(commands)

    def _try_initialize_last_commanded(self) -> bool:
        if not self._wb_adapter.has_motor_states():
            return False
        states = self._wb_adapter.read_motor_states()
        for i, name in enumerate(self._joint_names):
            self._last_commanded[name] = states[i].q
        self._initialized = True
        return True
