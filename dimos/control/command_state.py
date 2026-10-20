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

"""Coordinator-owned command preparation and existing hold semantics."""

from __future__ import annotations

import time

from dimos.control.hardware_interface import (
    ConnectedHardware,
    ConnectedTwistBase,
    ConnectedWholeBody,
)
from dimos.control.task import JointCommandOutput
from dimos.hardware.manipulators.spec import ControlMode
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CommandState:
    """One authoritative target state, owned by the ControlCoordinator."""

    def __init__(self) -> None:
        self._position_holds: dict[str, dict[str, float]] = {}
        self._base_holds: dict[str, dict[str, float]] = {}
        self.commands: dict[str, JointCommandOutput] = {}

    def register(self, hw: ConnectedHardware) -> None:
        """Twist bases have a defined zero-velocity hold before their first command."""
        if isinstance(hw, ConnectedTwistBase) and hw.hardware_id not in self.commands:
            self._base_holds[hw.hardware_id] = dict.fromkeys(hw.joint_names, 0.0)
            self.commands[hw.hardware_id] = JointCommandOutput(
                joint_names=list(hw.joint_names),
                velocities=[0.0] * hw.dof,
                mode=ControlMode.VELOCITY,
            )

    def remove(self, hardware_id: str) -> None:
        self._position_holds.pop(hardware_id, None)
        self._base_holds.pop(hardware_id, None)
        self.commands.pop(hardware_id, None)

    def prepare(
        self, hw: ConnectedHardware, values: dict[str, float], mode: ControlMode
    ) -> JointCommandOutput | None:
        """Expand a supported command before I/O; rejection never corrupts holds."""
        names = hw.joint_names
        hw_id = hw.hardware_id
        if isinstance(hw, ConnectedTwistBase):
            mode = ControlMode.VELOCITY
            held = self._base_holds.setdefault(hw_id, dict.fromkeys(names, 0.0))
        elif mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            if hw_id not in self._position_holds:
                initial = self._initial_positions(hw)
                if initial is None:
                    return None
                self._position_holds[hw_id] = dict(zip(names, initial, strict=True))
            held = self._position_holds[hw_id]
        elif mode == ControlMode.VELOCITY and not isinstance(hw, ConnectedWholeBody):
            held = dict.fromkeys(names, 0.0)
        else:
            logger.warning(f"Hardware {hw_id} does not support {mode.name}")
            return None

        for name, value in values.items():
            if name in held:
                held[name] = value
            else:
                logger.warning(f"Hardware {hw_id} received unknown joint {name}")
        ordered = [held[name] for name in names]
        command = JointCommandOutput(
            joint_names=list(names),
            mode=mode,
            positions=ordered
            if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION)
            else None,
            velocities=ordered if mode == ControlMode.VELOCITY else None,
        )
        self.commands[hw_id] = command
        return command

    def _initial_positions(self, hw: ConnectedHardware) -> list[float] | None:
        if isinstance(hw, ConnectedWholeBody):
            if not hw.adapter.has_motor_states():
                return None
            states = hw.adapter.read_motor_states()
            return [states[i].q for i in range(hw.dof)]
        for _ in range(10):
            try:
                positions = hw.adapter.read_joint_positions()
                return [positions[i] for i in range(hw.dof)]
            except Exception:
                time.sleep(0.01)
        raise RuntimeError(
            f"Hardware {hw.hardware_id} failed to read initial positions after retries"
        )
