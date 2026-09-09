# Copyright 2026 Dimensional Inc.
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

"""Generic in-memory whole-body adapter for blueprints and tests."""

from __future__ import annotations

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState


class MockWholeBodyAdapter:
    """Stateful ordered whole-body IO without robot-specific behavior."""

    def __init__(
        self,
        *,
        dof: int,
        initial_positions: list[float] | None = None,
        limits: JointLimits | None = None,
        **_: object,
    ) -> None:
        positions = initial_positions or [0.0] * dof
        if len(positions) != dof:
            raise ValueError(f"expected {dof} initial positions, got {len(positions)}")
        self._states = [MotorState(q=position) for position in positions]
        self._limits = limits
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def activate(self) -> bool:
        return self._connected

    def deactivate(self) -> bool:
        return self._connected

    def read_motor_states(self) -> list[MotorState]:
        return list(self._states)

    def has_motor_states(self) -> bool:
        return self._connected

    def read_imu(self) -> IMUState:
        return IMUState()

    def get_limits(self) -> JointLimits | None:
        return self._limits

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self._connected or len(commands) != len(self._states):
            return False
        self._states = [
            MotorState(q=command.q, dq=command.dq, tau=command.tau) for command in commands
        ]
        return True
