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

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState


class _NativePositionWholeBodyAdapter:
    def __init__(self) -> None:
        self.accept = False
        self.position_writes: list[list[float]] = []

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        return None

    def is_connected(self) -> bool:
        return True

    def read_motor_states(self) -> list[MotorState]:
        return [MotorState(q=0.0), MotorState(q=0.0)]

    def has_motor_states(self) -> bool:
        return True

    def read_imu(self) -> IMUState:
        return IMUState()

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        raise AssertionError("native position path should not call write_motor_commands")

    def write_joint_positions(self, positions: list[float]) -> bool:
        self.position_writes.append(list(positions))
        return self.accept


class _MotorCommandWholeBodyAdapter:
    def __init__(self) -> None:
        self.motor_writes: list[list[MotorCommand]] = []

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        return None

    def is_connected(self) -> bool:
        return True

    def read_motor_states(self) -> list[MotorState]:
        return [MotorState(q=0.0), MotorState(q=0.0)]

    def has_motor_states(self) -> bool:
        return True

    def read_imu(self) -> IMUState:
        return IMUState()

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        self.motor_writes.append(commands)
        return True


def _component() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="body",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=["body/j1", "body/j2"],
    )


def test_whole_body_write_position_uses_native_adapter_and_commits_only_on_success() -> None:
    adapter = _NativePositionWholeBodyAdapter()
    connected = ConnectedWholeBody(adapter=adapter, component=_component())

    assert connected.write_position({"body/j1": 2.0}) is False
    adapter.accept = True
    assert connected.write_position({"body/j2": 3.0}) is True

    assert adapter.position_writes == [[2.0, 0.0], [0.0, 3.0]]


def test_whole_body_write_position_falls_back_to_motor_commands() -> None:
    adapter = _MotorCommandWholeBodyAdapter()
    connected = ConnectedWholeBody(adapter=adapter, component=_component())

    assert connected.write_position({"body/j1": 1.0}) is True

    commands = adapter.motor_writes[-1]
    assert [command.q for command in commands] == [1.0, 0.0]
    assert [command.dq for command in commands] == [0.0, 0.0]
