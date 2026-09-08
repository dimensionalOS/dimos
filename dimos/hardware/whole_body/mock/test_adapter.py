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

import pytest

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState


def test_write_motor_commands_connected_adapter_applies_ordered_commands() -> None:
    adapter = MockWholeBodyAdapter(dof=2, initial_positions=[0.1, 0.2])
    assert adapter.connect()

    assert adapter.write_motor_commands(
        [
            MotorCommand(q=0.3, dq=0.4, tau=0.5),
            MotorCommand(q=0.6, dq=0.7, tau=0.8),
        ]
    )

    assert adapter.read_motor_states() == [
        MotorState(q=0.3, dq=0.4, tau=0.5),
        MotorState(q=0.6, dq=0.7, tau=0.8),
    ]
    assert adapter.read_imu() == IMUState()


def test_write_motor_commands_wrong_command_count_rejects_without_state_change() -> None:
    adapter = MockWholeBodyAdapter(dof=2)
    assert adapter.connect()

    assert not adapter.write_motor_commands([MotorCommand(q=0.3)])
    assert adapter.read_motor_states() == [MotorState(), MotorState()]


def test_init_mismatched_initial_positions_raises_value_error() -> None:
    with pytest.raises(ValueError, match="expected 2 initial positions, got 1"):
        MockWholeBodyAdapter(dof=2, initial_positions=[0.1])


def test_write_motor_commands_disconnected_adapter_rejects_command() -> None:
    adapter = MockWholeBodyAdapter(dof=1)

    assert not adapter.write_motor_commands([MotorCommand(q=0.3)])
    assert adapter.read_motor_states() == [MotorState()]


def test_connection_lifecycle_controls_availability_and_activation() -> None:
    adapter = MockWholeBodyAdapter(dof=1)
    assert not adapter.activate()
    assert not adapter.deactivate()

    assert adapter.connect()
    assert adapter.is_connected()
    assert adapter.has_motor_states()
    assert adapter.activate()
    assert adapter.deactivate()

    adapter.disconnect()

    assert not adapter.is_connected()
    assert not adapter.has_motor_states()


def test_joint_limits_are_optional_adapter_metadata() -> None:
    limits = JointLimits(
        position_lower=[None, 0.0],
        position_upper=[None, 1.0],
        velocity_max=[None, None],
    )

    assert MockWholeBodyAdapter(dof=2).get_limits() is None
    assert MockWholeBodyAdapter(dof=2, limits=limits).get_limits() == limits
