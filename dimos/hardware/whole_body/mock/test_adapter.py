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

from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState


def test_mock_whole_body_applies_ordered_commands() -> None:
    adapter = MockWholeBodyAdapter(
        dof=2,
        initial_positions=[0.1, 0.2],
        address=None,
        hardware_id="test_robot",
        domain_id=0,
    )
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


def test_mock_whole_body_rejects_wrong_command_count() -> None:
    adapter = MockWholeBodyAdapter(dof=2)
    assert adapter.connect()

    assert not adapter.write_motor_commands([MotorCommand(q=0.3)])
    assert adapter.read_motor_states() == [MotorState(), MotorState()]
