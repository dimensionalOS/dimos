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

from pathlib import Path
from typing import Any

from pytest_mock import MockerFixture

from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter
from dimos.simulation.adapters.whole_body.microduck import (
    SimMujocoMicroDuckWholeBodyAdapter,
)


def test_adapter_satisfies_protocol_and_uses_native_position_commands(
    mocker: MockerFixture,
) -> None:
    shared_memory = mocker.Mock()
    shared_memory.is_ready.return_value = True
    shared_memory.num_joints.return_value = 14
    reader = mocker.patch(
        "dimos.simulation.adapters.whole_body.microduck.ManipShmReader",
        return_value=shared_memory,
    )
    adapter = SimMujocoMicroDuckWholeBodyAdapter(address=Path("microduck.xml"))

    try:
        assert isinstance(adapter, WholeBodyAdapter)
        assert adapter.connect()
        commands = [MotorCommand(q=float(index), kp=99.0, kd=88.0, tau=77.0) for index in range(14)]

        assert adapter.write_motor_commands(commands)

        reader.assert_called_once()
        shared_memory.write_position_command.assert_called_once_with([float(i) for i in range(14)])
        assert not shared_memory.write_pd_tau_command.called
    finally:
        adapter.disconnect()


def test_adapter_rejects_wrong_command_count(mocker: MockerFixture) -> None:
    shared_memory: Any = mocker.Mock()
    shared_memory.is_ready.return_value = True
    shared_memory.num_joints.return_value = 14
    mocker.patch(
        "dimos.simulation.adapters.whole_body.microduck.ManipShmReader",
        return_value=shared_memory,
    )
    adapter = SimMujocoMicroDuckWholeBodyAdapter(address=Path("microduck.xml"))

    try:
        assert adapter.connect()
        assert adapter.write_motor_commands([MotorCommand()] * 13) is False
        assert not shared_memory.write_position_command.called
    finally:
        adapter.disconnect()
