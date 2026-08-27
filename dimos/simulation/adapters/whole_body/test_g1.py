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

from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter
from dimos.simulation.adapters.whole_body.g1 import SimMujocoG1WholeBodyAdapter


def test_sim_g1_adapter_satisfies_whole_body_protocol() -> None:
    adapter = SimMujocoG1WholeBodyAdapter(address=Path("unused.xml"))

    assert isinstance(adapter, WholeBodyAdapter)
    assert adapter.get_limits() is None


def test_sim_g1_adapter_rejects_commands_until_activated(mocker: Any) -> None:
    shm_class = mocker.patch("dimos.simulation.adapters.whole_body.g1.ManipShmReader")
    shm = shm_class.return_value
    shm.is_ready.return_value = True
    adapter = SimMujocoG1WholeBodyAdapter(address=Path("unused.xml"))

    try:
        assert adapter.connect()
        commands = [MotorCommand(q=0.1, kp=10.0, kd=1.0)] * 29

        assert adapter.write_motor_commands(commands) is False
        assert adapter.activate()
        assert adapter.write_motor_commands(commands) is True

        shm.write_pd_tau_command.assert_called_once()
    finally:
        adapter.disconnect()
