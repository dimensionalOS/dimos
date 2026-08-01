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

from dimos.hardware.whole_body.benchmark_runtime.adapter import BenchmarkRuntimeWholeBodyAdapter
from dimos.hardware.whole_body.spec import MotorCommand, MotorState
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner


def test_motor_shm_round_trip() -> None:
    key = "test-runtime-shm-round-trip"
    names = ["fakebot/joint1", "fakebot/joint2"]
    owner = MotorShmOwner(key, names)
    adapter = BenchmarkRuntimeWholeBodyAdapter(
        dof=2,
        hardware_id="fakebot",
        address=key,
        motor_names=names,
    )
    try:
        assert adapter.connect()
        owner.write_state([MotorState(q=0.1), MotorState(q=0.2)], sequence=1)
        states = adapter.read_motor_states()
        assert [state.q for state in states] == [0.1, 0.2]

        assert adapter.write_motor_commands([MotorCommand(q=0.3), MotorCommand(q=0.4)])
        sequence, commands = owner.read_commands()
        assert sequence == 1
        assert [command.q for command in commands] == [0.3, 0.4]
    finally:
        adapter.disconnect()
        owner.close()
        owner.unlink()
