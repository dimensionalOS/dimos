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

"""WholeBodyAdapter for benchmark runtime local SHM motor plane."""

from __future__ import annotations

from pathlib import Path
import time

from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState, WholeBodyAdapter
from dimos.simulation.runtime_client.shm_motor import MotorShmClient


class BenchmarkRuntimeWholeBodyAdapter(WholeBodyAdapter):
    """Attach-only whole-body adapter backed by local benchmark runtime SHM."""

    def __init__(
        self,
        *,
        dof: int,
        hardware_id: str,
        address: str | Path | None = None,
        domain_id: int = 0,
        motor_names: list[str] | None = None,
        connect_timeout_s: float = 2.0,
    ) -> None:
        self.dof = dof
        self.hardware_id = hardware_id
        self.address = str(address or hardware_id)
        self.domain_id = domain_id
        self.motor_names = motor_names or [f"{hardware_id}/joint{i + 1}" for i in range(dof)]
        self.connect_timeout_s = connect_timeout_s
        self._client: MotorShmClient | None = None
        self._last_sequence = -1
        self._last_states: list[MotorState] = [MotorState() for _ in range(dof)]

    def connect(self) -> bool:
        deadline = time.monotonic() + self.connect_timeout_s
        last_error: FileNotFoundError | None = None
        while time.monotonic() < deadline:
            try:
                self._client = MotorShmClient(self.address, self.motor_names)
                return True
            except FileNotFoundError as exc:
                last_error = exc
                time.sleep(0.01)
        if last_error is not None:
            return False
        return False

    def disconnect(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def is_connected(self) -> bool:
        return self._client is not None

    def read_motor_states(self) -> list[MotorState]:
        if self._client is None:
            return self._last_states
        sequence, states = self._client.read_states()
        self._last_sequence = sequence
        self._last_states = states
        return states

    def has_motor_states(self) -> bool:
        if self._client is None:
            return False
        try:
            sequence, states = self._client.read_states()
        except Exception:
            return False
        self._last_sequence = sequence
        self._last_states = states
        return len(states) == self.dof

    def read_imu(self) -> IMUState:
        if self._client is None:
            return IMUState()
        return self._client.read_imu()

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if self._client is None:
            return False
        return self._client.write_commands(commands)

    def read_gripper_position(self) -> float | None:
        """Expose the final runtime motor as a gripper position for demos."""

        if not self._last_states:
            return None
        if self._client is not None:
            try:
                _, self._last_states = self._client.read_states()
            except Exception:
                pass
        return self._last_states[-1].q

    def write_gripper_position(self, position: float) -> bool:
        """Command the final runtime motor as a gripper through the SHM bridge."""

        if self._client is None:
            return False
        _, states = self._client.read_states()
        if len(states) != self.dof:
            return False
        commands = [MotorCommand(q=state.q, dq=0.0, kp=40.0, kd=3.0, tau=0.0) for state in states]
        commands[-1] = MotorCommand(q=position, dq=0.0, kp=40.0, kd=3.0, tau=0.0)
        ok = self._client.write_commands(commands)
        if ok:
            self._last_states = states
        return ok


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Register benchmark runtime adapter."""

    registry.register("benchmark_runtime", BenchmarkRuntimeWholeBodyAdapter)
