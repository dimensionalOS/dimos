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

"""Local shared-memory motor command/state bridge for benchmark runtimes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from multiprocessing import shared_memory
import struct
import time
from typing import cast

from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP, IMUState, MotorCommand, MotorState

_HEADER = struct.Struct("!QI")
_DEFAULT_SIZE = 64 * 1024
JsonPayload = dict[str, str | int | float | list[str] | list[float]]


@dataclass(frozen=True)
class MotorShmNames:
    """Shared-memory block names for one runtime motor plane."""

    command: str
    state: str


def shm_names(key: str) -> MotorShmNames:
    """Create deterministic local shared-memory names from a session key."""

    safe = "".join(ch if ch.isalnum() else "_" for ch in key)[:80]
    return MotorShmNames(command=f"dimos_rt_{safe}_cmd", state=f"dimos_rt_{safe}_state")


class JsonShmBlock:
    """One fixed-size shared-memory JSON frame with sequence metadata."""

    def __init__(self, name: str, *, create: bool, size: int = _DEFAULT_SIZE) -> None:
        self.name = name
        self._owner = create
        self._shm = shared_memory.SharedMemory(name=name, create=create, size=size)
        self._size = size
        if create:
            self.write({"sequence": 0})

    def write(self, payload: JsonPayload) -> None:
        sequence = _int_value(payload.get("sequence", 0))
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        max_payload = self._size - _HEADER.size
        if len(data) > max_payload:
            raise ValueError(
                f"payload too large for SHM block {self.name}: {len(data)}>{max_payload}"
            )
        buf = cast("memoryview", self._shm.buf)
        buf[: _HEADER.size] = _HEADER.pack(sequence, len(data))
        buf[_HEADER.size : _HEADER.size + len(data)] = data

    def read(self) -> JsonPayload:
        buf = cast("memoryview", self._shm.buf)
        sequence, length = _HEADER.unpack(bytes(buf[: _HEADER.size]))
        if length == 0:
            return {"sequence": sequence}
        data = bytes(buf[_HEADER.size : _HEADER.size + length])
        value = json.loads(data.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError(f"SHM block {self.name} did not contain a JSON object")
        value["sequence"] = sequence
        return value

    def close(self) -> None:
        self._shm.close()

    def unlink(self) -> None:
        if self._owner:
            try:
                self._shm.unlink()
            except FileNotFoundError:
                pass


class MotorShmOwner:
    """Owner side of the local motor SHM plane.

    The DimOS runtime client creates/unlinks the blocks. The WholeBody adapter
    attaches and never unlinks.
    """

    def __init__(self, key: str, motor_names: list[str], *, size: int = _DEFAULT_SIZE) -> None:
        self.key = key
        self.motor_names = motor_names
        names = shm_names(key)
        self.command = JsonShmBlock(names.command, create=True, size=size)
        self.state = JsonShmBlock(names.state, create=True, size=size)
        self.write_state([MotorState() for _ in motor_names], sequence=0)
        self.write_commands([MotorCommand(q=0.0, dq=0.0) for _ in motor_names], sequence=0)

    def write_state(self, states: list[MotorState], *, sequence: int) -> None:
        if len(states) != len(self.motor_names):
            raise ValueError(f"expected {len(self.motor_names)} states, got {len(states)}")
        self.state.write(
            {
                "sequence": sequence,
                "timestamp_s": time.time(),
                "names": self.motor_names,
                "q": [state.q for state in states],
                "dq": [state.dq for state in states],
                "tau": [state.tau for state in states],
            }
        )

    def read_commands(self) -> tuple[int, list[MotorCommand]]:
        payload = self.command.read()
        names = _list_str(payload.get("names", []))
        if names and names != self.motor_names:
            raise ValueError(f"command names mismatch: expected {self.motor_names}, got {names}")
        sequence = _int_value(payload.get("sequence", 0))
        q = _list_float(payload.get("q", []), len(self.motor_names), POS_STOP)
        dq = _list_float(payload.get("dq", []), len(self.motor_names), VEL_STOP)
        kp = _list_float(payload.get("kp", []), len(self.motor_names), 0.0)
        kd = _list_float(payload.get("kd", []), len(self.motor_names), 0.0)
        tau = _list_float(payload.get("tau", []), len(self.motor_names), 0.0)
        return sequence, [
            MotorCommand(q=q[i], dq=dq[i], kp=kp[i], kd=kd[i], tau=tau[i])
            for i in range(len(self.motor_names))
        ]

    def write_commands(self, commands: list[MotorCommand], *, sequence: int) -> None:
        if len(commands) != len(self.motor_names):
            raise ValueError(f"expected {len(self.motor_names)} commands, got {len(commands)}")
        self.command.write(
            {
                "sequence": sequence,
                "timestamp_s": time.time(),
                "names": self.motor_names,
                "q": [command.q for command in commands],
                "dq": [command.dq for command in commands],
                "kp": [command.kp for command in commands],
                "kd": [command.kd for command in commands],
                "tau": [command.tau for command in commands],
            }
        )

    def close(self) -> None:
        self.command.close()
        self.state.close()

    def unlink(self) -> None:
        self.command.unlink()
        self.state.unlink()


class MotorShmClient:
    """Attach-only client used by the local WholeBodyAdapter."""

    def __init__(self, key: str, motor_names: list[str], *, size: int = _DEFAULT_SIZE) -> None:
        self.key = key
        self.motor_names = motor_names
        names = shm_names(key)
        self.command = JsonShmBlock(names.command, create=False, size=size)
        self.state = JsonShmBlock(names.state, create=False, size=size)
        self._command_sequence = 0

    def read_states(self) -> tuple[int, list[MotorState]]:
        payload = self.state.read()
        names = _list_str(payload.get("names", []))
        if names and names != self.motor_names:
            raise ValueError(f"state names mismatch: expected {self.motor_names}, got {names}")
        sequence = _int_value(payload.get("sequence", 0))
        q = _list_float(payload.get("q", []), len(self.motor_names), 0.0)
        dq = _list_float(payload.get("dq", []), len(self.motor_names), 0.0)
        tau = _list_float(payload.get("tau", []), len(self.motor_names), 0.0)
        return sequence, [
            MotorState(q=q[i], dq=dq[i], tau=tau[i]) for i in range(len(self.motor_names))
        ]

    def write_commands(self, commands: list[MotorCommand]) -> bool:
        if len(commands) != len(self.motor_names):
            raise ValueError(f"expected {len(self.motor_names)} commands, got {len(commands)}")
        self._command_sequence += 1
        self.command.write(
            {
                "sequence": self._command_sequence,
                "timestamp_s": time.time(),
                "names": self.motor_names,
                "q": [command.q for command in commands],
                "dq": [command.dq for command in commands],
                "kp": [command.kp for command in commands],
                "kd": [command.kd for command in commands],
                "tau": [command.tau for command in commands],
            }
        )
        return True

    def read_imu(self) -> IMUState:
        return IMUState()

    def close(self) -> None:
        self.command.close()
        self.state.close()


def _list_str(value: object) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value]


def _list_float(value: object, length: int, default: float) -> list[float]:
    if not isinstance(value, list) or not value:
        return [default] * length
    result = [float(item) for item in value]
    if len(result) != length:
        raise ValueError(f"expected list length {length}, got {len(result)}")
    return result


def _int_value(value: object) -> int:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int | float | str):
        return int(value)
    return 0
