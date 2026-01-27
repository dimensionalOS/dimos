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

from dataclasses import dataclass
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
from typing import Any

import numpy as np
from numpy.typing import NDArray

_FLOAT_BYTES = 8
_INT_BYTES = 4
_INT64_BYTES = 8


def _shm_sizes(dof: int) -> dict[str, int]:
    return {
        "control": 2 * _INT_BYTES,  # ready, stop
        "seq": 2 * _INT64_BYTES,  # cmd, state
        "mode": _INT_BYTES,  # command mode
        "cmd_pos": dof * _FLOAT_BYTES,
        "cmd_vel": dof * _FLOAT_BYTES,
        "cmd_eff": dof * _FLOAT_BYTES,
        "state_pos": dof * _FLOAT_BYTES,
        "state_vel": dof * _FLOAT_BYTES,
        "state_eff": dof * _FLOAT_BYTES,
    }


def _unregister(shm: SharedMemory) -> SharedMemory:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


@dataclass(frozen=True)
class ShmSet:
    control: SharedMemory
    seq: SharedMemory
    mode: SharedMemory
    cmd_pos: SharedMemory
    cmd_vel: SharedMemory
    cmd_eff: SharedMemory
    state_pos: SharedMemory
    state_vel: SharedMemory
    state_eff: SharedMemory

    @classmethod
    def from_names(cls, shm_names: dict[str, str]) -> "ShmSet":
        return cls(**{k: _unregister(SharedMemory(name=shm_names[k])) for k in shm_names})

    @classmethod
    def from_sizes(cls, sizes: dict[str, int]) -> "ShmSet":
        return cls(**{k: _unregister(SharedMemory(create=True, size=sizes[k])) for k in sizes})

    def to_names(self) -> dict[str, str]:
        return {name: getattr(self, name).name for name in self.__dataclass_fields__}

    def as_list(self) -> list[SharedMemory]:
        return [getattr(self, name) for name in self.__dataclass_fields__]


class ShmReader:
    shm: ShmSet
    _last_cmd_seq: int
    _dof: int

    def __init__(self, shm_names: dict[str, str], dof: int) -> None:
        self.shm = ShmSet.from_names(shm_names)
        self._last_cmd_seq = 0
        self._dof = dof

    def signal_ready(self) -> None:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[0] = 1

    def should_stop(self) -> bool:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(control_array[1] == 1)

    def read_command(self) -> tuple[int, NDArray[Any], NDArray[Any], NDArray[Any]] | None:
        seq = self._get_seq(0)
        if seq <= self._last_cmd_seq:
            return None
        self._last_cmd_seq = seq
        mode = int(np.ndarray((1,), dtype=np.int32, buffer=self.shm.mode.buf)[0])
        cmd_pos: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.cmd_pos.buf
        ).copy()
        cmd_vel: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.cmd_vel.buf
        ).copy()
        cmd_eff: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.cmd_eff.buf
        ).copy()
        return mode, cmd_pos, cmd_vel, cmd_eff

    def write_state(
        self,
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
    ) -> None:
        pos_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_pos.buf
        )
        vel_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_vel.buf
        )
        eff_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_eff.buf
        )
        pos_array[:] = positions
        vel_array[:] = velocities
        eff_array[:] = efforts
        self._increment_seq(1)

    def _increment_seq(self, index: int) -> None:
        seq_array: NDArray[Any] = np.ndarray((2,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[index] += 1

    def _get_seq(self, index: int) -> int:
        seq_array: NDArray[Any] = np.ndarray((2,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq_array[index])

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.close()
            except Exception:
                pass


class ShmWriter:
    shm: ShmSet
    _dof: int

    def __init__(self, dof: int) -> None:
        self._dof = dof
        sizes = _shm_sizes(dof)
        self.shm = ShmSet.from_sizes(sizes)

        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[:] = 0

        seq_array: NDArray[Any] = np.ndarray((2,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[:] = 0

        mode_array: NDArray[Any] = np.ndarray((1,), dtype=np.int32, buffer=self.shm.mode.buf)
        mode_array[0] = 0

        for name in ("cmd_pos", "cmd_vel", "cmd_eff", "state_pos", "state_vel", "state_eff"):
            arr: NDArray[Any] = np.ndarray(
                (dof,), dtype=np.float64, buffer=getattr(self.shm, name).buf
            )
            arr[:] = 0.0

    def is_ready(self) -> bool:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(control_array[0] == 1)

    def signal_stop(self) -> None:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[1] = 1

    def write_command(
        self,
        mode: int,
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        mode_array: NDArray[Any] = np.ndarray((1,), dtype=np.int32, buffer=self.shm.mode.buf)
        mode_array[0] = mode

        if positions is not None:
            pos_array: NDArray[Any] = np.ndarray(
                (self._dof,), dtype=np.float64, buffer=self.shm.cmd_pos.buf
            )
            count = min(len(positions), self._dof)
            pos_array[:count] = positions[:count]
        if velocities is not None:
            vel_array: NDArray[Any] = np.ndarray(
                (self._dof,), dtype=np.float64, buffer=self.shm.cmd_vel.buf
            )
            count = min(len(velocities), self._dof)
            vel_array[:count] = velocities[:count]
        if efforts is not None:
            eff_array: NDArray[Any] = np.ndarray(
                (self._dof,), dtype=np.float64, buffer=self.shm.cmd_eff.buf
            )
            count = min(len(efforts), self._dof)
            eff_array[:count] = efforts[:count]

        self._increment_seq(0)

    def read_state(self) -> tuple[list[float], list[float], list[float]]:
        pos_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_pos.buf
        )
        vel_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_vel.buf
        )
        eff_array: NDArray[Any] = np.ndarray(
            (self._dof,), dtype=np.float64, buffer=self.shm.state_eff.buf
        )
        return pos_array.tolist(), vel_array.tolist(), eff_array.tolist()

    def _increment_seq(self, index: int) -> None:
        seq_array: NDArray[Any] = np.ndarray((2,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[index] += 1

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.close()
            except Exception:
                pass
            try:
                shm.unlink()
            except Exception:
                pass
