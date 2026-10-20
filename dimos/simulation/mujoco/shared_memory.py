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

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import threading
from typing import Any, TypeVar

import numpy as np
from numpy.typing import NDArray

from dimos.simulation.mujoco.constants import VIDEO_HEIGHT, VIDEO_WIDTH
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

T = TypeVar("T")

# Video buffer: VIDEO_WIDTH x VIDEO_HEIGHT x 3 RGB
_video_size = VIDEO_WIDTH * VIDEO_HEIGHT * 3
# Depth buffers: 3 cameras x VIDEO_WIDTH x VIDEO_HEIGHT float32
_depth_size = VIDEO_WIDTH * VIDEO_HEIGHT * 4  # float32 = 4 bytes
# Odometry buffer: position(3) + quaternion(4) + timestamp(1) = 8 floats
_odom_size = 8 * 8  # 8 float64 values
# Command buffer: linear(3) + angular(3) = 6 floats
_cmd_size = 6 * 4  # 6 float32 values
# Lidar message buffer: for serialized lidar data
_lidar_size = 1024 * 1024 * 4  # 4MB should be enough for point cloud
_lidar_header_size = 8  # one float64 timestamp before the N x 3 float32 points
# Sequence/version numbers for detecting updates
_seq_size = 8 * 8  # 8 int64 values for different data types
# Control buffer: ready flag + stop flag + run flag
_control_size = 3 * 4  # 3 int32 values

_shm_sizes = {
    "video": _video_size,
    "depth_front": _depth_size,
    "depth_left": _depth_size,
    "depth_right": _depth_size,
    "odom": _odom_size,
    "cmd": _cmd_size,
    "lidar": _lidar_size,
    "lidar_len": 4,
    "seq": _seq_size,
    "control": _control_size,
}


def _unregister(shm: SharedMemory) -> SharedMemory:
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


@dataclass(frozen=True)
class ShmSet:
    video: SharedMemory
    depth_front: SharedMemory
    depth_left: SharedMemory
    depth_right: SharedMemory
    odom: SharedMemory
    cmd: SharedMemory
    lidar: SharedMemory
    lidar_len: SharedMemory
    seq: SharedMemory
    control: SharedMemory

    @classmethod
    def from_names(cls, shm_names: dict[str, str]) -> "ShmSet":
        return cls(**{k: _unregister(SharedMemory(name=shm_names[k])) for k in _shm_sizes.keys()})

    @classmethod
    def from_sizes(cls) -> "ShmSet":
        return cls(**{k: SharedMemory(create=True, size=_shm_sizes[k]) for k in _shm_sizes.keys()})

    def to_names(self) -> dict[str, str]:
        return {k: getattr(self, k).name for k in _shm_sizes.keys()}

    def as_list(self) -> list[SharedMemory]:
        return [getattr(self, k) for k in _shm_sizes.keys()]


class _ShmEnd:
    """One end of the shared memory.

    Each frame slot has a sequence counter used as a seqlock: the slot's single
    writer bumps it to an odd value before writing the payload and to the next
    even value after, so a reader that sees an odd counter, or a different
    counter after copying, knows the frame was torn and reads again. Callers
    see the frame number, counter // 2.
    """

    shm: ShmSet

    def _control(self) -> NDArray[Any]:
        return np.ndarray((3,), dtype=np.int32, buffer=self.shm.control.buf)

    def _seq_array(self) -> NDArray[Any]:
        return np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)

    def _seq(self, index: int) -> int:
        return int(self._seq_array()[index])

    @contextmanager
    def _writing(self, index: int) -> Iterator[None]:
        self._seq_array()[index] += 1  # odd: a write is in progress
        yield
        self._seq_array()[index] += 1  # even: the frame is complete

    def _read_frame(self, index: int, copy: Callable[[], T | None]) -> tuple[T | None, int]:
        """The latest complete frame of a slot and its number, or (None, 0)."""
        for _ in range(3):
            before = self._seq(index)
            if before == 0:
                return None, 0
            if before % 2 == 1:
                continue
            frame = copy()
            if frame is not None and self._seq(index) == before:
                return frame, before // 2
        return None, 0


class ShmReader(_ShmEnd):
    """The simulator's end: writes the sensors and reads the command."""

    _last_cmd_seq: int

    def __init__(self, shm_names: dict[str, str]) -> None:
        self.shm = ShmSet.from_names(shm_names)
        self._last_cmd_seq = 0

    def signal_ready(self) -> None:
        self._control()[0] = 1

    def should_stop(self) -> bool:
        return bool(self._control()[1] == 1)

    def signal_stop(self) -> None:
        self._control()[1] = 1

    def should_run(self) -> bool:
        return bool(self._control()[2] == 1)

    def write_video(self, pixels: NDArray[Any]) -> None:
        video_array: NDArray[Any] = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
        )
        with self._writing(0):
            video_array[:] = pixels

    def write_depth(self, front: NDArray[Any], left: NDArray[Any], right: NDArray[Any]) -> None:
        with self._writing(1):
            for shm, depth in (
                (self.shm.depth_front, front),
                (self.shm.depth_left, left),
                (self.shm.depth_right, right),
            ):
                depth_array: NDArray[Any] = np.ndarray(
                    (VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32, buffer=shm.buf
                )
                depth_array[:] = depth

    def write_odom(self, pos: NDArray[Any], quat: NDArray[Any], timestamp: float) -> None:
        odom_array: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
        with self._writing(2):
            odom_array[0:3] = pos
            odom_array[3:7] = quat
            odom_array[7] = timestamp

    def write_lidar(self, points: NDArray[Any], ts: float) -> None:
        n_points = len(points)
        nbytes = _lidar_header_size + n_points * 3 * 4
        if nbytes > self.shm.lidar.size:
            logger.error(f"Lidar data too large: {nbytes} > {self.shm.lidar.size}")
            return

        header: NDArray[Any] = np.ndarray((1,), dtype=np.float64, buffer=self.shm.lidar.buf)
        lidar_array: NDArray[Any] = np.ndarray(
            (n_points, 3), dtype=np.float32, buffer=self.shm.lidar.buf, offset=_lidar_header_size
        )
        len_array: NDArray[Any] = np.ndarray((1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf)
        with self._writing(4):
            header[0] = ts
            lidar_array[:] = points
            len_array[0] = n_points

    def read_command(self) -> tuple[NDArray[Any], NDArray[Any]] | None:
        command, seq = self._read_frame(3, self._copy_command)
        if command is None or seq <= self._last_cmd_seq:
            return None
        self._last_cmd_seq = seq
        return command

    def _copy_command(self) -> tuple[NDArray[Any], NDArray[Any]]:
        cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        return cmd_array[0:3].copy(), cmd_array[3:6].copy()

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.close()
            except Exception:
                pass


class ShmWriter(_ShmEnd):
    """The connection's end: owns the segments, reads the sensors and writes the command."""

    def __init__(self) -> None:
        self.shm = ShmSet.from_sizes()
        self._seq_array()[:] = 0
        cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        cmd_array[:] = 0
        self._control()[:] = 0  # [ready_flag, stop_flag, run_flag]
        # move() and its stop timer write the command from different threads.
        self._cmd_lock = threading.Lock()

    def is_ready(self) -> bool:
        return bool(self._control()[0] == 1)

    def signal_stop(self) -> None:
        self._control()[1] = 1

    def signal_run(self) -> None:
        """Let the simulator step the world; it stands still until then."""
        self._control()[2] = 1

    def read_video(self) -> tuple[NDArray[Any] | None, int]:
        return self._read_frame(0, self._copy_video)

    def _copy_video(self) -> NDArray[Any]:
        video_array: NDArray[Any] = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
        )
        return video_array.copy()

    def read_odom(self) -> tuple[tuple[NDArray[Any], NDArray[Any], float] | None, int]:
        return self._read_frame(2, self._copy_odom)

    def _copy_odom(self) -> tuple[NDArray[Any], NDArray[Any], float]:
        odom_array: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
        return odom_array[0:3].copy(), odom_array[3:7].copy(), float(odom_array[7])

    def write_command(self, linear: NDArray[Any], angular: NDArray[Any]) -> None:
        cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        with self._cmd_lock, self._writing(3):
            cmd_array[0:3] = linear
            cmd_array[3:6] = angular

    def read_lidar(self) -> tuple[tuple[NDArray[Any], float] | None, int]:
        """The latest lidar frame as (N x 3 float32 points, timestamp) and its sequence number."""
        return self._read_frame(4, self._copy_lidar)

    def _copy_lidar(self) -> tuple[NDArray[Any], float] | None:
        len_array: NDArray[Any] = np.ndarray((1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf)
        n_points = int(len_array[0])
        if _lidar_header_size + n_points * 3 * 4 > self.shm.lidar.size:
            return None  # a torn read of the length; _read_frame tries again
        header: NDArray[Any] = np.ndarray((1,), dtype=np.float64, buffer=self.shm.lidar.buf)
        lidar_array: NDArray[Any] = np.ndarray(
            (n_points, 3), dtype=np.float32, buffer=self.shm.lidar.buf, offset=_lidar_header_size
        )
        return lidar_array.copy(), float(header[0])

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.unlink()
            except Exception:
                pass

            try:
                shm.close()
            except Exception:
                pass
