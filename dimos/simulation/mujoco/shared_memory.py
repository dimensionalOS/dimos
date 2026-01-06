# Copyright 2025 Dimensional Inc.
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
import pickle
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.simulation.mujoco.constants import VIDEO_HEIGHT, VIDEO_WIDTH
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
# Sequence/version numbers for detecting updates
_seq_size = 8 * 8  # 8 int64 values for different data types
# Control buffer: ready flag + stop flag
_control_size = 2 * 4  # 2 int32 values

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
        return cls(
            **{
                k: _unregister(SharedMemory(create=True, size=_shm_sizes[k]))
                for k in _shm_sizes.keys()
            }
        )

    def to_names(self) -> dict[str, str]:
        return {k: getattr(self, k).name for k in _shm_sizes.keys()}

    def as_list(self) -> list[SharedMemory]:
        return [getattr(self, k) for k in _shm_sizes.keys()]


class ShmReader:
    shm: ShmSet
    _last_cmd_seq: int

    def __init__(self, shm_names: dict[str, str]) -> None:
        self.shm = ShmSet.from_names(shm_names)
        self._last_cmd_seq = 0

    def signal_ready(self) -> None:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[0] = 1  # ready flag

    def should_stop(self) -> bool:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(control_array[1] == 1)  # stop flag

    def write_video(self, pixels: NDArray[Any]) -> None:
        video_array: NDArray[Any] = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
        )
        video_array[:] = pixels
        self._increment_seq(0)

    def write_depth(self, front: NDArray[Any], left: NDArray[Any], right: NDArray[Any]) -> None:
        # Front camera
        depth_array: NDArray[Any] = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32, buffer=self.shm.depth_front.buf
        )
        depth_array[:] = front

        # Left camera
        depth_array = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32, buffer=self.shm.depth_left.buf
        )
        depth_array[:] = left

        # Right camera
        depth_array = np.ndarray(
            (VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32, buffer=self.shm.depth_right.buf
        )
        depth_array[:] = right

        self._increment_seq(1)

    def write_odom(self, pos: NDArray[Any], quat: NDArray[Any], timestamp: float) -> None:
        odom_array: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
        odom_array[0:3] = pos
        odom_array[3:7] = quat
        odom_array[7] = timestamp
        self._increment_seq(2)

    def write_lidar(self, lidar_msg: LidarMessage) -> None:
        data = pickle.dumps(lidar_msg)
        data_len = len(data)

        if data_len > self.shm.lidar.size:
            logger.error(f"Lidar data too large: {data_len} > {self.shm.lidar.size}")
            return

        # Write length
        len_array: NDArray[Any] = np.ndarray((1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf)
        len_array[0] = data_len

        # Write data
        lidar_array: NDArray[Any] = np.ndarray(
            (data_len,), dtype=np.uint8, buffer=self.shm.lidar.buf
        )
        lidar_array[:] = np.frombuffer(data, dtype=np.uint8)

        self._increment_seq(4)

    def read_command(self) -> tuple[NDArray[Any], NDArray[Any]] | None:
        seq = self._get_seq(3)
        if seq > self._last_cmd_seq:
            self._last_cmd_seq = seq
            cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
            linear = cmd_array[0:3].copy()
            angular = cmd_array[3:6].copy()
            return linear, angular
        return None

    def _increment_seq(self, index: int) -> None:
        seq_array: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[index] += 1

    def _get_seq(self, index: int) -> int:
        seq_array: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq_array[index])

    def cleanup(self) -> None:
        for shm in self.shm.as_list():
            try:
                shm.close()
            except Exception:
                pass


class ShmWriter:
    shm: ShmSet

    def __init__(self) -> None:
        self.shm = ShmSet.from_sizes()

        seq_array: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[:] = 0

        cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        cmd_array[:] = 0

        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[:] = 0  # [ready_flag, stop_flag]

    def is_ready(self) -> bool:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        return bool(control_array[0] == 1)

    def signal_stop(self) -> None:
        control_array: NDArray[Any] = np.ndarray((2,), dtype=np.int32, buffer=self.shm.control.buf)
        control_array[1] = 1  # Set stop flag

    def read_video(self) -> tuple[NDArray[Any] | None, int]:
        seq = self._get_seq(0)
        if seq > 0:
            video_array: NDArray[Any] = np.ndarray(
                (VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8, buffer=self.shm.video.buf
            )
            return video_array.copy(), seq
        return None, 0

    def read_odom(self) -> tuple[tuple[NDArray[Any], NDArray[Any], float] | None, int]:
        seq = self._get_seq(2)
        if seq > 0:
            odom_array: NDArray[Any] = np.ndarray((8,), dtype=np.float64, buffer=self.shm.odom.buf)
            pos = odom_array[0:3].copy()
            quat = odom_array[3:7].copy()
            timestamp = odom_array[7]
            return (pos, quat, timestamp), seq
        return None, 0

    def write_command(self, linear: NDArray[Any], angular: NDArray[Any]) -> None:
        cmd_array: NDArray[Any] = np.ndarray((6,), dtype=np.float32, buffer=self.shm.cmd.buf)
        cmd_array[0:3] = linear
        cmd_array[3:6] = angular
        self._increment_seq(3)

    def read_lidar(self) -> tuple[LidarMessage | None, int]:
        seq = self._get_seq(4)
        if seq > 0:
            # Read length
            len_array: NDArray[Any] = np.ndarray(
                (1,), dtype=np.uint32, buffer=self.shm.lidar_len.buf
            )
            data_len = int(len_array[0])

            if data_len > 0 and data_len <= self.shm.lidar.size:
                # Read data
                lidar_array: NDArray[Any] = np.ndarray(
                    (data_len,), dtype=np.uint8, buffer=self.shm.lidar.buf
                )
                data = bytes(lidar_array)

                try:
                    lidar_msg = pickle.loads(data)
                    return lidar_msg, seq
                except Exception as e:
                    logger.error(f"Failed to deserialize lidar message: {e}")
        return None, 0

    def _increment_seq(self, index: int) -> None:
        seq_array: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_array[index] += 1

    def _get_seq(self, index: int) -> int:
        seq_array: NDArray[Any] = np.ndarray((8,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq_array[index])

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
