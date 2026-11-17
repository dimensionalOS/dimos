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

"""MuJoCo 4 RGB-D rendering through a bounded process and shared pixel buffer."""

from dataclasses import replace
import mmap
from multiprocessing.connection import Connection
import socket
import subprocess
import sys
import tempfile
import threading
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import UsdModel
from dimos.sim2.sensors.camera.config import CameraConfig

_NO_COLOR: NDArray[np.uint8] = np.empty((0, 0, 3), dtype=np.uint8)
_NO_DEPTH: NDArray[np.float32] = np.empty((0, 0), dtype=np.float32)


def _shared_views(
    memory: mmap.mmap, cameras: tuple[CameraConfig, ...]
) -> dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]]:
    images = {}
    offset = 0
    for camera in cameras:
        color, depth = _NO_COLOR, _NO_DEPTH
        if camera.color:
            color = np.ndarray((camera.height, camera.width, 3), np.uint8, memory, offset)
            offset += color.nbytes
        if camera.depth:
            offset = (offset + 3) // 4 * 4
            depth = np.ndarray((camera.height, camera.width), np.float32, memory, offset)
            offset += depth.nbytes
        images[camera.frame_id] = color, depth
    return images


class MujocoRenderer:
    """MuJoCo Filament RGB-D, with shared pixels and one capture in flight.

    Color-only, depth-only and RGB-D devices share retained scene resources.
    Depth is float32 optical-axis distance in metres, with zero for no return.

    Cocoa's offscreen swapchain needs the process main thread. A native renderer
    on a sensor thread deadlocks while the DimOS worker's main thread handles RPC.
    Keep that requirement inside this adapter; physics and other sensors keep
    their existing workers and the camera module keeps its latest-pose mailbox.
    """

    def __init__(self) -> None:
        self._process: subprocess.Popen[bytes] | None = None
        self._connection: Connection | None = None
        self._memory: mmap.mmap | None = None
        self._images: dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]] = {}
        self._cancelled = threading.Event()

    def _receive(self, timeout: float) -> Any:
        if self._connection is None or not self._connection.poll(timeout):
            self.cancel()
            raise TimeoutError("Filament renderer did not answer within its deadline")
        ok, result = self._connection.recv()
        if not ok:
            raise RuntimeError(f"Filament renderer: {result}")
        return result

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        if self._process is not None:
            raise RuntimeError("close the existing camera scene before loading another")
        if (
            not cameras
            or len({c.frame_id for c in cameras}) != len(cameras)
            or any(not c.frame_id for c in cameras)
        ):
            raise ValueError("cameras require distinct, nonempty frame IDs")
        self._cancelled.clear()
        parent, child = socket.socketpair()
        self._connection = Connection(parent.detach())
        size = sum(c.width * c.height * (3 * c.color + 4 * c.depth) + 3 for c in cameras)
        try:
            # Inherited file descriptors need no named SHM resource tracker and
            # disappear even if either process crashes. Pixels never enter the pipe.
            with tempfile.TemporaryFile() as pixels:
                pixels.truncate(size)
                self._memory = mmap.mmap(pixels.fileno(), size)
                self._images = _shared_views(self._memory, cameras)
                self._process = subprocess.Popen(
                    [
                        sys.executable,
                        "-m",
                        f"{__package__}.mujoco_native",
                        str(child.fileno()),
                        str(pixels.fileno()),
                    ],
                    pass_fds=(child.fileno(), pixels.fileno()),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                )
            child.close()
            # Copy just the description fields, not cached USD objects.
            self._connection.send((replace(scene), cameras))
            self._receive(60.0)
        except BaseException:
            child.close()
            self.close()
            raise

    def render(
        self, sample: WorldState, camera_ids: tuple[str, ...]
    ) -> dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]]:
        if len(set(camera_ids)) != len(camera_ids) or not set(camera_ids) <= self._images.keys():
            raise ValueError("render requests require distinct configured camera IDs")
        if self._cancelled.is_set() or self._connection is None:
            raise RuntimeError("Filament renderer is closed")
        if not camera_ids:
            return {}
        self._connection.send((sample.lcm_encode(), camera_ids))
        self._receive(5.0)
        return {
            name: (self._images[name][0].copy(), self._images[name][1].copy())
            for name in camera_ids
        }

    def cancel(self) -> None:
        self._cancelled.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None
        if self._process is not None:
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1.0)
            self._process = None
        self._images.clear()
        if self._memory is not None:
            self._memory.close()
            self._memory = None
