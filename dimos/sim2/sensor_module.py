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

"""Acquisition lives in its own worker; rendering cannot take the physics lock."""

from queue import Empty, Full, Queue
import threading
from typing import Any, ClassVar

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.module import SimulationSpec
from dimos.sim2.types import Camera, Frame, Lidar, SensorFactory, SensorReader, World


class SensorConfig(ModuleConfig):
    world: World
    reader: SensorFactory
    prefix: str = "left"


class SimSensorModule(Module):
    config: SensorConfig
    dedicated_worker: ClassVar[bool] = True
    _sim: SimulationSpec
    world_frame: In[Frame]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    lidar: Out[PointCloud2]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._queue: Queue[Frame] = Queue(64)
        self._done = threading.Event()
        self._thread: threading.Thread | None = None
        self._error = ""
        self._dropped = 0
        self._scans = 0
        self._images = 0
        self._gaps = 0

    @rpc
    def start(self) -> None:
        PointCloud2.lcm_warmup()
        super().start()
        self._layout = self._sim.describe()
        self._thread = threading.Thread(target=self._acquire, name="sim2-sensors", daemon=True)
        self._thread.start()

    async def handle_world_frame(self, frame: Frame) -> None:
        try:
            self._queue.put_nowait(frame)
        except Full:
            self._dropped += 1
            # Keep newest input; the history continuity check will reject missing scan intervals.
            try:
                self._queue.get_nowait()
            except Empty:
                pass
            self._queue.put_nowait(frame)

    def _acquire(self) -> None:
        reader: SensorReader | None = None
        try:
            reader = self.config.reader(self.config.world, self._layout)
            camera, lidar, imu = (
                self.config.prefix + "/" + kind for kind in ("camera", "lidar", "imu")
            )
            camera_profile = next(
                s for s in self.config.world.sensors if isinstance(s, Camera) and s.frame == camera
            )
            lidar_profile = next(
                s for s in self.config.world.sensors if isinstance(s, Lidar) and s.frame == lidar
            )
            if (
                not np.isfinite([camera_profile.hz, lidar_profile.hz]).all()
                or min(camera_profile.hz, lidar_profile.hz) <= 0
            ):
                raise ValueError("sensor rates must be finite and positive")
            info = CameraInfo.from_fov(
                camera_profile.fovy, camera_profile.width, camera_profile.height, frame_id=camera
            )
            imu_index = self._layout.imu_frames.index(imu)
            epoch, next_camera, next_lidar = "", 0.0, 0.0
            while not self._done.is_set():
                try:
                    frame = self._queue.get(timeout=0.1)
                except Empty:
                    continue
                state = frame.state
                reader.observe(state)
                if state.epoch != epoch:
                    epoch, next_camera, next_lidar = state.epoch, 0.0, 0.0
                sample = frame.imu[imu_index]
                self.imu.publish(
                    Imu(
                        ts=state.wall_time,
                        frame_id=imu,
                        linear_acceleration=Vector3(*sample[:3]),
                        angular_velocity=Vector3(*sample[3:]),
                        orientation_covariance=[-1.0] + [0.0] * 8,
                    )
                )
                # Drain history promptly before optional rendering; camera is latest-frame work.
                if not self._queue.empty():
                    continue
                if state.sim_time >= next_camera:
                    rgb, depth = reader.camera(camera, state)
                    self.color_image.publish(
                        Image.from_numpy(rgb, ImageFormat.RGB, camera, state.wall_time)
                    )
                    self.depth_image.publish(
                        Image.from_numpy(depth, ImageFormat.DEPTH, camera, state.wall_time)
                    )
                    info.ts = state.wall_time
                    self.camera_info.publish(info)
                    self._images += 1
                    next_camera = state.sim_time + 1 / camera_profile.hz
                if state.sim_time >= next_lidar:
                    scan = reader.lidar(lidar, duration=1 / lidar_profile.hz)
                    self._gaps = reader.gaps
                    if scan is not None and len(scan.points):
                        stamp = state.wall_time + scan.times[0] - state.sim_time
                        offsets = np.rint((scan.times - scan.times[0]) * 1e9).astype(np.uint32)
                        self.lidar.publish(
                            PointCloud2.from_numpy(
                                scan.points, frame_id="world", timestamp=stamp, offset_times=offsets
                            )
                        )
                        self._scans += 1
                    next_lidar = state.sim_time + 1 / lidar_profile.hz
        except Exception as error:
            self._error = repr(error)
        finally:
            if reader is not None:
                reader.close()

    @rpc
    def statistics(self) -> dict[str, Any]:
        return {
            "images": self._images,
            "scans": self._scans,
            "gaps": self._gaps,
            "dropped_samples": self._dropped,
            "error": self._error,
        }

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        super().stop()
