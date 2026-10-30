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

import math
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
from dimos.sim2.types import (
    Camera,
    Frame,
    Imu as ImuProfile,
    Layout,
    Lidar,
    SensorFactory,
    SensorReader,
    World,
)


class SensorConfig(ModuleConfig):
    world: World
    reader: SensorFactory
    camera: str | None = None
    lidar: str | None = None
    imu: str | None = None


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
        self._camera: Camera | None = None
        self._lidar: Lidar | None = None
        self._imu_index: int | None = None
        self._camera_info: CameraInfo | None = None
        self._epoch = ""
        self._next_camera = self._next_lidar = 0.0

    @rpc
    def start(self) -> None:
        PointCloud2.lcm_warmup()
        super().start()
        self._layout = self._sim.describe()
        self.bind(self._layout)
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

    def bind(self, layout: Layout) -> None:
        """Validate explicit sensor-to-output bindings before starting acquisition."""
        profiles = {s.frame: s for s in self.config.world.sensors}
        if not any((self.config.camera, self.config.lidar, self.config.imu)):
            raise ValueError("select at least one sensor frame")
        for name, kind in (
            (self.config.camera, Camera),
            (self.config.lidar, Lidar),
            (self.config.imu, ImuProfile),
        ):
            if name is not None and not isinstance(profiles.get(name), kind):
                raise ValueError(f"{name!r} is not a configured {kind.__name__} sensor")
        camera, lidar = (
            profiles.get(self.config.camera or ""),
            profiles.get(self.config.lidar or ""),
        )
        self._camera = camera if isinstance(camera, Camera) else None
        self._lidar = lidar if isinstance(lidar, Lidar) else None
        for profile in (self._camera, self._lidar):
            if profile is not None and (not math.isfinite(profile.hz) or profile.hz <= 0):
                raise ValueError("sensor rates must be finite and positive")
        if self._camera is not None:
            c = self._camera
            self._camera_info = CameraInfo.from_fov(c.fovy, c.width, c.height, frame_id=c.frame)
        self._imu_index = (
            layout.imu_frames.index(self.config.imu) if self.config.imu is not None else None
        )

    def _acquire(self) -> None:
        reader: SensorReader | None = None
        try:
            reader = self.config.reader(self.config.world, self._layout)
            while not self._done.is_set():
                try:
                    frame = self._queue.get(timeout=0.1)
                except Empty:
                    continue
                reader.observe(frame.state)
                self.publish_sample(frame, reader, render=self._queue.empty())
        except Exception as error:
            self._error = repr(error)
        finally:
            if reader is not None:
                reader.close()

    def publish_sample(self, frame: Frame, reader: SensorReader, *, render: bool = True) -> None:
        """Publish configured measurements; expensive acquisition uses the newest queued state."""
        state = frame.state
        if state.epoch != self._epoch:
            self._epoch, self._next_camera, self._next_lidar = state.epoch, 0.0, 0.0
        if self._imu_index is not None:
            assert self.config.imu is not None
            sample = frame.imu[self._imu_index]
            self.imu.publish(
                Imu(
                    ts=state.wall_time,
                    frame_id=self.config.imu,
                    linear_acceleration=Vector3(*sample[:3]),
                    angular_velocity=Vector3(*sample[3:]),
                    orientation_covariance=[-1.0] + [0.0] * 8,
                )
            )
        if not render:
            return
        camera = self._camera
        if camera is not None and state.sim_time >= self._next_camera:
            rgb, depth = reader.camera(camera.frame, state)
            self.color_image.publish(
                Image.from_numpy(rgb, ImageFormat.RGB, camera.frame, state.wall_time)
            )
            self.depth_image.publish(
                Image.from_numpy(depth, ImageFormat.DEPTH, camera.frame, state.wall_time)
            )
            assert self._camera_info is not None
            self._camera_info.ts = state.wall_time
            self.camera_info.publish(self._camera_info)
            self._images += 1
            self._next_camera = (math.floor(state.sim_time * camera.hz) + 1) / camera.hz
        lidar = self._lidar
        if lidar is not None and state.sim_time >= self._next_lidar:
            scan = reader.lidar(lidar.frame, duration=1 / lidar.hz)
            self._gaps = reader.gaps
            if scan is not None and len(scan.points):
                stamp = state.wall_time + scan.times[0] - state.sim_time
                offsets = np.rint((scan.times - scan.times[0]) * 1e9).astype(np.uint32)
                self.lidar.publish(
                    PointCloud2.from_numpy(
                        scan.points,
                        frame_id="world",
                        timestamp=stamp,
                        offset_times=offsets,
                    )
                )
                self._scans += 1
            self._next_lidar = (math.floor(state.sim_time * lidar.hz) + 1) / lidar.hz

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
