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

"""One rendering worker serves camera devices through their existing output transports."""

from collections.abc import Callable
import time
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Transport
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.assets.model import UsdModel
from dimos.sim2.publication import PublicationRate
from dimos.sim2.sensors.camera.config import CameraConfig, CameraRenderer
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.poses import PoseSensorConfig, _PoseSensorModule
from dimos.sim2.sensors.sampling import Acquisition


class CameraRenderConfig(PoseSensorConfig):
    renderer: Callable[[], CameraRenderer]
    cameras: tuple[str, ...]


class CameraRenderModule(_PoseSensorModule):
    """Receive WorldState once, render due cameras together, publish without an image relay."""

    config: CameraRenderConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cameras: dict[str, CameraConfig] = {}
        self._sampling: dict[str, Acquisition] = {}
        self._counts: dict[str, int] = {}
        self._publication_rates: dict[str, PublicationRate] = {}
        self._outputs: dict[
            str, tuple[Transport[Image], Transport[Image], Transport[CameraInfo]]
        ] = {}

    @rpc
    def build(self) -> None:
        """Read named devices and their existing wired output transports once."""
        super().build()
        if not self.config.cameras or len(set(self.config.cameras)) != len(self.config.cameras):
            raise ValueError("renderer requires a nonempty list of unique camera module names")
        name = self.config.rpc_name or self.config.instance_name or type(self).__name__
        for device_name in self.config.cameras:
            device = RPCClient.remote(SimCameraModule, device_name, rpc=self.rpc)
            try:
                camera: CameraConfig = device.camera_config(name)
                outputs = {port.name: port.transport for port in device.get_stream_descriptors()}
            finally:
                device.stop_rpc_client()
            if camera.frame_id in self._cameras:
                raise ValueError(f"duplicate camera frame: {camera.frame_id}")
            self._cameras[camera.frame_id] = camera
            self._sampling[camera.frame_id] = Acquisition(camera.hz)
            self._counts[camera.frame_id] = 0
            self._publication_rates[camera.frame_id] = PublicationRate(
                self, f"{device_name}: color_image, depth_image, camera_info", camera.hz
            )
            transports = (
                outputs["color_image"].build(),
                outputs["depth_image"].build(),
                outputs["camera_info"].build(),
            )
            self._outputs[camera.frame_id] = transports
            for transport in transports:
                self.register_disposable(Disposable(transport.stop))

    def open_sensor(self, scene: UsdModel) -> None:
        self.renderer = self.config.renderer()
        try:
            self.renderer.load(scene, tuple(self._cameras.values()))
        except Exception:
            self.renderer.close()
            raise
        with self._stats_lock:
            self._stats["cameras"] = len(self._cameras)
            self._stats["scene_loads"] = self._stats.get("scene_loads", 0) + 1

    def acquire(self, sample: WorldState, *, latest: bool = True) -> None:
        for rate in self._publication_rates.values():
            rate.begin()
        if not latest:
            return
        due = tuple(
            name for name, schedule in self._sampling.items() if schedule.due(sample.header)
        )
        if not due:
            return
        started = time.monotonic()
        images = self.renderer.render(sample, due)
        if time.monotonic() - started > min(1 / self._cameras[name].hz for name in due):
            self.count("overruns")
        if set(images) != set(due):
            raise ValueError("renderer must return exactly the requested cameras")
        stamp = sample.header.time_ns / 1e9
        for name in due:
            c = self._cameras[name]
            rgb, depth = images[name]
            if (
                rgb.shape != (c.height, c.width, 3)
                or rgb.dtype != np.uint8
                or depth.shape != (c.height, c.width)
                or depth.dtype != np.float32
            ):
                raise ValueError(f"renderer returned invalid image dimensions or dtypes for {name}")
            color_image, depth_image, camera_info = self._outputs[name]
            color_image.publish(Image.from_numpy(rgb, ImageFormat.RGB, c.frame_id, stamp))
            depth_image.publish(Image.from_numpy(depth, ImageFormat.DEPTH, c.frame_id, stamp))
            camera_info.publish(
                CameraInfo(
                    width=c.width,
                    height=c.height,
                    K=[c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1],
                    frame_id=c.frame_id,
                    ts=stamp,
                )
            )
            self._publication_rates[name].published()
            with self._stats_lock:
                self._counts[name] += 1
                self._stats["acquisitions"] += 1

    @rpc
    def camera_statistics(self, frame_id: str) -> dict[str, Any]:
        with self._stats_lock:
            return {**self._stats, "acquisitions": self._counts[frame_id]}

    def close_sensor(self) -> None:
        self.renderer.close()
