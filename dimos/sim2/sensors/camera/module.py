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

"""Camera acquisition, independent of physics and selectable renderer."""

from collections.abc import Callable

from dimos_lcm.sim_msgs import WorldState

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.sensors.camera.config import CameraConfig, CameraRenderer
from dimos.sim2.sensors.poses import PoseSensorConfig, _PoseSensorModule
from dimos.sim2.sensors.sampling import Acquisition, measurement_header


class CameraModuleConfig(PoseSensorConfig):
    sensor: CameraConfig
    renderer: Callable[[], CameraRenderer]


class SimCameraModule(_PoseSensorModule):
    config: CameraModuleConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]

    def open_sensor(self, scene: SceneConfig) -> None:
        self.renderer = self.config.renderer()
        self.renderer.load(scene, self.config.sensor)
        self._sampling = Acquisition(self.config.sensor.hz)

    def acquire(self, sample: WorldState, *, latest: bool = True) -> None:
        if not latest or not self._sampling.due(sample.header):
            return
        c = self.config.sensor
        pose = poses_by_id(sample)[c.frame_id]
        self.renderer.set_poses(sample)
        rgb, depth = self.renderer.render(pose)
        header = measurement_header(sample.header, c.source_id, c.frame_id)
        self.color_image.publish(
            Image.from_numpy(
                rgb, ImageFormat.RGB, c.frame_id, header.time_ns / 1e9
            ).with_sample_header(header)
        )
        self.depth_image.publish(
            Image.from_numpy(
                depth, ImageFormat.DEPTH, c.frame_id, header.time_ns / 1e9
            ).with_sample_header(header)
        )
        info = CameraInfo(
            width=c.width,
            height=c.height,
            K=[c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1],
            frame_id=c.frame_id,
            ts=header.time_ns / 1e9,
        )
        self.camera_info.publish(info.with_sample_header(header))
        self.count("acquisitions")

    def close_sensor(self) -> None:
        self.renderer.close()
