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

"""Three RGB-D views sharing one renderer scene and a bounded capture worker."""

from collections.abc import Callable
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.galaxea.r1pro.sim.cameras import WRIST_MAX_DEPTH_M, WRIST_MIN_DEPTH_M
from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.sensors.camera.config import CameraConfig, CameraRigRenderer
from dimos.sim2.sensors.poses import PoseSensorConfig, _PoseSensorModule
from dimos.sim2.sensors.sampling import Acquisition


class R1ProCamerasConfig(PoseSensorConfig):
    head: CameraConfig
    left: CameraConfig
    right: CameraConfig
    renderer: Callable[[], CameraRigRenderer]


class R1ProSimCameras(_PoseSensorModule):
    config: R1ProCamerasConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    left_wrist: Out[Image]
    left_wrist_depth: Out[Image]
    left_wrist_info: Out[CameraInfo]
    right_wrist: Out[Image]
    right_wrist_depth: Out[Image]
    right_wrist_info: Out[CameraInfo]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.renderer = self.config.renderer()
        self._cameras = (self.config.head, self.config.left, self.config.right)
        self._slots = ("workspace", "left", "right")
        self._next_camera = 0
        self._stats.update({slot: 0 for slot in self._slots})

    def open_sensor(self, scene: SceneConfig) -> None:
        self.renderer.load(scene, self.config.head)
        self._sampling = [Acquisition(c.hz) for c in self._cameras]

    def acquire(self, sample: WorldState, *, latest: bool = True) -> None:
        if not latest:
            return
        # At most one capture per fresh observation. Round-robin fairness and
        # source-clock rate caps; no queue of obsolete arm/camera poses.
        for offset in range(len(self._cameras)):
            index = (self._next_camera + offset) % len(self._cameras)
            if self._sampling[index].due(sample.header):
                break
        else:
            return
        self._next_camera = (index + 1) % len(self._cameras)
        c = self._cameras[index]
        self.renderer.set_poses(sample)
        rgb, depth = self.renderer.render_camera(c, poses_by_id(sample)[c.frame_id])
        if index:
            depth = np.where(
                np.isfinite(depth) & (depth >= WRIST_MIN_DEPTH_M) & (depth <= WRIST_MAX_DEPTH_M),
                depth,
                0,
            ).astype(np.float32)
        stamp = sample.header.time_ns / 1e9
        ports = (
            (self.color_image, self.depth_image, self.camera_info),
            (self.left_wrist, self.left_wrist_depth, self.left_wrist_info),
            (self.right_wrist, self.right_wrist_depth, self.right_wrist_info),
        )[index]
        # Calibration arrives before its first synchronized pair.
        ports[2].publish(
            CameraInfo(
                width=c.width,
                height=c.height,
                K=[c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1],
                frame_id=c.frame_id,
                ts=stamp,
            )
        )
        ports[0].publish(Image.from_numpy(rgb, ImageFormat.RGB, c.frame_id, stamp))
        ports[1].publish(Image.from_numpy(depth, ImageFormat.DEPTH, c.frame_id, stamp))
        self.count("acquisitions")
        self.count(self._slots[index])

    def close_sensor(self) -> None:
        self.renderer.close()

    def interrupt_sensor(self) -> None:
        self.renderer.cancel()
