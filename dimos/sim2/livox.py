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


import asyncio
from collections.abc import AsyncIterator
from pathlib import Path

from dimos_lcm.world_msgs import WorldObservations
import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.livox import module as mid360_driver
from dimos.hardware.sensors.lidar.livox.module import mid360_usd
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids, rotation_matrix
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import OcclusionMap, SimLidar
from dimos.simulation.sensors.mid360.lidar import sim_mid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.utils.data import get_data


class LivoxSimConfig(SceneConfig):
    frame_id: str = "mid360_link"
    imu_frame_id: str = "imu_link"
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    geom_groups: tuple[int, ...] = (0, 3)
    blind_to: tuple[str, ...] = ()
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(Module):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    world_observations: In[WorldObservations]

    @property
    def imu_frame_id(self) -> str:
        prefix = self.config.frame_id_prefix
        return f"{prefix}/{self.config.imu_frame_id}" if prefix else self.config.imu_frame_id

    @rpc
    def get_usd(self) -> list[str]:
        return [
            str(Path(mid360_driver.__file__ or "").parent),
            mid360_usd(self.frame_id, self.imu_frame_id),
        ]

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._scanning = asyncio.create_task(self._scan())
        self._reading = asyncio.create_task(self._read_imu())
        yield
        self._scanning.cancel()
        self._reading.cancel()

    def _layer(self) -> Path:
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to"
                f" range against. Give this lidar the scene_manager that composes the world."
            )
        return usd_layer(scene)

    def _open(self, layer: Path) -> WorldView:
        return open_view(
            layer,
            robot_ids(self._world),
            SensorMount("lidar", self.frame_id, self.frame_id),
        )

    async def _scan(self) -> None:
        layer, site = None, self.frame_id
        view: WorldView | None = None
        sensor: SimLidar | None = None
        points_per_scan = int(POINT_RATE / self.config.rate_hz)
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                layer = layer or self._layer()
                view = self._open(layer)
                sensor = sim_mid360(
                    view.raycaster(
                        self.config.geom_groups,
                        mounted_at=site,
                        blind_to=self.config.blind_to,
                    ),
                    self.config.seed,
                    OcclusionMap.load(get_data("go2_sim") / "go2_mid360_occlusion.npz")
                    if self.config.go2_occlusion
                    else None,
                )
            assert sensor is not None
            view.update(self._world)
            position, orientation = view.pose(site)
            points = await asyncio.to_thread(
                sensor.cast, position, rotation_matrix(orientation), points_per_scan
            )
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points,
                    self.frame_id,
                    self._world.stamp(),
                    offset_times=np.zeros(len(points), np.uint32),
                )
            )

    async def _read_imu(self) -> None:
        site, view = self.frame_id, None
        async for _ in self._world.ticks(self.config.imu_rate_hz):
            if view is None:
                view = self._open(self._layer())
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.imu_frame_id,
                    ts=self._world.stamp(),
                )
            )
