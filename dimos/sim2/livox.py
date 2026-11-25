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

import numpy as np

from dimos.core.hardware_module import HardwareModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import UsdModel
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids, rotation_matrix
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sim import until_it_stops
from dimos.sim2.world_observations import geometry
from dimos.simulation.sensors.lidar import OcclusionMap, SimLidar
from dimos.simulation.sensors.mid360.lidar import sim_mid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.utils.data import get_data


class LivoxSimConfig(HardwareModuleConfig):
    usd: UsdModel = UsdModel.from_file(Path(__file__).parent / "assets" / "mid360.usda")
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    geom_groups: tuple[int, ...] = (0, 3)
    blind_to: tuple[str, ...] = ()
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(SimSensor):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    # The one view the lidar and IMU loops read, of the world's shapes at a geometry() count, opened by whichever asks first.
    _opening: "tuple[int, asyncio.Future[WorldView]] | None" = None

    @property
    def imu_frame_id(self) -> str:
        return self.usd_frame("imu_link")

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        self._scanning = asyncio.create_task(until_it_stops(f"{self.frame_id} lidar", self._scan()))
        self._reading = asyncio.create_task(
            until_it_stops(f"{self.frame_id} lidar imu", self._read_imu())
        )
        yield
        self._scanning.cancel()
        self._reading.cancel()

    def _open(self) -> WorldView:
        return open_view(
            self.world_layer(),
            robot_ids(self._world),
            SensorMount("lidar", self.frame_id, self.frame_id),
        )

    async def _view(self, built: int) -> tuple[WorldView, bool]:
        """The shared view of the world at geometry count `built`, and whether this tick was spent waiting for it to open."""
        if self._opening is None or self._opening[0] != built:
            self._opening = (built, asyncio.ensure_future(asyncio.to_thread(self._open)))
        opening = self._opening[1]
        waited = not opening.done()
        # Shielded, so one loop stopping mid-open leaves the other its view.
        return await asyncio.shield(opening), waited

    async def _scan(self) -> None:
        site = self.frame_id
        view: WorldView | None = None
        sensor: SimLidar | None = None
        built = 0
        points_per_scan = int(POINT_RATE / self.config.rate_hz)
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is not None and geometry(self._world) != built:
                view.close()
                view = None
            if view is None:
                built = geometry(self._world)
                view, waited = await self._view(built)
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
                if waited:
                    # Opening took this tick's sim time, so the next tick is the first read on time.
                    continue
            assert sensor is not None
            view.update(self._world)
            stamp = view.time
            position, orientation = view.pose(site)
            points = await asyncio.to_thread(
                sensor.cast, position, rotation_matrix(orientation), points_per_scan
            )
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points,
                    self.frame_id,
                    stamp,
                    offset_times=np.zeros(len(points), np.uint32),
                )
            )

    async def _read_imu(self) -> None:
        site, view, built = self.imu_frame_id, None, 0
        async for _ in self._world.ticks(self.config.imu_rate_hz):
            if view is not None and geometry(self._world) != built:
                view.close()
                view = None
            if view is None:
                built = geometry(self._world)
                shared, waited = await self._view(built)
                # Its own poses, as the lidar casts against the shared view's in a thread while this updates.
                view = shared.twin()
                if waited:
                    # Opening took this tick's sim time, so the next tick is the first read on time.
                    continue
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=site,
                    ts=view.time,
                )
            )
