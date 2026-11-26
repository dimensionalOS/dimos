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

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import UsdModel
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids, rotation_matrix
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor import SimSensor, SimSensorConfig
from dimos.sim2.sim import until_it_stops
from dimos.simulation.sensors.lidar import OcclusionMap, SimLidar
from dimos.simulation.sensors.mid360.lidar import sim_mid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.utils.data import get_data


class LivoxSimConfig(SimSensorConfig):
    usd: UsdModel = UsdModel.from_file(Path(__file__).parent / "assets" / "mid360.usda")
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    # stamp_tolerance for its IMU; None is one IMU sample period.
    imu_stamp_tolerance: float | None = None
    geom_groups: tuple[int, ...] = (0, 3)
    blind_to: tuple[str, ...] = ()
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(SimSensor):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    # The one view the lidar and IMU loops read, opened by whichever ticks first.
    _opening: "asyncio.Future[WorldView] | None" = None

    @property
    def imu_frame_id(self) -> str:
        return self.tf_frames[f"{self.config.usd.root_link}/imu_link"]

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

    async def _view(self) -> tuple[WorldView, bool]:
        """The shared view, and whether this tick was spent waiting for it to open."""
        if self._opening is None:
            self._opening = asyncio.ensure_future(asyncio.to_thread(self._open))
        waited = not self._opening.done()
        # Shielded, so one loop stopping mid-open leaves the other its view.
        return await asyncio.shield(self._opening), waited

    async def _scan(self) -> None:
        site = self.frame_id
        view: WorldView | None = None
        sensor: SimLidar | None = None
        points_per_scan = int(POINT_RATE / self.config.rate_hz)
        async for _ in self._world.ticks(self.config.rate_hz):
            if view is None:
                view, waited = await self._view()
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
                    # Opening took this tick, so the next tick is the first read on time.
                    continue
            assert sensor is not None
            view.update(self._world)
            stamp = view.time
            position, orientation = view.pose(site)
            points = await asyncio.to_thread(
                sensor.cast, position, rotation_matrix(orientation), points_per_scan
            )
            period, tolerance = 1 / self.config.rate_hz, self.config.stamp_tolerance
            if not self.on_time("lidar", stamp, period, tolerance):
                continue
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points,
                    self.frame_id,
                    stamp,
                    offset_times=np.zeros(len(points), np.uint32),
                )
            )
            self.publish_moving_tf(stamp)

    async def _read_imu(self) -> None:
        site, view = self.imu_frame_id, None
        async for _ in self._world.ticks(self.config.imu_rate_hz):
            if view is None:
                shared, waited = await self._view()
                # Its own poses, as the lidar casts against the shared view's in a thread while this updates.
                view = shared.twin()
                if waited:
                    # Opening took this tick, so the next tick is the first read on time.
                    continue
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            period, tolerance = 1 / self.config.imu_rate_hz, self.config.imu_stamp_tolerance
            if not self.on_time("imu", view.time, period, tolerance):
                continue
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=site,
                    ts=view.time,
                )
            )
