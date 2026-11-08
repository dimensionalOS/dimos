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

"""LivoxSim: the sim twin of the Mid360 module (its lidar and built-in IMU), read from its own view of the world.

The lidar casts the fitted Mid-360 scan pattern and return model; the IMU is the one inside the unit, at the same mount.
"""

import asyncio
from collections.abc import AsyncIterator
from typing import Any

from dimos_lcm.world_msgs import WorldObservations
import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.spec import SimulatedSensor
from dimos.sim2.view import WorldView, open_view, rotation_matrix, sensor_frame
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import OcclusionMap, SimLidar
from dimos.simulation.sensors.mid360.lidar import sim_mid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.utils.data import get_data


class LivoxSimConfig(ModuleConfig):
    # The lidar's frame, as a robot's URDF or the scene names it: where it sits.
    frame_id: str | None = "mid360_link"
    imu_frame_id: str = "imu_link"
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    # A lidar sees whatever a body would walk into, so these are the groups MuJoCo's own
    # convention puts collision geometry in: 0, where a hand-written scene's planes and boxes
    # go, and 3, where a converted one's collision meshes and every robot's do. Group 1 is left
    # out because it holds visual-only meshes (contype 0) -- drawing a desk is not the same as
    # being able to hit it. Seeing only group 0 was the old default: in office_555 that is 354
    # geoms out of 4138, so the beam passed straight through every desk, chair and robot and
    # returned the wall behind them.
    geom_groups: tuple[int, ...] = (0, 3)
    # The Go2 rig's mount blocks some directions; only meaningful when mounted on a Go2.
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(Module):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    world_observations: In[WorldObservations]
    _simulation: SimulationSpec

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedSensor("lidar", self.frame_id)}

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        description = self._simulation.describe()
        site = sensor_frame(description, self.frame_id)
        view = open_view(description)
        sensor = sim_mid360(
            view.raycaster(self.config.geom_groups),
            self.config.seed,
            OcclusionMap.load(get_data("go2_sim") / "go2_mid360_occlusion.npz")
            if self.config.go2_occlusion
            else None,
        )
        self._scanning = asyncio.create_task(self._scan(view, sensor, site))
        self._reading = asyncio.create_task(self._read_imu(open_view(description), site))
        yield

    async def _scan(self, view: WorldView, sensor: SimLidar, site: str) -> None:
        points_per_scan = int(POINT_RATE / self.config.rate_hz)
        # Scans and IMU readings pace on sim time, so both keep their rates per sim second at any speed.
        async for _ in self._world.ticks(self.config.rate_hz):
            view.update(self._world)
            position, orientation = view.pose(site)
            # Off the loop, so the IMU beside it keeps its rate while a scan is cast.
            points = await asyncio.to_thread(
                sensor.cast, position, rotation_matrix(orientation), points_per_scan
            )
            # The Mid360's own layout; each scan is cast from one pose, so every point is at offset 0.
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points,
                    self.frame_id,
                    self._world.stamp(),
                    offset_times=np.zeros(len(points), np.uint32),
                )
            )

    async def _read_imu(self, view: WorldView, site: str) -> None:
        async for _ in self._world.ticks(self.config.imu_rate_hz):
            view.update(self._world)
            gyro, accelerometer, (w, x, y, z) = view.imu(site)
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.config.imu_frame_id,
                    ts=self._world.stamp(),
                )
            )
