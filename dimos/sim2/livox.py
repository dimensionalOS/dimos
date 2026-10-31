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

from dimos_lcm.world_msgs import WorldChange
import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.spec import Mount, SimulatedSensor
from dimos.sim2.view import WorldView, open_view, rotation_matrix
from dimos.sim2.world_change import WorldMirror
from dimos.simulation.sensors.mid360.lidar import OcclusionMap, SimMid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE, Mid360Pattern
from dimos.utils.data import get_data


class LivoxSimConfig(ModuleConfig):
    name: str = "lidar"
    mount: Mount = Mount("world")
    robot_id: str | None = None
    frame_id: str | None = "mid360_link"
    imu_frame_id: str = "imu_link"
    rate_hz: float = 10.0
    imu_rate_hz: float = 200.0
    # Robots' meshes sit outside group 0, so the default sees only the scene, and stays cheap.
    geom_groups: tuple[int, ...] = (0,)
    # The Go2 rig's mount blocks some directions; only meaningful when mounted on a Go2.
    go2_occlusion: bool = False
    seed: int = 0


class LivoxSim(Module):
    config: LivoxSimConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    world_change: In[WorldChange]
    _simulation: SimulationSpec

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config = self.config
        return {"simulated": SimulatedSensor(config.name, "lidar", config.mount, config.robot_id)}

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_change)
        view = open_view(self._simulation.describe())
        root = get_data("go2_sim")
        sensor = SimMid360(
            view.raycaster(self.config.geom_groups),
            Mid360Pattern.load(root / "mid360_pattern.npz"),
            self.config.seed,
            OcclusionMap.load(root / "go2_mid360_occlusion.npz")
            if self.config.go2_occlusion
            else None,
        )
        self._scanning = asyncio.create_task(self._scan(view, sensor))
        self._reading = asyncio.create_task(self._read_imu(open_view(self._simulation.describe())))
        yield

    async def _scan(self, view: WorldView, sensor: SimMid360) -> None:
        site = f"{self.config.robot_id or 'world'}/{self.config.name}"
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

    async def _read_imu(self, view: WorldView) -> None:
        site = f"{self.config.robot_id or 'world'}/{self.config.name}"
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
