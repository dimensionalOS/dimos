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

"""The Go2's head L1 as GO2Connection's `lidar` sees it: the robot's rolling occupancy map, not a scan.

Measured on real recordings: 5 cm world-anchored cells, the whole map resent every 0.13 s,
cropped to a world-axis-aligned 128x128-cell square around the robot. Cells persist while
inside the square; the real map's flicker comes from sensor noise this sim does not have.
"""

import asyncio
from collections.abc import AsyncIterator
import math
from typing import Any

from dimos_lcm.world_msgs import WorldObservations
import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, rotation_matrix, sensor_frame
from dimos.sim2.sensor_usd import frame_usd
from dimos.sim2.spec import SimulatedRobot, SimulatedSensor
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import Raycaster

# Cell keys pack three offset 21-bit indices into one int64.
_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


class SimL1Config(ModuleConfig):
    # The L1's frame, as the robot's URDF names it: where it sits.
    frame_id: str | None = "utlidar_lidar"
    # The rolling map is published in this frame.
    map_frame_id: str = "world"
    period: float = 0.13
    points_per_second: float = 21_600.0
    max_range: float = 30.0
    cell: float = 0.05
    half_width_cells: int = 64
    z_range: tuple[float, float] = (-0.6, 1.3)
    geom_groups: tuple[int, ...] = (0,)
    seed: int = 0


class SimL1(Module):
    config: SimL1Config
    lidar: Out[PointCloud2]
    world_observations: In[WorldObservations]
    _simulation: SimulationSpec

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedSensor("lidar", self.frame_id)}

    @rpc
    def get_usd(self) -> list[str]:
        """``[asset_root, usd]``: the frame this L1 scans from.

        A frame rather than a case: the Go2's utlidar sits under a cover that is part of the
        robot's own shell, and nobody has measured the unit inside it.
        """
        return frame_usd(self.frame_id, name="sim_l1")

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        description = self._simulation.describe()
        site = sensor_frame(description, self.frame_id)
        # The map is centred on the robot the L1 is on.
        robot_id = site.split("/")[0]
        root = next(
            d.root_body
            for d in description["simulated"]
            if isinstance(d, SimulatedRobot) and d.robot_id == robot_id
        )
        view = open_view(description)
        self._mapping = asyncio.create_task(
            self._map(view, view.raycaster(self.config.geom_groups), site, f"{robot_id}/{root}")
        )
        yield

    async def _map(
        self, view: WorldView, raycaster: Raycaster, site: str, center_body: str
    ) -> None:
        config = self.config
        rng = np.random.default_rng(config.seed)
        rays = int(config.points_per_second * config.period)
        cells = np.zeros(0, dtype=np.int64)
        # Every period of sim time, so the map keeps its rate per sim second at any speed.
        async for _ in self._world.ticks(1 / config.period):
            view.update(self._world)
            origin, orientation = view.pose(site)
            directions = _hemisphere(rng, rays) @ rotation_matrix(orientation).T
            distance, _ = raycaster.cast(origin, directions, config.max_range)
            hit = distance > 0
            cells = np.union1d(cells, self._keys(origin + directions[hit] * distance[hit, None]))
            cells = self._crop(cells, view.pose(center_body)[0])
            self.lidar.publish(
                PointCloud2.from_numpy(
                    (self._indices(cells) + 0.5) * config.cell,
                    config.map_frame_id,
                    self._world.stamp(),
                )
            )

    def _keys(self, points: NDArray[np.float64]) -> NDArray[np.int64]:
        index = np.floor(points / self.config.cell).astype(np.int64) + _KEY_OFFSET
        return np.unique((index[:, 0] << 2 * _KEY_BITS) | (index[:, 1] << _KEY_BITS) | index[:, 2])

    def _indices(self, keys: NDArray[np.int64]) -> NDArray[np.int64]:
        mask = (1 << _KEY_BITS) - 1
        return (
            np.stack([keys >> 2 * _KEY_BITS, (keys >> _KEY_BITS) & mask, keys & mask], axis=1)
            - _KEY_OFFSET
        )

    def _crop(self, keys: NDArray[np.int64], center: NDArray[np.float64]) -> NDArray[np.int64]:
        config = self.config
        index = self._indices(keys)
        low = np.floor(center[:2] / config.cell).astype(np.int64) - config.half_width_cells
        inside = np.all(
            (index[:, :2] >= low) & (index[:, :2] < low + 2 * config.half_width_cells), axis=1
        )
        z = (index[:, 2] + 0.5) * config.cell
        inside &= (z >= config.z_range[0]) & (z <= config.z_range[1])
        return keys[inside]


def _hemisphere(rng: np.random.Generator, count: int) -> NDArray[np.float64]:
    """Uniform directions over the sensor's +z hemisphere: 360 degrees around, 90 up."""
    z = rng.uniform(0.0, 1.0, count)
    azimuth = rng.uniform(0.0, 2 * math.pi, count)
    ring = np.sqrt(1 - z**2)
    return np.stack([ring * np.cos(azimuth), ring * np.sin(azimuth), z], axis=1)
