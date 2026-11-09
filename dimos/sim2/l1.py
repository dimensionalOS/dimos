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
import math

from dimos_lcm.world_msgs import WorldObservations
import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids, rotation_matrix
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor_usd import frame_usd
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import Raycaster

_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


class SimL1Config(SceneConfig):
    frame_id: str = "utlidar_lidar"
    map_frame_id: str = "world"
    period: float = 0.13
    points_per_second: float = 21_600.0
    max_range: float = 30.0
    cell: float = 0.05
    half_width_cells: int = 64
    z_range: tuple[float, float] = (-0.6, 1.3)
    geom_groups: tuple[int, ...] = (0, 3)
    seed: int = 0


class SimL1(Module):
    config: SimL1Config
    lidar: Out[PointCloud2]
    world_observations: In[WorldObservations]

    @rpc
    def get_usd(self) -> list[str]:
        return frame_usd(self.frame_id, name="sim_l1")

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._mapping = asyncio.create_task(self._map())
        yield
        self._mapping.cancel()

    def _open(self) -> WorldView:
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to map."
                f" Give this L1 the scene_manager that composes the world."
            )
        return open_view(
            usd_layer(scene),
            robot_ids(self._world),
            SensorMount("lidar", self.frame_id, self.frame_id),
        )

    async def _map(self) -> None:
        config, site = self.config, self.frame_id
        view: WorldView | None = None
        raycaster: Raycaster | None = None
        rider: str | None = None
        rng = np.random.default_rng(config.seed)
        rays = int(config.points_per_second * config.period)
        cells = np.zeros(0, dtype=np.int64)
        async for _ in self._world.ticks(1 / config.period):
            if view is None:
                view = self._open()
                raycaster = view.raycaster(config.geom_groups)
                rider = view.owner(site)
            assert raycaster is not None
            view.update(self._world)
            origin, orientation = view.pose(site)
            directions = _hemisphere(rng, rays) @ rotation_matrix(orientation).T
            distance, _ = raycaster.cast(origin, directions, config.max_range)
            hit = distance > 0
            cells = np.union1d(cells, self._keys(origin + directions[hit] * distance[hit, None]))
            cells = self._crop(cells, self._center(rider, origin))
            self.lidar.publish(
                PointCloud2.from_numpy(
                    (self._indices(cells) + 0.5) * config.cell,
                    config.map_frame_id,
                    self._world.stamp(),
                )
            )

    def _center(self, rider: str | None, origin: NDArray[np.float64]) -> NDArray[np.float64]:
        entity = self._world.entity(rider) if rider is not None else None
        return origin if entity is None or entity.pose is None else entity.pose[:3]

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
    z = rng.uniform(0.0, 1.0, count)
    azimuth = rng.uniform(0.0, 2 * math.pi, count)
    ring = np.sqrt(1 - z**2)
    return np.stack([ring * np.cos(azimuth), ring * np.sin(azimuth), z], axis=1)
