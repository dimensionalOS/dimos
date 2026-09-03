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

"""An evolving occupancy-grid stream derived from recorded lidar."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar

import numpy as np

from dimos.evals.types import Agent, RunningEnvironment

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def _align_grids(grids: list[OccupancyGrid]) -> list[OccupancyGrid]:
    """Place evolving grids on one fixed world raster for temporal comparison."""
    if len(grids) < 2:
        return grids

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    resolution = grids[0].resolution
    if any(not math.isclose(grid.resolution, resolution) for grid in grids[1:]):
        raise ValueError("occupancy-grid sequence has inconsistent resolutions")
    min_x = min(grid.origin.position.x for grid in grids)
    min_y = min(grid.origin.position.y for grid in grids)
    max_x = max(grid.origin.position.x + grid.width * resolution for grid in grids)
    max_y = max(grid.origin.position.y + grid.height * resolution for grid in grids)
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)

    aligned: list[OccupancyGrid] = []
    for grid in grids:
        cells = np.full((height, width), -1, dtype=np.int8)
        x = round((grid.origin.position.x - min_x) / resolution)
        y = round((grid.origin.position.y - min_y) / resolution)
        cells[y : y + grid.height, x : x + grid.width] = grid.grid
        aligned.append(
            OccupancyGrid(
                grid=cells,
                resolution=resolution,
                origin=Pose(min_x, min_y, 0.0),
                frame_id=grid.frame_id,
                ts=grid.ts,
            )
        )
    return aligned


@dataclass
class OccupancyDataset:
    """Rebuild cumulative costmaps from a frozen recording's lidar stream.

    Each output grid accumulates every lidar frame seen so far, matching the
    live ``VoxelGridMapper -> CostMapper`` data flow. Snapshots are aligned to
    one world raster so temporal changes are spatially comparable. Only
    ``global_costmap`` is exposed to the agent; source streams are not.
    """

    name: str
    start_s: float | None = None
    stop_s: float | None = None
    emit_every: int = 8
    voxel_size: float = 0.05
    resolution: float = 0.1
    occupancy_algo: str = "height_cost"
    device: str = "CPU:0"

    artifacts: ClassVar[tuple[str, ...]] = ("recording",)
    has_robot: ClassVar[bool] = False
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)

    def preflight(self, agent: Agent) -> None:
        if agent.modules:
            raise RuntimeError(
                f"OccupancyDataset({self.name!r}) launches nothing; "
                f"{type(agent).__name__} adds modules {agent.modules!r}"
            )
        if self.emit_every < 0:
            raise ValueError("emit_every must be non-negative")

        from dimos.mapping.pointclouds.occupancy import OCCUPANCY_ALGOS
        from dimos.memory.cli.dataset import open_dataset

        if self.occupancy_algo not in OCCUPANCY_ALGOS:
            raise ValueError(
                f"unknown occupancy algorithm {self.occupancy_algo!r}; "
                f"expected one of {sorted(OCCUPANCY_ALGOS)}"
            )
        source = open_dataset(self.name)
        try:
            source.streams.lidar.range_time(self.start_s, self.stop_s)
        finally:
            source.stop()

    def start(self, modules: str, trace_dir: Path | None = None) -> RunningEnvironment:
        from dimos.mapping.pointclouds.occupancy import OCCUPANCY_ALGOS
        from dimos.mapping.voxels.module import VoxelMapTransformer
        from dimos.memory.cli.dataset import open_dataset, resolve_dataset
        from dimos.memory.store.memory import MemoryStore
        from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

        source = open_dataset(self.name)
        recording = MemoryStore()
        target = recording.stream("global_costmap", OccupancyGrid)
        try:
            lidar = source.streams.lidar.range_time(self.start_s, self.stop_s)
            maps = lidar.transform(
                VoxelMapTransformer(
                    emit_every=self.emit_every,
                    voxel_size=self.voxel_size,
                    device=self.device,
                )
            )
            occupancy = OCCUPANCY_ALGOS[self.occupancy_algo]
            generated: list[tuple[OccupancyGrid, float, dict[str, Any]]] = []
            for obs in maps:
                grid = occupancy(obs.data, resolution=self.resolution)
                generated.append((grid, obs.ts, obs.tags))
            aligned = _align_grids([grid for grid, _, _ in generated])
            for grid, (_, ts, tags) in zip(aligned, generated, strict=True):
                target.append(grid, ts=ts, tags=tags)
        except Exception:
            recording.stop()
            raise
        finally:
            source.stop()

        self._recording = recording
        return RunningEnvironment(
            mcp_url="",
            recording=recording,
            artifacts={"recording": resolve_dataset(self.name)},
        )

    def settle(self, budget_s: float) -> None:
        return None

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
