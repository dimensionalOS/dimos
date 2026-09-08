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

"""The unit under evaluation: lidar and odometry in, paths out."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from collections.abc import Callable

    import numpy as np
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig

Point = tuple[float, float, float]


class NavPipeline(Protocol):
    """Everything the evaluator requires of a navigation stack."""

    def add_frame(self, points: NDArray[np.float32], origin: Point, ts: float) -> None:
        """Take one world-frame lidar cloud and the sensor origin it was shot from."""
        ...

    def sync_map(self) -> None:
        """Fold everything ingested since the last plan into the planner's map.

        Called outside the plan timer, so map work is never scored as search.
        """
        ...

    def plan(self, start: Point, goal: Point) -> NDArray[np.float32] | None:
        """Foot-level waypoints from start to goal, or None when there is no route."""
        ...


@runtime_checkable
class PipelineIntrospection(Protocol):
    """Optional map and graph layers, for the gates and the rerun recording."""

    def surface_clearance_map(self) -> NDArray[np.float32]: ...

    def node_edges(self) -> NDArray[np.float32]: ...

    def occupied(self) -> NDArray[np.float32]: ...


class MLSPipeline:
    """Voxel ray-tracing mapper feeding the MLS planner."""

    def __init__(self, cfg: EvalConfig) -> None:
        # Lazy: the planner is a native module, only needed by this pipeline.
        from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
        from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

        self._mapper = VoxelRayMapper(voxel_size=cfg.voxel_size, max_range=cfg.max_range)
        overrides = dict(cfg.planner)
        # The planner's one int param, bound explicitly so the unpack type-checks.
        threads = int(overrides.pop("worker_threads", 4))
        self._planner = MLSPlanner(
            voxel_size=cfg.voxel_size,
            robot_height=cfg.robot_height,
            worker_threads=threads,
            **overrides,
        )
        self._pending = False
        self._mapped = False

    def add_frame(self, points: NDArray[np.float32], origin: Point, ts: float) -> None:
        self._mapper.add_frame_world(points, origin)
        self._pending = True

    def sync_map(self) -> None:
        if not self._pending:
            return
        occupied = self._mapper.global_map()
        if len(occupied):
            self._planner.update_global_map(occupied)
            self._mapped = True
        self._pending = False

    def plan(self, start: Point, goal: Point) -> NDArray[np.float32] | None:
        self.sync_map()
        if not self._mapped:
            return None
        return self._planner.plan(start, goal)

    def surface_clearance_map(self) -> NDArray[np.float32]:
        return self._planner.surface_clearance_map()

    def node_edges(self) -> NDArray[np.float32]:
        return self._planner.node_edges()

    def occupied(self) -> NDArray[np.float32]:
        return self._mapper.global_map()


PIPELINES: dict[str, Callable[[EvalConfig], NavPipeline]] = {"mls": MLSPipeline}


def make_pipeline(name: str, cfg: EvalConfig) -> NavPipeline:
    if name not in PIPELINES:
        raise ValueError(f"unknown pipeline {name!r}; known: {sorted(PIPELINES)}")
    return PIPELINES[name](cfg)
