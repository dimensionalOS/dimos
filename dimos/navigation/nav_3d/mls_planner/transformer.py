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

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


class MLSPlan(Transformer[PointCloud2, Path]):
    """Plan paths over an incrementally accumulated voxel map.

    Consumes a stream of cumulative voxel-map point clouds (e.g. from
    :class:`RayTraceMap`). Each upstream observation must carry the current
    robot pose in ``obs.pose_tuple``; its xyz drives the planner ``start``
    (with z dropped by ``robot_height`` to put it at foot level). The fixed
    ``goal`` is supplied at construction. Yields a :class:`Path` per frame,
    empty when no route exists.

    Surface map, graph nodes, graph edges, and the current start are attached
    to ``obs.tags`` for visualization.

    Args:
        goal: world-frame goal position.
        voxel_size: planner voxelization edge length (m).
        robot_height: vertical clearance required above standable cells (m).
        surface_dilation_passes: morphological dilation passes on surface mask.
        surface_erosion_passes: morphological erosion passes on surface mask.
        node_spacing_m: target spacing between graph nodes (m).
        node_wall_buffer_m: minimum distance from a node to a wall voxel (m).
        node_step_threshold_m: max vertical step between connected nodes (m).
    """

    def __init__(
        self,
        *,
        goal: tuple[float, float, float],
        voxel_size: float = 0.1,
        robot_height: float = 1.5,
        surface_dilation_passes: int = 3,
        surface_erosion_passes: int = 3,
        node_spacing_m: float = 1.0,
        node_wall_buffer_m: float = 0.3,
        node_step_threshold_m: float = 0.25,
    ) -> None:
        self.goal = goal
        self.voxel_size = voxel_size
        self.robot_height = robot_height
        self.surface_dilation_passes = surface_dilation_passes
        self.surface_erosion_passes = surface_erosion_passes
        self.node_spacing_m = node_spacing_m
        self.node_wall_buffer_m = node_wall_buffer_m
        self.node_step_threshold_m = node_step_threshold_m

    def _path_from_waypoints(self, waypoints, ts: float) -> Path:  # type: ignore[no-untyped-def]
        poses: list[PoseStamped] = []
        if waypoints is not None:
            for x, y, z in waypoints:
                poses.append(
                    PoseStamped(
                        ts=ts,
                        frame_id="world",
                        position=(float(x), float(y), float(z)),
                        orientation=(0.0, 0.0, 0.0, 1.0),
                    )
                )
        return Path(ts=ts, frame_id="world", poses=poses)

    def __call__(
        self,
        upstream: Iterator[Observation[PointCloud2]],
    ) -> Iterator[Observation[Path]]:
        planner = MLSPlanner(
            voxel_size=self.voxel_size,
            robot_height=self.robot_height,
            surface_dilation_passes=self.surface_dilation_passes,
            surface_erosion_passes=self.surface_erosion_passes,
            node_spacing_m=self.node_spacing_m,
            node_wall_buffer_m=self.node_wall_buffer_m,
            node_step_threshold_m=self.node_step_threshold_m,
        )
        for obs in upstream:
            if obs.pose_tuple is None:
                logger.debug("MLSPlan: obs %s has no pose; skipping", obs.id)
                continue
            x, y, z, *_ = obs.pose_tuple
            start = (float(x), float(y), float(z) - self.robot_height)

            voxel_map = obs.data
            planner.update_global_map(voxel_map.points_f32())
            waypoints = planner.plan(start, self.goal)
            path = self._path_from_waypoints(waypoints, obs.ts)

            yield obs.derive(
                data=path,
                tags={
                    **obs.tags,
                    "voxel_map": voxel_map,
                    "surface_map": planner.surface_map(),
                    "nodes": planner.nodes(),
                    "node_edges": planner.node_edges(),
                    "start": start,
                    "planned": waypoints is not None,
                },
            )
