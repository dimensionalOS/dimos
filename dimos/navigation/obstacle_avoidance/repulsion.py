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

from dataclasses import asdict
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.memory2.module import StreamModule
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def repulsion_from_costmap(
    costmap: OccupancyGrid,
    position: VectorLike,
    *,
    influence_radius: float = 1.5,
    gain: float = 1.0,
    cost_threshold: int = 50,
) -> Vector3:
    """Potential-field repulsive force at *position* from a costmap.

    Classic APF: each obstacle cell within ``influence_radius`` pushes the
    robot away with magnitude ``gain * (1/d - 1/R) / d²``, weighted by the
    cell cost (0-100 → 0-1) and cell area (resolution²) so the result is
    roughly independent of grid resolution. Returns a world-frame force
    with z=0.
    """
    pos = Vector3(position)
    grid = costmap.grid
    rows, cols = np.nonzero(grid >= cost_threshold)
    if len(rows) == 0:
        return Vector3(0.0, 0.0, 0.0)

    res = costmap.resolution
    cell_x = costmap.origin.position.x + (cols + 0.5) * res
    cell_y = costmap.origin.position.y + (rows + 0.5) * res

    dx = pos.x - cell_x
    dy = pos.y - cell_y
    d = np.sqrt(dx**2 + dy**2)

    near = (d > 1e-6) & (d < influence_radius)
    if not np.any(near):
        return Vector3(0.0, 0.0, 0.0)

    d = d[near]
    weight = grid[rows[near], cols[near]] / 100.0
    magnitude = gain * weight * res**2 * (1.0 / d - 1.0 / influence_radius) / d**2

    fx = float(np.sum(magnitude * dx[near] / d))
    fy = float(np.sum(magnitude * dy[near] / d))
    return Vector3(fx, fy, 0.0)


class ObstacleRepulsion(Transformer[PointCloud2, Twist]):
    """Turn each lidar frame into a repulsive velocity nudge (world frame).

    Collapses the cloud into a costmap (same algos as CostMapper) and sums
    the potential-field push away from nearby obstacle cells. The output
    Twist is linear-only — combine with a heading controller downstream.
    Observations without a pose are skipped.
    """

    def __init__(
        self,
        *,
        algo: str = "height_cost",
        occupancy: OccupancyConfig | None = None,
        influence_radius: float = 1.5,
        gain: float = 1.0,
        cost_threshold: int = 50,
    ) -> None:
        self.algo = algo
        self.occupancy = occupancy if occupancy is not None else HeightCostConfig()
        self.influence_radius = influence_radius
        self.gain = gain
        self.cost_threshold = cost_threshold

    def costmap(self, cloud: PointCloud2) -> OccupancyGrid:
        return OCCUPANCY_ALGOS[self.algo](cloud, **asdict(self.occupancy))

    def force(self, cloud: PointCloud2, position: VectorLike) -> Vector3:
        return repulsion_from_costmap(
            self.costmap(cloud),
            position,
            influence_radius=self.influence_radius,
            gain=self.gain,
            cost_threshold=self.cost_threshold,
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[Twist]]:
        for obs in upstream:
            pose = obs.pose
            if pose is None:
                continue
            yield obs.derive(data=Twist(self.force(obs.data, pose.position), (0.0, 0.0, 0.0)))


def body_corners(pose: Pose, body: VectorLike) -> list[Vector3]:
    """World positions of the corners of an oriented rectangular footprint."""
    size = Vector3(body)
    hx, hy = size.x / 2, size.y / 2
    yaw = pose.yaw
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        Vector3(
            pose.position.x + dx * cos_yaw - dy * sin_yaw,
            pose.position.y + dx * sin_yaw + dy * cos_yaw,
            0.0,
        )
        for dx, dy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))
    ]


def torque_from_costmap(
    costmap: OccupancyGrid,
    pose: Pose,
    *,
    body: VectorLike = (0.70, 0.31, 0.0),
    influence_radius: float = 0.8,
    gain: float = 1.0,
    cost_threshold: int = 50,
) -> float:
    """Torque (z, right-handed) from per-corner repulsion of an oriented body.

    Each footprint corner samples the repulsive field; its force contributes
    torque about the body center like a rigid body: ``τ = Σ cross(r, F)``. In a
    corridor this rotates the body parallel to the walls; a symmetric pinch
    (doorway) cancels to zero.
    """
    tau = 0.0
    for corner in body_corners(pose, body):
        force = repulsion_from_costmap(
            costmap,
            corner,
            influence_radius=influence_radius,
            gain=gain,
            cost_threshold=cost_threshold,
        )
        rx = corner.x - pose.position.x
        ry = corner.y - pose.position.y
        tau += rx * force.y - ry * force.x
    return tau


class ObstacleTorque(ObstacleRepulsion):
    """Turn each lidar frame into a rotation nudge (angular.z-only Twist).

    Samples the repulsive field at the corners of the robot's footprint and
    sums the rigid-body torque, so an elongated body rotates to align with
    tight spaces (corridors, tunnels) instead of wedging into them. Combine
    with :class:`ObstacleRepulsion` downstream; the two are independent.
    """

    def __init__(
        self,
        *,
        body: VectorLike = (0.70, 0.31, 0.0),
        influence_radius: float = 0.8,
        **kwargs: Any,
    ) -> None:
        super().__init__(influence_radius=influence_radius, **kwargs)
        self.body = Vector3(body)

    def torque(self, cloud: PointCloud2, pose: Pose) -> float:
        return torque_from_costmap(
            self.costmap(cloud),
            pose,
            body=self.body,
            influence_radius=self.influence_radius,
            gain=self.gain,
            cost_threshold=self.cost_threshold,
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[Twist]]:
        for obs in upstream:
            pose = obs.pose
            if pose is None:
                continue
            yield obs.derive(data=Twist((0.0, 0.0, 0.0), (0.0, 0.0, self.torque(obs.data, pose))))


class ObstacleRepulsionConfig(ModuleConfig):
    algo: str = "height_cost"
    influence_radius: float = 1.5
    gain: float = 1.0
    cost_threshold: int = 50
    root_frame: str = "world"
    robot_frame: str = "base_link"
    tf_tolerance: float = 0.5


class _RepulsionModuleBase(StreamModule[PointCloud2, Twist]):
    """Shared pipeline: attach robot pose via tf, then apply the transformer."""

    config: ObstacleRepulsionConfig
    transformer_cls: type[ObstacleRepulsion]

    def transformer(self) -> ObstacleRepulsion:
        cfg = self.config.model_dump(
            include=set(type(self.config).model_fields)
            - set(ModuleConfig.model_fields)
            - {"root_frame", "robot_frame", "tf_tolerance"}
        )
        return self.transformer_cls(**cfg)

    def pipeline(self, stream: Stream[PointCloud2]) -> Stream[Twist]:
        # Live appends carry no pose — look up the robot pose via tf.
        def attach_pose(obs: Observation[PointCloud2]) -> Observation[PointCloud2]:
            if obs.pose is not None:
                return obs
            transform = self.tf.get(
                self.config.root_frame,
                self.config.robot_frame,
                time_point=obs.ts,
                time_tolerance=self.config.tf_tolerance,
            )
            if transform is None:
                return obs
            return obs.derive(data=obs.data, pose=transform.to_pose())

        return stream.map(attach_pose).transform(self.transformer())

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


class ObstacleRepulsionModule(_RepulsionModuleBase):
    """Publish a repulsive Twist for each incoming lidar frame."""

    config: ObstacleRepulsionConfig
    transformer_cls = ObstacleRepulsion

    lidar: In[PointCloud2]
    repulsion: Out[Twist]


class ObstacleTorqueConfig(ObstacleRepulsionConfig):
    body: tuple[float, float, float] = (0.70, 0.31, 0.0)
    influence_radius: float = 0.8


class ObstacleTorqueModule(_RepulsionModuleBase):
    """Publish a rotation-only Twist for each incoming lidar frame."""

    config: ObstacleTorqueConfig
    transformer_cls = ObstacleTorque

    lidar: In[PointCloud2]
    torque: Out[Twist]
