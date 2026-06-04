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

"""Sample point clouds for path planning."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.evaluator.mesh_loader import load_voxelized_mesh
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WORLD_FRAME = "map"
_WALL_HEIGHT_M = 2.0
_WALL_THICKNESS_M = 0.5

MESH_PATH = os.environ.get("MESH_PATH")


@dataclass
class PlannerScenario:
    name: str
    global_map: PointCloud2
    start_pose: PoseStamped
    goal_pose: PoseStamped
    expect_path: bool


def _pose(x: float, y: float, z: float = 0.0, frame: str = WORLD_FRAME) -> PoseStamped:
    return PoseStamped(
        frame_id=frame,
        position=[x, y, z],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )


def _wall(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    *,
    spacing: float = 0.1,
    height: float = _WALL_HEIGHT_M,
    thickness: float = _WALL_THICKNESS_M,
    frame: str = WORLD_FRAME,
) -> PointCloud2:
    """Sample a vertical wall as a 3D box from (x0,y0) to (x1,y1).

    Thickness extends perpendicular to the wall line in the XY plane.
    """
    dx, dy = x1 - x0, y1 - y0
    length = float(np.hypot(dx, dy))
    if length == 0:
        return PointCloud2.from_numpy(
            np.zeros((0, 3), dtype=np.float32), frame_id=frame, timestamp=0.0
        )
    perp_x, perp_y = -dy / length, dx / length
    along = np.linspace(0.0, 1.0, max(2, int(np.ceil(length / spacing))))
    perp = np.linspace(-thickness / 2, thickness / 2, max(1, int(np.ceil(thickness / spacing)) + 1))
    zs = np.linspace(0.0, height, max(2, int(np.ceil(height / spacing))))
    a, p, z = np.meshgrid(along, perp, zs, indexing="ij")
    x = x0 + a.ravel() * dx + p.ravel() * perp_x
    y = y0 + a.ravel() * dy + p.ravel() * perp_y
    pts = np.column_stack([x, y, z.ravel()]).astype(np.float32)
    return PointCloud2.from_numpy(pts, frame_id=frame, timestamp=0.0)


def _floor(
    x_min: float = -2.0,
    x_max: float = 8.0,
    y_min: float = -3.0,
    y_max: float = 3.0,
    spacing: float = 0.25,
    frame: str = WORLD_FRAME,
) -> PointCloud2:
    """Flat ground plane sampled as points at z=0."""
    xs = np.arange(x_min, x_max + spacing, spacing)
    ys = np.arange(y_min, y_max + spacing, spacing)
    grid_xs, grid_ys = np.meshgrid(xs, ys)
    pts = np.column_stack([grid_xs.ravel(), grid_ys.ravel(), np.zeros(grid_xs.size)]).astype(
        np.float32
    )
    return PointCloud2.from_numpy(pts, frame_id=frame, timestamp=0.0)


def _map_with_walls(*walls: PointCloud2) -> PointCloud2:
    return sum(walls, _floor())


def empty_floor() -> PlannerScenario:
    return PlannerScenario(
        name="empty_floor",
        global_map=_floor(),
        start_pose=_pose(-1.0, 0.0, 0.2),
        goal_pose=_pose(7.0, 0.0, 0.2),
        expect_path=True,
    )


def blocked_wall() -> PlannerScenario:
    return PlannerScenario(
        name="blocked_wall",
        global_map=_map_with_walls(_wall(3.0, -3.0, 3.0, 3.0)),
        start_pose=_pose(-1.0, 0.0, 0.2),
        goal_pose=_pose(6.0, 0.0, 0.2),
        expect_path=False,
    )


def two_rooms_one_door() -> PlannerScenario:
    return PlannerScenario(
        name="two_rooms_one_door",
        global_map=_map_with_walls(
            _wall(3.0, -3.0, 3.0, -0.75),
            _wall(3.0, 0.75, 3.0, 3.0),
        ),
        start_pose=_pose(-1.0, 0.0, 0.2),
        goal_pose=_pose(6.0, 0.0, 0.2),
        expect_path=True,
    )


def _mesh_scenarios() -> list[PlannerScenario]:
    """Two scenarios on a real building mesh: ground-level traverse and a stair climb."""
    if MESH_PATH is None:
        logger.info("MESH_PATH not set, skipping mesh scenarios")
        return []
    if not Path(MESH_PATH).is_file():
        logger.warning("Mesh file not found, skipping mesh scenarios", path=MESH_PATH)
        return []
    points = load_voxelized_mesh(MESH_PATH).astype(np.float32)
    return [
        PlannerScenario(
            name="mesh_outside",
            global_map=PointCloud2.from_numpy(points, frame_id=WORLD_FRAME, timestamp=0.0),
            start_pose=_pose(-20.45, -19.85, 1.75),
            goal_pose=_pose(21.95, -4.25, 1.75),
            expect_path=True,
        ),
        PlannerScenario(
            name="mesh_up_the_stairs",
            global_map=PointCloud2.from_numpy(points, frame_id=WORLD_FRAME, timestamp=0.0),
            start_pose=_pose(7.15, -3.55, 2.05),
            goal_pose=_pose(5.55, -2.05, 5.65),
            expect_path=True,
        ),
    ]


def default_scenarios() -> list[PlannerScenario]:
    return [empty_floor(), blocked_wall(), two_rooms_one_door(), *_mesh_scenarios()]
