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

"""Hand-crafted synthetic scenarios for black-box planner evaluation.

Each scenario is a self-contained (map, start, goal, expectation) bundle.
Maps are PointCloud2 obstacle clouds, start/goal are Odometry poses, and
``expect_path`` records whether a planner should be able to find a path
(used by the evaluator to score pass/fail).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.evaluator.mesh_loader import load_voxelized_mesh

WORLD_FRAME = "map"
_WALL_HEIGHT_M = 2.0

MESH_PATH = "/home/andrew/Downloads/19_fairdale_ave_papakura.glb"


@dataclass
class PlannerScenario:
    name: str
    global_map: PointCloud2
    start_pose: Odometry
    goal_pose: Odometry
    expect_path: bool


def _odom(x: float, y: float, z: float = 0.0, frame: str = WORLD_FRAME) -> Odometry:
    pose = Pose(position=[x, y, z], orientation=[0.0, 0.0, 0.0, 1.0])
    return Odometry(frame_id=frame, child_frame_id="body", pose=pose)


def _cloud(points: np.ndarray, frame: str = WORLD_FRAME) -> PointCloud2:
    if points.size == 0:
        points = np.zeros((0, 3), dtype=np.float32)
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id=frame)


_WALL_THICKNESS_M = 0.5


def _wall(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    *,
    spacing: float = 0.1,
    height: float = _WALL_HEIGHT_M,
    thickness: float = _WALL_THICKNESS_M,
) -> np.ndarray:
    """Sample a vertical wall as a 3D box from (x0,y0) to (x1,y1).

    Thickness extends perpendicular to the wall line in the XY plane.
    """
    dx, dy = x1 - x0, y1 - y0
    length = float(np.hypot(dx, dy))
    if length == 0:
        return np.zeros((0, 3), dtype=np.float32)
    perp_x, perp_y = -dy / length, dx / length
    along = np.linspace(0.0, 1.0, max(2, int(np.ceil(length / spacing))))
    perp = np.linspace(-thickness / 2, thickness / 2, max(1, int(np.ceil(thickness / spacing)) + 1))
    zs = np.linspace(0.0, height, max(2, int(np.ceil(height / spacing))))
    a, p, z = np.meshgrid(along, perp, zs, indexing="ij")
    x = x0 + a.ravel() * dx + p.ravel() * perp_x
    y = y0 + a.ravel() * dy + p.ravel() * perp_y
    return np.column_stack([x, y, z.ravel()])


def _floor(
    x_min: float = -2.0,
    x_max: float = 8.0,
    y_min: float = -3.0,
    y_max: float = 3.0,
    spacing: float = 0.25,
) -> np.ndarray:
    """Flat ground plane sampled as points at z=0."""
    xs = np.arange(x_min, x_max + spacing, spacing)
    ys = np.arange(y_min, y_max + spacing, spacing)
    grid_xs, grid_ys = np.meshgrid(xs, ys)
    return np.column_stack([grid_xs.ravel(), grid_ys.ravel(), np.zeros(grid_xs.size)])


def _map_with_walls(*walls: np.ndarray) -> PointCloud2:
    return _cloud(np.vstack([_floor(), *walls]))


def empty_floor() -> PlannerScenario:
    return PlannerScenario(
        name="empty_floor",
        global_map=_cloud(_floor()),
        start_pose=_odom(-1.0, 0.0, 0.2),
        goal_pose=_odom(7.0, 0.0, 0.2),
        expect_path=True,
    )


def blocked_wall() -> PlannerScenario:
    return PlannerScenario(
        name="blocked_wall",
        global_map=_map_with_walls(_wall(3.0, -3.0, 3.0, 3.0)),
        start_pose=_odom(-1.0, 0.0, 0.2),
        goal_pose=_odom(6.0, 0.0, 0.2),
        expect_path=False,
    )


def two_rooms_one_door() -> PlannerScenario:
    return PlannerScenario(
        name="two_rooms_one_door",
        global_map=_map_with_walls(
            _wall(3.0, -3.0, 3.0, -0.75),
            _wall(3.0, 0.75, 3.0, 3.0),
        ),
        start_pose=_odom(-1.0, 0.0, 0.2),
        goal_pose=_odom(6.0, 0.0, 0.2),
        expect_path=True,
    )


def _mesh_scenarios() -> list[PlannerScenario]:
    """Two scenarios on a real building mesh: ground-level traverse and a stair climb."""
    cloud = _cloud(load_voxelized_mesh(MESH_PATH))
    return [
        PlannerScenario(
            name="mesh_outside",
            global_map=cloud,
            start_pose=_odom(-20.45, -19.85, 1.75),
            goal_pose=_odom(21.95, -4.25, 1.75),
            expect_path=True,
        ),
        PlannerScenario(
            name="mesh_up_the_stairs",
            global_map=cloud,
            start_pose=_odom(7.15, -3.55, 2.05),
            goal_pose=_odom(5.55, -2.05, 5.65),
            expect_path=True,
        ),
    ]


def default_scenarios() -> list[PlannerScenario]:
    return [empty_floor(), blocked_wall(), two_rooms_one_door(), *_mesh_scenarios()]
