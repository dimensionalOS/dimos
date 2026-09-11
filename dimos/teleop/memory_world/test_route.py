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

import itertools
import math

import numpy as np
import pytest

from dimos.teleop.memory_world.route import LETHAL, RoutePlanner

VOXEL = 0.1
BODY_Z = 0.4  # the robot's base height above the floor at z = 0


def _path(*waypoints: tuple[float, float]) -> np.ndarray:
    """The robot's base along straight legs between waypoints, a point every 5 cm."""
    points = []
    for (x0, y0), (x1, y1) in itertools.pairwise(waypoints):
        n = max(int(math.dist((x0, y0), (x1, y1)) / 0.05), 2)
        for t in np.linspace(0, 1, n):
            points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, BODY_Z))
    return np.asarray(points)


# Drove from the left half through the doorway into the right half and back down.
DOORWAY_DRIVE = _path((1, 1), (1, 4.6), (9, 4.6), (9, 1))


def _floor(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)


def _wall(x0: float, x1: float, y0: float, y1: float, height: float = 1.5) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    zs = np.arange(VOXEL, height, VOXEL)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def _room_with_doorway() -> np.ndarray:
    """A 10 x 6 m floor with a wall across x = 5 that has a 1.2 m gap at y in [4, 5.2]."""
    return np.concatenate(
        [
            _floor(0, 10, 0, 6),
            _wall(4.9, 5.1, 0, 4.0),
            _wall(4.9, 5.1, 5.2, 6.0),
        ]
    )


def test_costmap_marks_corridor_free_and_walls_and_unseen_space_lethal() -> None:
    planner = RoutePlanner.from_voxels(_room_with_doorway(), DOORWAY_DRIVE, voxel_size=VOXEL)
    assert planner.costs[planner.cell_of((1.5, 2.0))] == 0
    assert 0 <= planner.costs[planner.cell_of((5.0, 4.6))] < LETHAL, (
        "the doorway itself is passable"
    )
    assert planner.costs[planner.cell_of((5.0, 3.5))] == LETHAL, "the wall"
    # Within the robot's radius of the wall is lethal too, further out only costly.
    assert planner.costs[planner.cell_of((5.3, 3.5))] == LETHAL
    assert 0 < planner.costs[planner.cell_of((5.7, 4.0))] < LETHAL
    assert planner.costs[0, 0] == LETHAL, "space the robot never came near is not planned over"
    assert planner.costs[planner.cell_of((3.0, 2.0))] == LETHAL, "nor floor beyond the corridor"
    assert abs(planner.floor_at((1.5, 2.0)) - (BODY_Z - 0.2)) < 1e-9


def test_route_goes_through_the_doorway() -> None:
    planner = RoutePlanner.from_voxels(_room_with_doorway(), DOORWAY_DRIVE, voxel_size=VOXEL)
    route = planner.plan((1.0, 1.0), (9.0, 1.0))
    assert route is not None
    xs = np.asarray([p[0] for p in route.points])
    ys = np.asarray([p[1] for p in route.points])
    crossing = ys[np.argmin(np.abs(xs - 5.0))]
    assert 4.0 < crossing < 5.2, f"crossed the wall at y={crossing}"
    assert route.length_m > math.dist((1, 1), (9, 1)) + 2, (
        "the detour must be longer than the straight line"
    )
    assert all(abs(p[2] - (BODY_Z - 0.2)) < 1e-9 for p in route.points), (
        "drawn a little under the base height"
    )
    assert route.cells == len(route.points)


def test_goal_inside_an_obstacle_snaps_to_the_nearest_free_cell() -> None:
    voxels = np.concatenate([_floor(0, 6, 0, 6), _wall(2.8, 3.4, 2.8, 3.4, height=0.9)])  # a crate
    planner = RoutePlanner.from_voxels(
        voxels, _path((0.5, 0.5), (0.5, 5.5), (5.5, 5.5), (5.5, 0.5), (0.5, 0.5)), voxel_size=VOXEL
    )
    route = planner.plan((0.5, 0.5), (3.1, 3.1))
    assert route is not None
    end = route.points[-1]
    assert 0.3 < math.dist(end[:2], (3.1, 3.1)) < 1.5, "ends next to the crate, not inside it"


def test_no_route_when_walled_off() -> None:
    voxels = np.concatenate(
        [_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)]
    )  # no doorway, wall past the floor edges
    left_only = _path((1, 1), (1, 5), (4, 5), (4, 1))  # the robot stayed on its side
    planner = RoutePlanner.from_voxels(voxels, left_only, voxel_size=VOXEL)
    assert planner.plan((1.0, 1.0), (9.0, 1.0)) is None


def test_sparse_path_samples_still_make_one_corridor() -> None:
    sparse = DOORWAY_DRIVE[::20]  # a metre between samples
    planner = RoutePlanner.from_voxels(_room_with_doorway(), sparse, voxel_size=VOXEL)
    route = planner.plan((1.0, 1.0), (9.0, 1.0))
    assert route is not None and route.length_m > 8


def test_unreachable_snap_returns_none() -> None:
    planner = RoutePlanner.from_voxels(
        _floor(0, 2, 0, 2), _path((0.5, 0.5), (1.5, 1.5)), voxel_size=VOXEL
    )
    assert planner.snap((30.0, 30.0)) is None


def test_city_scale_map_plans_on_coarse_cells() -> None:
    # A 4 km straight road: at 10 cm cells the grid would be 40k cells across.
    road = _floor(0, 4000, 0, 4)
    drive = _path((1, 2), (3999, 2))
    planner = RoutePlanner.from_voxels(road[::7], drive, voxel_size=VOXEL)
    assert 0.9 < planner.resolution < 1.2
    assert planner.costs.shape[1] < 4200
    route = planner.plan((10, 2), (3990, 2))
    assert route is not None and route.length_m > 3900


def test_mls_planner_routes_through_the_doorway() -> None:
    pytest.importorskip("dimos_mls_planner")
    from dimos.teleop.memory_world.route import MlsRoutePlanner

    planner = MlsRoutePlanner(_room_with_doorway(), voxel_size=VOXEL)
    assert planner.surface_cells > 100
    route = planner.plan((1.0, 1.0, 0.0), (9.0, 1.0, 0.0))
    assert route is not None and route.planner == "mls"
    xs = np.asarray([p[0] for p in route.points])
    ys = np.asarray([p[1] for p in route.points])
    crossing = ys[np.argmin(np.abs(xs - 5.0))]
    assert 4.0 < crossing < 5.2, f"crossed the wall at y={crossing}"
