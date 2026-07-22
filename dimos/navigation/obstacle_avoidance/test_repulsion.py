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

import math
import random

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.obstacle_avoidance.repulsion import (
    ObstacleRepulsion,
    ObstacleTorque,
    body_corners,
    repulsion_from_costmap,
    torque_from_costmap,
)


def make_grid(grid, resolution=0.1, origin_x=0.0, origin_y=0.0):
    origin = Pose()
    origin.position.x = origin_x
    origin.position.y = origin_y
    origin.orientation.w = 1.0
    return OccupancyGrid(
        grid=np.asarray(grid, dtype=np.int8),
        resolution=resolution,
        origin=origin,
        frame_id="world",
    )


def wall_grid(width=20, height=20, wall_col=15):
    """20x20 free grid (2m x 2m at 0.1m) with a vertical wall of cost 100."""
    grid = np.zeros((height, width), dtype=np.int8)
    grid[:, wall_col] = 100
    return make_grid(grid)


class TestRepulsionFromCostmap:
    def test_pushes_away_from_wall(self):
        # Wall at x=1.55, robot at x=1.0 -> push toward -x
        force = repulsion_from_costmap(wall_grid(), (1.0, 1.0, 0.0))
        assert force.x < 0
        assert abs(force.y) < abs(force.x)  # wall is symmetric around robot y

    def test_empty_grid_is_zero(self):
        force = repulsion_from_costmap(make_grid(np.zeros((20, 20))), (1.0, 1.0, 0.0))
        assert np.allclose(force.data, 0)

    def test_unknown_cells_ignored(self):
        force = repulsion_from_costmap(make_grid(np.full((20, 20), -1)), (1.0, 1.0, 0.0))
        assert np.allclose(force.data, 0)

    def test_out_of_influence_radius_is_zero(self):
        force = repulsion_from_costmap(wall_grid(), (0.2, 1.0, 0.0), influence_radius=1.0)
        assert np.allclose(force.data, 0)

    def test_decays_with_distance(self):
        near = repulsion_from_costmap(wall_grid(), (1.3, 1.0, 0.0))
        far = repulsion_from_costmap(wall_grid(), (0.8, 1.0, 0.0))
        assert abs(near.x) > abs(far.x) > 0

    def test_scales_with_gain_and_cost(self):
        base = repulsion_from_costmap(wall_grid(), (1.0, 1.0, 0.0))
        doubled = repulsion_from_costmap(wall_grid(), (1.0, 1.0, 0.0), gain=2.0)
        assert math.isclose(doubled.x, 2 * base.x, rel_tol=1e-9)

        half_cost = wall_grid()
        half_cost.grid[half_cost.grid == 100] = 50
        halved = repulsion_from_costmap(half_cost, (1.0, 1.0, 0.0))
        assert math.isclose(halved.x, base.x / 2, rel_tol=1e-9)

    def test_roughly_resolution_independent(self):
        # Same 2m x 2m world with a wall at x~1.5, sampled at two resolutions
        coarse = repulsion_from_costmap(wall_grid(20, 20, 15), (1.0, 1.0, 0.0))
        fine_grid = np.zeros((40, 40), dtype=np.int8)
        fine_grid[:, 30:32] = 100
        fine = repulsion_from_costmap(make_grid(fine_grid, resolution=0.05), (1.0, 1.0, 0.0))
        assert math.isclose(fine.x, coarse.x, rel_tol=0.25)


def corridor_grid(width=40, height=20):
    """4m x 2m corridor along x at 0.1m: walls at y=0.05 and y=1.95."""
    grid = np.zeros((height, width), dtype=np.int8)
    grid[0, :] = 100
    grid[-1, :] = 100
    return make_grid(grid)


def posed(x, y, yaw):
    return Pose((x, y, 0.0), Quaternion.from_euler(Vector3(0.0, 0.0, yaw)))


class TestTorqueFromCostmap:
    def test_body_corners_oriented(self):
        corners = body_corners(posed(1.0, 1.0, math.pi / 2), (0.7, 0.3, 0.0))
        xs = [c.x for c in corners]
        ys = [c.y for c in corners]
        # 90°: long side now spans y, short side spans x
        assert max(xs) - min(xs) == pytest.approx(0.3)
        assert max(ys) - min(ys) == pytest.approx(0.7)

    def test_corridor_aligns_body(self):
        grid = corridor_grid()
        # Nose tilted toward the top wall -> torque must rotate it back (negative)
        assert torque_from_costmap(grid, posed(2.0, 1.0, 0.3)) < 0
        # Mirror image
        assert torque_from_costmap(grid, posed(2.0, 1.0, -0.3)) > 0

    def test_aligned_body_is_stable(self):
        grid = corridor_grid()
        aligned = torque_from_costmap(grid, posed(2.0, 1.0, 0.0))
        tilted = torque_from_costmap(grid, posed(2.0, 1.0, 0.3))
        assert abs(aligned) < abs(tilted)
        assert abs(aligned) == pytest.approx(0.0, abs=1e-6)

    def test_open_space_is_zero(self):
        grid = make_grid(np.zeros((20, 40)))
        assert torque_from_costmap(grid, posed(2.0, 1.0, 0.3)) == 0.0


def tight_corridor(width_m=0.5, length_m=4.0, resolution=0.05):
    """Corridor along x, one wall cell each side; free width is *width_m*."""
    rows = round(width_m / resolution) + 2
    cols = round(length_m / resolution)
    grid = np.zeros((rows, cols), dtype=np.int8)
    grid[0, :] = 100
    grid[-1, :] = 100
    return make_grid(grid, resolution=resolution)


class TestTightCorridor:
    """The go2 (0.70 x 0.31) must be able to pass a 0.5m corridor:
    the field aligns and centers the body instead of blocking it."""

    CENTER_Y = (0.5 + 2 * 0.05) / 2  # corridor centerline

    def test_yawed_body_gets_aligned(self):
        grid = tight_corridor()
        assert torque_from_costmap(grid, posed(2.0, self.CENTER_Y, 0.2)) < 0
        assert torque_from_costmap(grid, posed(2.0, self.CENTER_Y, -0.2)) > 0

    def test_aligned_centered_body_passes(self):
        # Aligned in the middle: no torque, no lateral push, no braking push
        grid = tight_corridor()
        pose = posed(2.0, self.CENTER_Y, 0.0)
        assert torque_from_costmap(grid, pose) == pytest.approx(0.0, abs=1e-9)
        force = repulsion_from_costmap(grid, pose.position)
        assert force.x == pytest.approx(0.0, abs=1e-9)
        assert force.y == pytest.approx(0.0, abs=1e-9)

    def test_off_center_body_is_pushed_back_to_centerline(self):
        grid = tight_corridor()
        force = repulsion_from_costmap(grid, (2.0, self.CENTER_Y + 0.08, 0.0))
        assert force.y < 0


@pytest.fixture(scope="module")
def observations():
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    store = SqliteStore(path=get_data("go2_hongkong_office.db"))
    lidar = store.streams.lidar.to_list()
    rng = random.Random(42)
    picks = [lidar[i] for i in sorted(rng.sample(range(len(lidar)), 5))]
    yield picks
    store.dispose()


@pytest.mark.self_hosted
class TestObstacleTorqueDataset:
    def test_frames(self, observations):
        transformer = ObstacleTorque()
        results = list(transformer(iter(observations)))
        assert len(results) == len(observations)

        for src, out in zip(observations, results, strict=True):
            twist = out.data
            assert isinstance(twist, Twist)
            assert np.allclose(twist.linear.data, 0)
            assert twist.angular.x == twist.angular.y == 0.0
            assert math.isfinite(twist.angular.z)
            # The transform matches the pure function it wraps
            assert twist.angular.z == pytest.approx(
                torque_from_costmap(
                    transformer.costmap(src.data),
                    src.pose,
                    influence_radius=transformer.influence_radius,
                )
            )


@pytest.mark.self_hosted
class TestObstacleRepulsionDataset:
    def test_random_frames(self, observations):
        transformer = ObstacleRepulsion()
        results = list(transformer(iter(observations)))
        assert len(results) == len(observations)

        for src, out in zip(observations, results, strict=True):
            twist = out.data
            assert isinstance(twist, Twist)
            assert np.allclose(twist.angular.data, 0)
            assert np.all(np.isfinite(twist.linear.data))

            # Cross-check the vectorized force against a plain per-cell loop
            costmap = transformer.costmap(src.data)
            pos = src.pose.position
            radius = transformer.influence_radius
            res = costmap.resolution
            fx = fy = 0.0
            for row, col in zip(
                *np.nonzero(costmap.grid >= transformer.cost_threshold), strict=False
            ):
                cx = costmap.origin.position.x + (col + 0.5) * res
                cy = costmap.origin.position.y + (row + 0.5) * res
                d = math.hypot(pos.x - cx, pos.y - cy)
                if not 1e-6 < d < radius:
                    continue
                mag = (costmap.grid[row, col] / 100.0) * res**2 * (1 / d - 1 / radius) / d**2
                fx += mag * (pos.x - cx) / d
                fy += mag * (pos.y - cy) / d
            assert twist.linear.x == pytest.approx(fx)
            assert twist.linear.y == pytest.approx(fy)
