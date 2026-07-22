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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.obstacle_avoidance.path_correction import (
    correct_path,
    footprint_clear,
    truncate_before_collision,
)
from dimos.navigation.obstacle_avoidance.test_repulsion import (
    make_grid,
    posed,
    wall_grid,
)


def straight_path(y, x0=0.2, x1=1.8, n=17):
    return Path(
        frame_id="world",
        poses=[PoseStamped(position=(x, y, 0.0)) for x in np.linspace(x0, x1, n)],
    )


class TestFootprintClear:
    def test_clear_in_open_space(self):
        grid = make_grid(np.zeros((20, 20)))
        assert footprint_clear(grid, posed(1.0, 1.0, 0.0))

    def test_corner_in_wall_detected(self):
        # Wall at x=1.55; body center at x=1.3 with 0.35 half-length reaches into it
        assert not footprint_clear(wall_grid(), posed(1.3, 1.0, 0.0))
        assert footprint_clear(wall_grid(), posed(1.0, 1.0, 0.0))

    def test_orientation_matters(self):
        # Sideways (yaw 90°) the long side spans y, keeping x extent at 0.155
        assert footprint_clear(wall_grid(), posed(1.3, 1.0, np.pi / 2))


class TestCorrectPath:
    def test_endpoints_fixed(self):
        # Path along a wall: y=1.0 path, wall row at y=1.55
        grid = make_grid(np.zeros((20, 20)))
        grid.grid[15, :] = 100
        path = straight_path(1.45)
        corrected = correct_path(path, grid)
        assert corrected.poses[0].position.x == pytest.approx(path.poses[0].position.x)
        assert corrected.poses[0].position.y == pytest.approx(path.poses[0].position.y)
        assert corrected.poses[-1].position.x == pytest.approx(path.poses[-1].position.x)
        assert corrected.poses[-1].position.y == pytest.approx(path.poses[-1].position.y)

    def test_pushed_away_from_wall(self):
        # Horizontal wall at y=1.55, path skimming it at y=1.45
        grid = make_grid(np.zeros((20, 20)))
        grid.grid[15, :] = 100
        corrected = correct_path(straight_path(1.45), grid)
        interior = [p.position.y for p in corrected.poses[2:-2]]
        assert max(interior) < 1.45  # every interior waypoint moved away

    def test_open_space_unchanged(self):
        grid = make_grid(np.zeros((20, 20)))
        path = straight_path(1.0)
        corrected = correct_path(path, grid)
        for orig, new in zip(path.poses, corrected.poses, strict=True):
            assert new.position.x == pytest.approx(orig.position.x, abs=1e-6)
            assert new.position.y == pytest.approx(orig.position.y, abs=1e-6)

    def test_footprint_clear_after_correction(self):
        grid = make_grid(np.zeros((20, 20)))
        grid.grid[15, :] = 100
        corrected = correct_path(straight_path(1.4), grid)
        for pose in corrected.poses[1:-1]:
            assert footprint_clear(grid, pose)

    def test_orientation_follows_path(self):
        grid = make_grid(np.zeros((20, 20)))
        corrected = correct_path(straight_path(1.0), grid)
        for pose in corrected.poses:
            assert pose.yaw == pytest.approx(0.0, abs=1e-6)

    def test_short_path_passthrough(self):
        grid = make_grid(np.zeros((20, 20)))
        path = Path(frame_id="world", poses=[PoseStamped(position=(0.2, 1.0, 0.0))])
        assert correct_path(path, grid) is path


class TestTruncateBeforeCollision:
    def wall_across(self):
        # Vertical wall at x=1.55 blocking a straight x-bound path at y=1.0
        return wall_grid()

    def x_path(self, x0=0.2, x1=1.8, n=17):
        return Path(
            frame_id="world",
            poses=[PoseStamped(position=(x, 1.0, 0.0)) for x in np.linspace(x0, x1, n)],
        )

    def test_clear_path_unchanged(self):
        grid = make_grid(np.zeros((20, 20)))
        path = self.x_path()
        assert truncate_before_collision(path, grid) is path

    def test_truncates_before_wall(self):
        grid = self.wall_across()
        truncated = truncate_before_collision(self.x_path(), grid, margin=0.2)
        assert 0 < len(truncated.poses) < 17
        # Collision starts once the footprint nose (0.35) reaches the wall
        # cells (x ~ 1.5); the cut backs off another 0.2m of arc.
        first_hit = next(p for p in self.x_path().poses if not footprint_clear(grid, p))
        assert truncated.poses[-1].position.x <= first_hit.position.x - 0.2 + 1e-6
        for pose in truncated.poses:
            assert footprint_clear(grid, pose)

    def test_start_in_collision_empty(self):
        grid = self.wall_across()
        truncated = truncate_before_collision(self.x_path(x0=1.4), grid)
        assert len(truncated.poses) == 0
