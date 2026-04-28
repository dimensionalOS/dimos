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

from collections.abc import Callable

import pytest

from dimos.navigation.nav_stack.modules.simple_planner.simple_planner import (
    Costmap,
    SimplePlanner,
    StuckState,
    _blocked_at_inflation,
    astar,
    plan_on_costmap,
    progress_tick,
)

_DEFAULT_MAX_EXPANSIONS = 200_000


class TestCostmap:
    def test_world_cell_roundtrip(self):
        costmap = Costmap(cell_size=0.5, obstacle_height=0.1, inflation_radius=0.0)
        for x, y in [(0.0, 0.0), (1.25, -2.75), (10.1, 4.4)]:
            ix, iy = costmap.world_to_cell(x, y)
            cx, cy = costmap.cell_to_world(ix, iy)
            # Cell center is within half-cell of original
            assert abs(cx - x) <= 0.5
            assert abs(cy - y) <= 0.5

    def test_height_max_tracks_tallest(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.5, inflation_radius=0.0)
        costmap.update(0.1, 0.1, 0.2)
        costmap.update(0.2, 0.3, 0.8)
        costmap.update(0.4, 0.4, 0.4)  # same cell, smaller than 0.8
        assert costmap.is_blocked(0, 0)  # 0.8 > 0.5

    def test_height_below_threshold_not_blocked(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.5, inflation_radius=0.0)
        costmap.update(0.5, 0.5, 0.3)  # below threshold
        assert not costmap.is_blocked(0, 0)

    def test_clear_wipes_obstacles(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=0.0)
        costmap.update(0.0, 0.0, 1.0)
        assert costmap.is_blocked(0, 0)
        costmap.clear()
        assert not costmap.is_blocked(0, 0)

    def test_inflation_blocks_neighbours(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=1.5)
        costmap.update(0.0, 0.0, 1.0)
        # Center is blocked
        assert costmap.is_blocked(0, 0)
        # Cells within radius 1.5 are blocked (Manhattan dist ≤ 1 is always in a circle of r=1.5)
        assert costmap.is_blocked(1, 0)
        assert costmap.is_blocked(0, 1)
        assert costmap.is_blocked(-1, 0)
        assert costmap.is_blocked(1, 1)  # sqrt(2) ≈ 1.41 < 1.5
        # Cells outside radius 1.5 are not blocked
        assert not costmap.is_blocked(2, 0)
        assert not costmap.is_blocked(0, 2)

    def test_zero_inflation(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=0.0)
        costmap.update(0.0, 0.0, 1.0)
        assert costmap.is_blocked(0, 0)
        assert not costmap.is_blocked(1, 0)

    def test_invalid_cell_size(self):
        with pytest.raises(ValueError):
            Costmap(cell_size=0.0, obstacle_height=0.1, inflation_radius=0.0)
        with pytest.raises(ValueError):
            Costmap(cell_size=-1.0, obstacle_height=0.1, inflation_radius=0.0)

    def test_invalid_inflation(self):
        with pytest.raises(ValueError):
            Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=-0.1)


def _never_blocked(ix: int, iy: int) -> bool:
    return False


def _blocked_set(cells: set[tuple[int, int]]) -> Callable[[int, int], bool]:
    def _inner(ix: int, iy: int) -> bool:
        return (ix, iy) in cells

    return _inner


class TestAstar:
    def test_trivial_same_cell(self):
        assert astar((3, 4), (3, 4), _never_blocked) == [(3, 4)]

    def test_straight_line_no_obstacles(self):
        path = astar((0, 0), (5, 0), _never_blocked)
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (5, 0)
        # 5 straight steps → 6 cells
        assert len(path) == 6

    def test_diagonal_no_obstacles(self):
        path = astar((0, 0), (3, 3), _never_blocked)
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (3, 3)
        # Prefer diagonal: 3 moves + 1 cell = 4 cells
        assert len(path) == 4

    def test_wall_detours(self):
        # vertical wall at x=2 for y in [-1..1], need to go around
        wall = {(2, -1), (2, 0), (2, 1)}
        path = astar((0, 0), (4, 0), _blocked_set(wall))
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (4, 0)
        # Must not pass through wall cells
        for cell in path:
            assert cell not in wall

    def test_unreachable_goal(self):
        # Enclosed goal
        wall = {(2, -1), (2, 0), (2, 1), (1, -1), (3, -1), (1, 1), (3, 1), (2, 2)}
        # Add missing walls to fully enclose (2, 0)
        wall |= {(1, 0), (3, 0)}  # but goal is (2, 0) which is inside walls — wait
        # Actually goal (2, 0) IS in the wall. Use a different example.
        wall = {
            (0, 1),
            (1, 1),
            (2, 1),
            (2, 0),
            (0, -1),
            (1, -1),
            (2, -1),
            (-1, -1),
            (-1, 0),
            (-1, 1),
        }  # encloses (0, 0) and (1, 0)
        # Goal outside the box
        path = astar((0, 0), (5, 0), _blocked_set(wall))
        assert path is None

    def test_max_expansions_cap(self):
        # Should give up instead of hanging
        path = astar((0, 0), (10000, 10000), _never_blocked, max_expansions=100)
        assert path is None

    def test_octile_prefers_diagonal(self):
        # 4 straight moves vs 2 diagonal + 2 straight = same displacement
        # but A* should find the optimal octile path.
        path = astar((0, 0), (2, 2), _never_blocked)
        assert path is not None
        # Two diagonal steps = 3 cells
        assert len(path) == 3


class TestSimplePlannerPlan:
    def _make_costmap(self, cell_size=0.5):
        return Costmap(cell_size=cell_size, obstacle_height=0.1, inflation_radius=0.0)

    def test_plan_straight_open_path(self):
        costmap = self._make_costmap(cell_size=0.5)
        path = plan_on_costmap(costmap, 0.0, 0.0, 2.0, 0.0, _DEFAULT_MAX_EXPANSIONS)
        assert path is not None
        assert path[0][0] == pytest.approx(0.25)
        assert path[0][1] == pytest.approx(0.25)
        assert path[-1][0] == pytest.approx(2.25)
        assert path[-1][1] == pytest.approx(0.25)

    def test_plan_routes_around_obstacle(self):
        costmap = self._make_costmap(cell_size=0.5)
        for y in (-0.5, 0.0, 0.5, 1.0):
            costmap.update(1.0, y, 1.0)
        path = plan_on_costmap(costmap, 0.0, 0.0, 2.0, 0.0, _DEFAULT_MAX_EXPANSIONS)
        assert path is not None
        blocked = costmap.blocked_cells()
        for wx, wy in path:
            ix, iy = costmap.world_to_cell(wx, wy)
            assert (
                (ix, iy) not in blocked
                or (ix, iy) == costmap.world_to_cell(0.0, 0.0)
                or (ix, iy) == costmap.world_to_cell(2.0, 0.0)
            )

    def test_plan_returns_none_when_blocked(self):
        costmap = self._make_costmap(cell_size=1.0)
        gx, gy = 5.0, 0.0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
            costmap.update(gx + dx * 1.0, gy + dy * 1.0, 1.0)
        path = plan_on_costmap(costmap, 0.0, 0.0, gx, gy, _DEFAULT_MAX_EXPANSIONS)
        assert path is None

    def test_lookahead_picks_far_enough(self):
        path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
        wx, wy = SimplePlanner._lookahead(path, 0.0, 0.0, 1.0)
        assert wx == pytest.approx(1.0)
        assert wy == pytest.approx(0.0)

    def test_lookahead_falls_back_to_end(self):
        path = [(0.0, 0.0), (0.1, 0.0)]
        wx, wy = SimplePlanner._lookahead(path, 0.0, 0.0, 5.0)
        assert wx == pytest.approx(0.1)
        assert wy == pytest.approx(0.0)

    def test_lookahead_empty_path(self):
        wx, wy = SimplePlanner._lookahead([], 3.0, 4.0, 1.0)
        assert wx == 3.0 and wy == 4.0

    def test_plan_with_inflation_override_opens_doorway(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=1.0)
        for ix in range(-3, 4):
            costmap.update(float(ix), -1.0, 1.0)
            costmap.update(float(ix), 7.0, 1.0)
        for iy in range(-1, 8):
            costmap.update(-3.0, float(iy), 1.0)
            costmap.update(3.0, float(iy), 1.0)
        for ix in range(-2, 3):
            if ix == 0:
                continue
            costmap.update(float(ix), 3.0, 1.0)
        assert plan_on_costmap(costmap, 0.0, 0.0, 0.0, 6.0, _DEFAULT_MAX_EXPANSIONS) is None
        path = plan_on_costmap(
            costmap, 0.0, 0.0, 0.0, 6.0, _DEFAULT_MAX_EXPANSIONS, inflation_override=0.0
        )
        assert path is not None
        assert any(costmap.world_to_cell(wx, wy) == (0, 3) for wx, wy in path)

    def test_lookahead_moving_robot(self):
        path = [(x, 0.0) for x in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
        wx, wy = SimplePlanner._lookahead(path, 2.0, 0.0, 1.5)
        assert wx == pytest.approx(4.0)


class TestBlockedAtInflation:
    def _cm_with_single_obstacle(self) -> Costmap:
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=0.0)
        costmap.update(0.0, 0.0, 1.0)
        return costmap

    def test_zero_inflation_single_cell(self):
        costmap = self._cm_with_single_obstacle()
        blocked = _blocked_at_inflation(costmap, 0.0)
        assert blocked == {(0, 0)}

    def test_larger_inflation_includes_neighbours(self):
        costmap = self._cm_with_single_obstacle()
        blocked_0 = _blocked_at_inflation(costmap, 0.0)
        blocked_2 = _blocked_at_inflation(costmap, 2.0)
        assert blocked_0.issubset(blocked_2)
        assert (1, 0) in blocked_2
        assert (0, 1) in blocked_2
        assert (2, 2) not in blocked_2  # sqrt(8) ≈ 2.83 > 2

    def test_below_height_threshold_ignored(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.5, inflation_radius=0.0)
        costmap.update(0.0, 0.0, 0.3)  # below threshold
        costmap.update(5.0, 0.0, 1.0)  # above threshold
        blocked = _blocked_at_inflation(costmap, 0.0)
        assert blocked == {(5, 0)}

    def test_does_not_mutate_costmap(self):
        costmap = Costmap(cell_size=1.0, obstacle_height=0.1, inflation_radius=0.0)
        costmap.update(0.0, 0.0, 1.0)
        assert costmap.inflation_radius == 0.0
        _blocked_at_inflation(costmap, 3.0)
        assert costmap.inflation_radius == 0.0  # unchanged
        # Live costmap's own blocked_cells still reflects its own inflation
        assert costmap.blocked_cells() == {(0, 0)}

    def test_rejects_negative_inflation(self):
        costmap = self._cm_with_single_obstacle()
        with pytest.raises(ValueError):
            _blocked_at_inflation(costmap, -0.5)


class TestStuckEscalation:
    def _initial_state(self, inflation_radius=0.4):
        return StuckState(
            ref_goal_dist=float("inf"),
            last_progress_time=0.0,
            effective_inflation=inflation_radius,
        )

    def _step(
        self,
        state,
        dist,
        now,
        *,
        progress_epsilon=0.25,
        stuck_seconds=5.0,
        stuck_shrink_factor=0.5,
        stuck_min_inflation=0.0,
    ):
        new_state, _ = progress_tick(
            state,
            dist,
            now,
            progress_epsilon=progress_epsilon,
            stuck_seconds=stuck_seconds,
            stuck_shrink_factor=stuck_shrink_factor,
            stuck_min_inflation=stuck_min_inflation,
        )
        return new_state

    def test_progress_refreshes_last_time(self):
        s = self._initial_state()
        s = self._step(s, 10.0, 0.0)
        assert s.ref_goal_dist == 10.0
        s = self._step(s, 9.0, 1.0)
        assert s.last_progress_time == 1.0
        assert s.ref_goal_dist == 9.0
        assert s.effective_inflation == 0.4

    def test_tiny_progress_does_not_count(self):
        s = self._initial_state()
        s = self._step(s, 10.0, 0.0, progress_epsilon=0.25)
        s = self._step(s, 9.9, 1.0, progress_epsilon=0.25)
        assert s.ref_goal_dist == 10.0
        assert s.last_progress_time == 0.0

    def test_escalation_shrinks_inflation(self):
        s = self._initial_state(inflation_radius=0.4)
        kw = dict(stuck_seconds=5.0, stuck_shrink_factor=0.5)
        s = self._step(s, 10.0, 0.0, **kw)
        s = self._step(s, 10.0, 4.9, **kw)
        assert s.effective_inflation == 0.4
        s = self._step(s, 10.0, 5.0, **kw)
        assert s.effective_inflation == 0.2
        s = self._step(s, 10.0, 10.0, **kw)
        assert s.effective_inflation == 0.1

    def test_escalation_respects_floor(self):
        s = self._initial_state(inflation_radius=0.4)
        kw = dict(stuck_seconds=1.0, stuck_shrink_factor=0.5, stuck_min_inflation=0.2)
        s = self._step(s, 10.0, 0.0, **kw)
        s = self._step(s, 10.0, 1.0, **kw)
        assert s.effective_inflation == 0.2
        s = self._step(s, 10.0, 2.0, **kw)
        assert s.effective_inflation == 0.2
        s = self._step(s, 10.0, 3.0, **kw)
        assert s.effective_inflation == 0.2

    def test_cached_path_lookahead_tracks_robot_position(self):
        cached = [(x, 0.0) for x in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
        wx, wy = SimplePlanner._lookahead(cached, 2.0, 0.0, 1.5)
        assert wx == pytest.approx(4.0)
        assert wy == pytest.approx(0.0)

    def test_progress_after_escalation_keeps_shrunk_inflation(self):
        # Once we shrink inflation to clear a tight spot, we DON'T bump
        # it back up on subsequent progress — escalated value stays in
        # force until the next goal arrives.
        s = self._initial_state(inflation_radius=0.4)
        s = self._step(s, 10.0, 0.0, stuck_seconds=1.0)
        s = self._step(s, 10.0, 1.0, stuck_seconds=1.0)
        assert s.effective_inflation == 0.2
        s = self._step(s, 9.0, 1.5, stuck_seconds=1.0)
        assert s.effective_inflation == 0.2
        assert s.ref_goal_dist == 9.0
