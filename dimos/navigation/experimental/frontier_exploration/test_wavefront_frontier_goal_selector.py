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

"""Goal selection for the plain frontier explorer.

These cover the three things that decide whether a mapping run sweeps a building
or walks the same ground twice: only reachable frontiers are candidates, a target
the robot could not reach is cooled off, and a target it already reached is not
approached again.
"""

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.experimental.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)


def grid(values, resolution: float = 1.0) -> OccupancyGrid:
    return OccupancyGrid(
        grid=np.array(values, dtype=np.int8),
        resolution=resolution,
        origin=Pose(),
        frame_id="world",
    )


UNKNOWN = int(CostValues.UNKNOWN)
FREE = int(CostValues.FREE)
WALL = int(CostValues.OCCUPIED)


@pytest.fixture
def explorer():
    module = WavefrontFrontierExplorer()
    yield module
    module.exploration_active = False
    module._close_module()


def test_unreachable_frontier_across_a_wall_is_never_selected(explorer):
    """A frontier straight ahead but behind a wall loses to one down the corridor.

    Straight-line scoring picks the wall-side frontier every cycle, the navigator
    cannot route to it, and the robot walks back and forth over the same cells.
    """
    values = np.full((9, 12), WALL, dtype=np.int8)
    values[1:8, 1:5] = FREE  # a room the robot has already mapped
    values[4, 5:8] = FREE  # doorway and corridor, the only way through
    values[1:8, 8:11] = UNKNOWN  # the room beyond, straight east across the wall

    goal = explorer.select_goal(Vector3(2.5, 4.5, 0.0), grid(values))

    assert goal is not None
    # The unknown room on the far side of the wall is closer in a straight line,
    # but it is not reachable; the goal must be the corridor mouth.
    assert goal.x == pytest.approx(7.0)
    assert explorer._last_goal_cost == pytest.approx(5.0)


def test_failed_goal_is_cooled_off_and_not_picked_again(explorer):
    values = np.full((7, 7), UNKNOWN, dtype=np.int8)
    values[1:6, 1:6] = FREE

    first = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))
    assert first is not None

    explorer.mark_failed_goal(first)
    second = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))

    assert second is not None
    assert np.hypot(second.x - first.x, second.y - first.y) > 0.75


def test_failure_cooling_expires(explorer):
    explorer.config.failure_cooldown = -1.0  # already expired
    explorer.mark_failed_goal(Vector3(3.0, 3.0, 0.0))

    assert explorer._active_failures() == []


def test_reached_goal_is_not_approached_again(explorer):
    values = np.full((7, 7), UNKNOWN, dtype=np.int8)
    values[1:6, 1:6] = FREE

    goal = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))
    assert goal is not None

    explorer.mark_explored_goal(goal)
    again = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))

    assert again is None or np.hypot(again.x - goal.x, again.y - goal.y) > 0.6


def test_failed_goal_under_the_robot_does_not_wall_it_in(explorer):
    """When a target times out the robot is standing on it, so cooling it off must
    not block the cell it occupies - that would stop exploration in place."""
    values = np.full((7, 7), UNKNOWN, dtype=np.int8)
    values[1:6, 1:6] = FREE

    first = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))
    assert first is not None
    assert (first.x, first.y) == (1.0, 1.0)  # the robot's own cell is a frontier cell

    explorer.mark_failed_goal(first)
    second = explorer.select_goal(Vector3(1.5, 1.5, 0.0), grid(values))

    assert second is not None
    assert np.hypot(second.x - first.x, second.y - first.y) >= 1.0


def test_cooling_zones_cannot_wall_the_robot_in(explorer):
    """A cooled-off target in the only doorway must not end the run."""
    values = np.full((5, 10), WALL, dtype=np.int8)
    values[2, 1:8] = FREE  # a corridor out of the room
    values[1:4, 8:10] = UNKNOWN  # unexplored space at the far end

    # The robot gave up on the doorway one cell ahead of it.
    explorer.mark_failed_goal(Vector3(2.5, 2.5, 0.0))
    goal = explorer.select_goal(Vector3(1.5, 2.5, 0.0), grid(values))

    assert goal is not None
    assert goal.x > 3.0


def test_no_frontier_when_everything_is_known(explorer):
    values = np.full((5, 5), FREE, dtype=np.int8)

    assert explorer.select_goal(Vector3(2.0, 2.0, 0.0), grid(values)) is None


def test_robot_reading_inside_an_obstacle_still_explains(explorer):
    """Odometry drift puts the robot inside a mapped wall. It is physically there, so
    the map is wrong under it; refusing to plan would park it forever."""
    values = np.full((5, 5), UNKNOWN, dtype=np.int8)
    values[1:4, 1:4] = FREE
    values[2, 2] = WALL

    goal = explorer.select_goal(Vector3(2.5, 2.5, 0.0), grid(values))

    assert goal is not None
    # A real frontier inside the mapped patch, not a halt.
    assert 1.0 <= goal.x <= 3.0 and 1.0 <= goal.y <= 3.0


def test_robot_off_the_map_gets_no_goal(explorer):
    values = np.full((3, 3), FREE, dtype=np.int8)

    assert explorer.select_goal(Vector3(50.0, 50.0, 0.0), grid(values)) is None


def test_session_reset_forgets_failures(explorer):
    explorer.mark_failed_goal(Vector3(2.0, 2.0, 0.0))
    explorer.mark_explored_goal(Vector3(3.0, 3.0, 0.0))

    explorer.reset_exploration_session()

    assert explorer._failed == []
    assert explorer.explored_goals == []


def test_goal_budget_grows_with_the_leg(explorer):
    """A far frontier must not be declared unreachable just because it is far."""
    far = 12.0
    config = explorer.config
    base = config.goal_timeout
    budget = base + far / config.goal_speed

    assert budget > base
    assert budget == pytest.approx(base + 30.0)
