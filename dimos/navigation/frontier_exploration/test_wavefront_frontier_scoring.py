"""Scoring tests for the direction momentum term of the wavefront selector.

Three candidate frontiers are placed at the same distance from the robot, with
the same cluster size, on an empty map with no explored goals, so every term of
the score except direction momentum is identical between them: one straight
ahead of the current exploration direction, one sideways, one straight behind.
"""

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontConfig,
    WavefrontFrontierExplorer,
)

RESOLUTION = 0.05
FRONTIER_SIZE = 40

ROBOT = Vector3(5.0, 5.0, 0.0)
AHEAD = Vector3(7.0, 5.0, 0.0)  # dot product +1 with the exploration direction
SIDEWAYS = Vector3(5.0, 7.0, 0.0)  # dot product 0
BEHIND = Vector3(3.0, 5.0, 0.0)  # dot product -1


def open_costmap() -> OccupancyGrid:
    """A 10 m x 10 m free map, no obstacle within any candidate's safe radius."""
    grid = np.full((200, 200), CostValues.FREE, dtype=np.int8)
    return OccupancyGrid(grid=grid, resolution=RESOLUTION, frame_id="world")


def score_candidates(**config_kwargs) -> dict[str, float]:
    """Score the three candidates with a selector heading in +x."""
    explorer = WavefrontFrontierExplorer.__new__(WavefrontFrontierExplorer)
    explorer.config = WavefrontConfig(**config_kwargs)
    explorer.explored_goals = []
    explorer.exploration_direction = Vector3(1.0, 0.0, 0.0)

    costmap = open_costmap()
    return {
        name: explorer._compute_comprehensive_frontier_score(
            frontier, FRONTIER_SIZE, ROBOT, costmap
        )
        for name, frontier in (("ahead", AHEAD), ("sideways", SIDEWAYS), ("behind", BEHIND))
    }


def test_signed_momentum_makes_a_u_turn_cost():
    scores = score_candidates()
    weight = WavefrontConfig().momentum_weight

    assert scores["ahead"] > scores["sideways"] > scores["behind"]
    # The candidates differ only in direction, so the whole gap is the momentum
    # term: +weight straight ahead against -weight straight behind.
    assert scores["ahead"] - scores["behind"] == pytest.approx(2 * weight)


def test_clamped_momentum_prices_a_u_turn_like_a_sideways_move():
    """The previous behavior, still reachable through the config."""
    scores = score_candidates(min_momentum_score=0.0)

    assert scores["behind"] == scores["sideways"]
    assert scores["ahead"] > scores["behind"]


def test_momentum_weight_scales_the_direction_gap():
    quiet = score_candidates(momentum_weight=0.0)
    loud = score_candidates(momentum_weight=0.5)

    assert quiet["ahead"] == quiet["behind"]
    assert loud["ahead"] - loud["behind"] == pytest.approx(1.0)


def test_zeroed_direction_neutralizes_momentum_after_a_timeout():
    """The timeout branch zeroes exploration_direction, so the ranking right
    after a timeout must carry no directional preference at all."""
    explorer = WavefrontFrontierExplorer.__new__(WavefrontFrontierExplorer)
    explorer.config = WavefrontConfig()
    explorer.explored_goals = []
    explorer.exploration_direction = Vector3(0.0, 0.0, 0.0)

    costmap = open_costmap()
    ahead = explorer._compute_comprehensive_frontier_score(AHEAD, FRONTIER_SIZE, ROBOT, costmap)
    behind = explorer._compute_comprehensive_frontier_score(BEHIND, FRONTIER_SIZE, ROBOT, costmap)
    assert ahead == pytest.approx(behind)
