"""Scoring tests for the direction momentum term of the wavefront selector.

Three candidate frontiers are placed at the same distance from the robot, with
the same cluster size, on an empty map with no explored goals, so every term of
the score except direction momentum is identical between them: one straight
ahead of the current exploration direction, one sideways, one straight behind.

The explorer is built with its real constructor, with the RPC transport
replaced by a no-op (Module.__init__ starts the transport and serves the module
RPC on construction) and never started.
"""

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontConfig,
    WavefrontFrontierExplorer,
)
from dimos.protocol.rpc.spec import RPCSpec

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


class _NoRPC(RPCSpec):
    """RPC transport that serves nothing: the scoring tests never call RPCs."""

    def __init__(self, **kwargs) -> None:
        pass

    def serve_rpc(self, f, name):
        return lambda: None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


@pytest.fixture
def make_explorer():
    """Build real explorers on the no-op RPC transport, never started, stopped at teardown."""
    made: list[WavefrontFrontierExplorer] = []

    def _make(**config_kwargs) -> WavefrontFrontierExplorer:
        explorer = WavefrontFrontierExplorer(rpc_transport=_NoRPC, **config_kwargs)
        made.append(explorer)
        return explorer

    yield _make
    for explorer in made:
        explorer.stop()


def score(explorer: WavefrontFrontierExplorer, frontier: Vector3) -> float:
    return explorer._compute_comprehensive_frontier_score(
        frontier, FRONTIER_SIZE, ROBOT, open_costmap()
    )


def score_candidates(explorer: WavefrontFrontierExplorer) -> dict[str, float]:
    """Score the three candidates with the explorer heading in +x."""
    explorer._update_exploration_direction(ROBOT, AHEAD)
    return {
        name: score(explorer, frontier)
        for name, frontier in (("ahead", AHEAD), ("sideways", SIDEWAYS), ("behind", BEHIND))
    }


def test_default_weight_is_the_original_five_percent():
    """The signed term changes the price of a U-turn, not the weight of momentum."""
    assert WavefrontConfig().momentum_weight == pytest.approx(0.05)
    assert WavefrontConfig().min_momentum_score == pytest.approx(-1.0)


def test_signed_momentum_makes_a_u_turn_cost(make_explorer):
    scores = score_candidates(make_explorer())
    weight = WavefrontConfig().momentum_weight

    assert scores["ahead"] > scores["sideways"] > scores["behind"]
    # The candidates differ only in direction, so the whole gap is the momentum
    # term: +weight straight ahead against -weight straight behind, 0.10 by
    # default where the clamped score gave 0.05.
    assert scores["ahead"] - scores["behind"] == pytest.approx(2 * weight)
    assert scores["ahead"] - scores["behind"] == pytest.approx(0.10)


def test_clamped_momentum_prices_a_u_turn_like_a_sideways_move(make_explorer):
    """The previous score, still reachable through the config."""
    scores = score_candidates(make_explorer(min_momentum_score=0.0))

    assert scores["behind"] == scores["sideways"]
    assert scores["ahead"] > scores["behind"]


def test_momentum_weight_scales_the_direction_gap(make_explorer):
    quiet = score_candidates(make_explorer(momentum_weight=0.0))
    loud = score_candidates(make_explorer(momentum_weight=0.5))

    assert quiet["ahead"] == quiet["behind"]
    assert loud["ahead"] - loud["behind"] == pytest.approx(1.0)


def test_goal_timeout_neutralizes_momentum_until_the_next_goal(make_explorer):
    """The timeout path forgets the stale direction; the next chosen goal
    sets a new one. Uses the same methods the exploration loop calls."""
    explorer = make_explorer()

    explorer._update_exploration_direction(ROBOT, AHEAD)
    assert score(explorer, AHEAD) > score(explorer, BEHIND)

    explorer._on_goal_timeout()
    assert score(explorer, AHEAD) == pytest.approx(score(explorer, BEHIND))

    explorer._update_exploration_direction(ROBOT, BEHIND)
    assert score(explorer, BEHIND) > score(explorer, AHEAD)


def test_goal_timeout_in_the_loop_forgets_the_direction(make_explorer, monkeypatch):
    """_await_goal_outcome is what the exploration loop calls after publishing
    a goal: a wait that times out must forget the direction, a wait that
    returns because the goal was reached must keep it."""
    explorer = make_explorer(goal_timeout=7.0)
    seen: list[float] = []

    def timed_out(timeout=None):
        seen.append(timeout)
        return False

    def reached(timeout=None):
        seen.append(timeout)
        return True

    explorer._update_exploration_direction(ROBOT, AHEAD)
    monkeypatch.setattr(explorer.goal_reached_event, "wait", timed_out)
    assert explorer._await_goal_outcome() is False
    assert explorer.exploration_direction.x == 0.0 and explorer.exploration_direction.y == 0.0

    explorer._update_exploration_direction(ROBOT, AHEAD)
    monkeypatch.setattr(explorer.goal_reached_event, "wait", reached)
    assert explorer._await_goal_outcome() is True
    assert explorer.exploration_direction.x == pytest.approx(1.0)

    assert seen == [7.0, 7.0]  # the configured goal_timeout, both times
