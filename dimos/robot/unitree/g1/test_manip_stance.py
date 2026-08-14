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

import math

import numpy as np
import pytest

from dimos.robot.unitree.g1.manip_stance import (
    DEFAULT_SPOUT_OFFSET_IN_PALM,
    POUR_Z,
    PourReachMap,
    Stance,
    palm_position_for_spout,
    palm_to_capability_tcp,
    pot_in_base_frame,
    select_stance,
    tool_yaw_for,
    wrap_angle,
)


def _map(reachable, cell=0.05, x0=0.0, y0=-0.2):
    """A reach map with a hand-written reachable region."""
    grid = [[int(v) for v in row] for row in reachable]
    return PourReachMap(
        {
            "pour_z": POUR_Z,
            "tip_radians": -math.pi / 2,
            "spout_offset_in_palm": DEFAULT_SPOUT_OFFSET_IN_PALM,
            "cell": cell,
            "x0": x0,
            "y0": y0,
            "ik_upright": grid,
            "ik_tipped": grid,
        }
    )


def test_stance_puts_the_pot_at_the_chosen_offset():
    reach = _map([[0] * 5, [0, 1, 1, 1, 0], [0, 1, 1, 1, 0], [0, 1, 1, 1, 0], [0] * 5])
    for approach in (0.0, 1.1, -2.4, math.pi):
        stance = select_stance((1.4, -0.35), approach_yaw=approach, reach_map=reach, margin_cells=1)
        seen = pot_in_base_frame((1.4, -0.35), stance.xy, stance.yaw)
        assert seen == pytest.approx(stance.offset, abs=1e-9)


def test_stance_offset_is_reachable_with_margin():
    reach = PourReachMap.load()
    stance = select_stance((1.0, -0.15), approach_yaw=0.3, reach_map=reach)
    assert reach.contains(stance.offset, margin_cells=3)
    assert stance.margin_cells >= 3
    # A stance the robot walks to should be a stride away from the pot, not
    # on top of it or across the room.
    assert 0.2 < math.dist(stance.xy, (1.0, -0.15)) < 0.8


def test_only_cells_reachable_in_both_pour_poses_count():
    upright = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]
    tipped = [[0, 0, 0], [0, 1, 1], [0, 1, 1]]
    reach = PourReachMap(
        {
            "pour_z": POUR_Z,
            "tip_radians": -math.pi / 2,
            "spout_offset_in_palm": DEFAULT_SPOUT_OFFSET_IN_PALM,
            "cell": 0.05,
            "x0": 0.0,
            "y0": 0.0,
            "ik_upright": upright,
            "ik_tipped": tipped,
        }
    )
    assert reach.margin((0.05, 0.05)) == 1
    assert reach.margin((0.0, 0.0)) == 0


def test_margin_counts_cells_from_the_edge():
    reachable = np.zeros((7, 7), dtype=int)
    reachable[1:6, 1:6] = 1
    reach = _map(reachable, cell=0.1, x0=0.0, y0=0.0)
    assert reach.margin((0.3, 0.3)) == 3  # centre of a 5x5 block
    assert reach.margin((0.2, 0.3)) == 2
    assert reach.margin((0.1, 0.3)) == 1  # edge of the region
    assert reach.margin((0.0, 0.3)) == 0
    assert reach.contains((0.3, 0.3), margin_cells=3)
    assert not reach.contains((0.1, 0.3), margin_cells=2)


def test_offset_outside_the_grid_is_not_reachable():
    reach = _map([[1, 1], [1, 1]])
    assert reach.margin((5.0, 5.0)) == 0
    assert not reach.contains((-3.0, 0.0))


def test_best_offset_only_applies_a_facing_cone_when_requested():
    # Reachability is authoritative by default, while callers can still ask
    # for a strict facing policy when their task requires it.
    reach = _map([[1, 1, 1], [1, 1, 1], [1, 1, 1]], cell=0.05, x0=-0.6, y0=-0.05)
    assert reach.best_offset(margin_cells=1)[0] < 0.0
    with pytest.raises(ValueError, match="straight ahead"):
        reach.best_offset(margin_cells=1, max_bearing=math.radians(35.0))


def test_pot_in_base_frame_is_the_inverse_of_standing_there():
    pot = (2.3, -1.1)
    for yaw in (0.0, 0.7, -1.9, 3.0):
        base = (1.5, -0.4)
        x, y = pot_in_base_frame(pot, base, yaw)
        # Rotating the base-frame sighting back into the world recovers the pot.
        back_x = base[0] + math.cos(yaw) * x - math.sin(yaw) * y
        back_y = base[1] + math.sin(yaw) * x + math.cos(yaw) * y
        assert (back_x, back_y) == pytest.approx(pot, abs=1e-9)


def test_tool_points_at_the_pot():
    stance = Stance(x=0.0, y=0.0, yaw=0.4, offset=(0.35, -0.15), margin_cells=3)
    yaw = tool_yaw_for(stance.offset, stance.yaw)
    assert yaw == pytest.approx(0.4 + math.atan2(-0.15, 0.35))


def test_wrap_angle_folds_into_a_single_turn():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(-0.3) == pytest.approx(-0.3)
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert wrap_angle(-1.5 * math.pi) == pytest.approx(0.5 * math.pi)


def test_palm_to_capability_tcp_shifts_along_the_hand():
    # Straight out along +x with no rotation: the grasp centre is further out
    # along the hand than the palm frame, never behind it.
    position, rotation = palm_to_capability_tcp(np.array([0.4, 0.0, 0.9]), np.eye(3))
    assert position[0] > 0.4
    assert np.allclose(rotation, np.eye(3))
    # Turned 90 deg, the same offset has to follow the hand round.
    turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    turned, _ = palm_to_capability_tcp(np.array([0.4, 0.0, 0.9]), turn)
    assert turned[1] > 0.0
    assert turned[0] == pytest.approx(0.4 - (position[1] - 0.0), abs=1e-9)


def test_committed_map_matches_the_geometry_the_demo_commands():
    reach = PourReachMap.load()
    assert reach.pour_z == pytest.approx(POUR_Z)
    assert reach.tip_radians == pytest.approx(-math.pi / 2)
    assert reach.spout_offset_in_palm == pytest.approx(DEFAULT_SPOUT_OFFSET_IN_PALM)
    assert reach.reachable.any()
    offset = reach.best_offset(margin_cells=3)
    assert reach.contains(offset, margin_cells=3)


def test_tipped_spout_tcp_moves_the_sampled_palm_above_the_water_exit():
    rotation = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])

    palm = palm_position_for_spout(
        np.array([0.4, -0.2, POUR_Z]),
        rotation,
        DEFAULT_SPOUT_OFFSET_IN_PALM,
    )

    assert palm == pytest.approx([0.4, -0.2, POUR_Z + 0.20])


def test_a_map_sampled_for_a_different_pour_is_refused():
    # Silently standing by a map built for another pour height would only
    # surface as an unexplained planning failure after the robot has walked.
    stale = {
        "pour_z": POUR_Z + 0.2,
        "tip_radians": -math.pi / 2,
        "spout_offset_in_palm": DEFAULT_SPOUT_OFFSET_IN_PALM,
        "cell": 0.05,
        "x0": 0.0,
        "y0": 0.0,
        "ik_upright": [[1, 1], [1, 1]],
        "ik_tipped": [[1, 1], [1, 1]],
    }
    with pytest.raises(ValueError, match="stale"):
        PourReachMap(stale)
    with pytest.raises(ValueError, match="stale"):
        PourReachMap({**stale, "pour_z": POUR_Z, "tip_radians": 0.0})
    with pytest.raises(ValueError, match="stale"):
        PourReachMap(
            {**stale, "pour_z": POUR_Z},
            expected_spout_offset_in_palm=(0.0, 0.18, 0.0),
        )


def test_a_map_without_tcp_metadata_is_refused():
    with pytest.raises(ValueError, match="no spout_offset_in_palm"):
        PourReachMap(
            {
                "pour_z": POUR_Z,
                "tip_radians": -math.pi / 2,
                "cell": 0.05,
                "x0": 0.0,
                "y0": 0.0,
                "ik_upright": [[1]],
                "ik_tipped": [[1]],
            }
        )


def test_a_malformed_grid_is_refused():
    with pytest.raises(ValueError, match="matching 2D pair"):
        PourReachMap(
            {
                "pour_z": POUR_Z,
                "tip_radians": -math.pi / 2,
                "spout_offset_in_palm": DEFAULT_SPOUT_OFFSET_IN_PALM,
                "cell": 0.05,
                "x0": 0.0,
                "y0": 0.0,
                "ik_upright": [[1, 1]],
                "ik_tipped": [[1, 1], [1, 1]],
            }
        )
