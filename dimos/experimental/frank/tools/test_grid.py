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

"""grid.py against a hand-built map, so it can be checked without the robot.

    uv run pytest dimos/experimental/frank/tools/test_grid.py

The map is a 10 x 10 m room at 5 cm cells with origin (0, 0): a wall across x = 6 m, and
everything past it unknown. The robot stands at (2, 2) facing +x.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))

import grid

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


@pytest.fixture
def room() -> grid.Space:
    cells = np.zeros((200, 200), dtype=np.int8)  # free
    cells[:, 120:124] = 100  # wall at x = 6.0 .. 6.2 m
    cells[:, 124:] = -1  # unknown behind it
    og = OccupancyGrid(grid=cells, resolution=0.05, origin=Pose(position=[0.0, 0.0, 0.0]))
    return grid.Space(og, 2.0, 2.0, 0.0)


def test_at(room: grid.Space) -> None:
    assert room.at(2.0, 2.0) == "free"
    assert room.at(6.1, 2.0) == "occupied"
    assert room.at(8.0, 2.0) == "unknown"
    assert room.at(-1.0, 2.0) == "outside map"


def test_ray_hits_the_wall_ahead(room: grid.Space) -> None:
    dist, why = room.ray(0)
    assert why == "wall"
    assert dist == pytest.approx(4.0, abs=0.1)  # 2.0 -> 6.0


def test_ray_relative_is_measured_from_the_nose(room: grid.Space) -> None:
    facing_north = grid.Space(room.og, 2.0, 2.0, 90.0)
    assert facing_north.ray(-90, relative=True) == room.ray(0)


def test_ray_leaves_the_map(room: grid.Space) -> None:
    dist, why = room.ray(180)
    assert why == "edge of map"
    assert dist == pytest.approx(2.0, abs=0.1)


def test_look_is_eight_rays_starting_ahead(room: grid.Space) -> None:
    out = room.look()
    assert [name for name, _, _ in out] == [name for name, _ in grid.DIRECTIONS]
    assert dict((name, why) for name, _, why in out)["ahead"] == "wall"


def test_nearest_free_backs_off_the_wall(room: grid.Space) -> None:
    x, y, clearance = room.nearest_free(6.5, 2.0)
    assert x < 6.0 - grid.ROBOT_RADIUS + 0.1  # outside the inflated wall
    assert clearance >= grid.ROBOT_RADIUS


def test_reachable_this_side_of_the_wall(room: grid.Space) -> None:
    ok, length = room.reachable(4.0, 5.0)
    assert ok
    assert length == pytest.approx(math.hypot(2.0, 3.0), abs=0.6)


def test_unknown_is_not_reachable(room: grid.Space) -> None:
    """The point is behind the wall; the nearest cell the robot fits in is in front of it."""
    ok, length = room.reachable(9.0, 2.0)
    assert ok and length < 5.0
    assert room.at(9.0, 2.0) == "unknown"


def test_open_spots_are_in_the_free_half(room: grid.Space) -> None:
    spots = room.open_spots(3)
    assert spots
    for x, y, radius, _ in spots:
        assert room.at(x, y) == "free"
        assert x < 6.0
        assert radius >= grid.ROBOT_RADIUS
    assert [s[3] for s in spots] == sorted(s[3] for s in spots)


def test_front_of_a_pose(room: grid.Space) -> None:
    x, y = room.front(4.0, 2.0, 180.0, 1.5)
    assert (x, y) == pytest.approx((2.5, 2.0), abs=1e-6)


def test_render_writes_a_png(room: grid.Space, tmp_path: Path) -> None:
    out = room.render(str(tmp_path / "map.png"))
    assert Path(out).stat().st_size > 1000


@pytest.fixture
def doorway() -> grid.Space:
    """Same room, but the wall has a 1.5 m gap and the unknown starts right behind it."""
    cells = np.zeros((200, 200), dtype=np.int8)
    cells[:, 120:124] = 100
    cells[80:110, 120:124] = 0  # gap at y = 4.0 .. 5.5 m
    cells[:, 124:] = -1
    og = OccupancyGrid(grid=cells, resolution=0.05, origin=Pose(position=[0.0, 0.0, 0.0]))
    return grid.Space(og, 2.0, 2.0, 0.0)


def test_no_frontier_behind_a_solid_wall(room: grid.Space) -> None:
    assert room.frontiers() == []


def test_frontier_is_the_gap_in_the_wall(doorway: grid.Space) -> None:
    edges = doorway.frontiers()
    assert len(edges) == 1
    x, y, width, length, where = edges[0]
    assert 5.4 < x < 6.3
    assert 4.0 < y < 5.6
    assert width == pytest.approx(1.5, abs=0.2)
    assert length > 4.0
    assert where == "ahead-left"


def test_direction_words(doorway: grid.Space) -> None:
    assert doorway.direction(5.0, 2.0) == "ahead"
    assert doorway.direction(2.0, 5.0) == "left"
    assert doorway.direction(-1.0, 2.0) == "behind"
    assert doorway.direction(2.0, -1.0) == "right"


def test_path_clear_across_the_room(room: grid.Space) -> None:
    verdict, length, tightest, stop, _ = room.path(5.0, 2.0)
    assert verdict == "clear"
    assert length == pytest.approx(3.0, abs=0.1)
    assert stop is None
    assert tightest > 0.5


def test_path_blocked_by_the_wall(room: grid.Space) -> None:
    verdict, _, _, stop, why = room.path(8.0, 2.0)
    assert verdict == "blocked"
    assert stop == pytest.approx(3.65, abs=0.15)  # wall at 6.0 minus the robot's half-width
    assert "wall" in why


def test_path_through_the_gap_is_unseen_beyond_it(doorway: grid.Space) -> None:
    facing_gap = grid.Space(doorway.og, 2.0, 4.75, 0.0)
    verdict, _, _, stop, why = facing_gap.path(8.0, 4.75)
    assert verdict == "unseen"
    assert stop == pytest.approx(4.2, abs=0.3)  # unknown starts at x = 6.2
    assert "unscanned" in why


def test_path_tight_near_a_wall(room: grid.Space) -> None:
    near = grid.Space(room.og, 2.0, 2.0, 0.0)
    verdict, *_ = near.path(5.6, 2.0)  # ends 0.4 m from the wall, inside the tight band
    assert verdict == "tight"


@pytest.fixture
def standing_in_the_dark(room: grid.Space) -> grid.Space:
    """The robot's own footprint reads unknown, as it does on the real Go2."""
    cells = room.og.grid.copy()
    cells[34:46, 34:46] = -1  # 0.6 m square around (2, 2)
    og = OccupancyGrid(grid=cells, resolution=0.05, origin=Pose(position=[0.0, 0.0, 0.0]))
    return grid.Space(og, 2.0, 2.0, 0.0)


def test_own_footprint_counts_as_floor(standing_in_the_dark: grid.Space) -> None:
    s = standing_in_the_dark
    assert s.at(2.0, 2.0) == "free"
    verdict, length, _, stop, _ = s.path(4.0, 2.0)
    assert verdict == "clear" and stop is None
    dist, why = s.ray(0)
    assert why == "wall" and dist == pytest.approx(4.0, abs=0.1)
    assert not any(length < 0.5 for _, _, _, length, _ in s.frontiers(5))


def test_crop_draws_the_leg(room: grid.Space, tmp_path: Path) -> None:
    out = tmp_path / "leg_map.png"
    assert room.crop(str(out), 3.0, (4.0, 2.0)) == str(out)
    assert out.stat().st_size > 1000
    assert room.crop(str(tmp_path / "around.png")) == str(tmp_path / "around.png")


def test_check_leg_faces_the_leg_first(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    import leg
    import robot

    turns: list[float] = []

    def fake_turn(deg: float) -> str:
        turns.append(deg)
        return f"turned {deg:+.0f} deg of the {deg:+.0f} asked"

    monkeypatch.setattr(robot, "pose", lambda: (2.0, 2.0, 0.0))
    monkeypatch.setattr(robot, "turn", fake_turn)
    monkeypatch.setattr(robot, "_dimos", lambda: None)
    monkeypatch.setattr(robot, "close", lambda: None)
    monkeypatch.setattr(robot, "MOTION_OFF", tmp_path / "MOTION_OFF")
    assert leg.face(4.0, 2.5) is None  # 14 deg off: the photo covers it
    assert turns == []
    assert leg.face(2.0, 4.0) == "turned +90 deg of the +90 asked (to face the leg first)"
    assert turns == [90.0]
    (tmp_path / "MOTION_OFF").touch()
    assert "motion is off" in (leg.face(0.0, 2.0) or "")
    assert len(turns) == 1
