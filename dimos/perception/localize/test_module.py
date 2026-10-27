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

from dimos.perception.localize.association import Groups
from dimos.perception.localize.module import best_candidate, footprint, scene_around


def _group(groups: Groups, *sightings: tuple[float, float, tuple[float, float, float]]) -> None:
    """One object seen at (ts, score, camera position) sightings, centred where it was seen."""
    groups.centers.append(np.zeros(3))
    groups.members.append(
        [v for ts, score, cam in sightings for v in (ts, score, 0.5, 0.1, ts, *cam)]
    )


def test_best_candidate_is_the_top_scoring_group_seen_in_the_window() -> None:
    groups = Groups()
    _group(groups, (10.0, 0.21, (0, 0, 1)), (12.0, 0.18, (0.1, 0, 1)))
    _group(groups, (11.0, 0.15, (0, 0, 1)))
    _group(groups, (2.0, 0.9, (0, 0, 1)))  # a stronger sighting from before the window
    score, views, (x, y, z) = best_candidate(groups, 5.0, 15.0) or (0.0, 0, (0, 0, 0))
    assert score == pytest.approx(0.21)
    assert views == 2
    assert (x, y, z) == pytest.approx((0.5, 0.1, 12.0))  # the newest sighting's position


def test_no_candidate_without_a_lifted_sighting_in_the_window() -> None:
    assert best_candidate(None, 0.0, 10.0) is None
    assert best_candidate(Groups(), 0.0, 10.0) is None
    groups = Groups()
    _group(groups, (2.0, 0.9, (0, 0, 1)))
    assert best_candidate(groups, 5.0, 15.0) is None


def _box_cloud(
    length: float, width: float, yaw_deg: float, centre: tuple[float, float]
) -> np.ndarray:
    """A dense top face, z 0.95, over a short side wall down to 0.91."""
    u, v = np.meshgrid(
        np.linspace(-length / 2, length / 2, 60), np.linspace(-width / 2, width / 2, 30)
    )
    yaw = np.radians(yaw_deg)
    x = centre[0] + u * np.cos(yaw) - v * np.sin(yaw)
    y = centre[1] + u * np.sin(yaw) + v * np.cos(yaw)
    top = np.stack([x.ravel(), y.ravel(), np.full(x.size, 0.95)], axis=1)
    wall = top[: len(top) // 3].copy()
    wall[:, 2] = np.linspace(0.91, 0.95, len(wall))
    return np.concatenate([top, wall])


@pytest.mark.parametrize("yaw", [30.0, -60.0, 0.0])
def test_footprint_measures_a_rotated_box(yaw: float) -> None:
    box = footprint(_box_cloud(0.30, 0.10, yaw, (0.5, -0.1)))
    assert box is not None
    (cx, cy), (length, width), measured, (bottom, top) = box
    assert (cx, cy) == pytest.approx((0.5, -0.1), abs=0.01)
    assert (length, width) == pytest.approx((0.30, 0.10), abs=0.02)
    assert measured == pytest.approx(yaw, abs=2.0)
    assert (bottom, top) == pytest.approx((0.91, 0.95), abs=0.005)


def test_footprint_needs_points() -> None:
    assert footprint(np.zeros((2, 3))) is None


def test_scene_around_keeps_the_surroundings_and_drops_the_object() -> None:
    table = np.stack(
        np.meshgrid(np.linspace(0.0012, 1.0012, 401), np.linspace(-0.4988, 0.5012, 401)), -1
    ).reshape(-1, 2)
    table = np.column_stack([table, np.full(len(table), 0.80)])
    box = _box_cloud(0.10, 0.06, 0.0, (0.5, 0.0))
    around = scene_around(np.concatenate([table, box]), box, margin=0.1)
    # Inside the margin box only, nothing within 1 cm of the object, thinned to 5 mm cells.
    assert np.all(np.abs(around[:, 0] - 0.5) <= 0.15 + 1e-6)
    assert np.all(np.abs(around[:, 1]) <= 0.13 + 1e-6)
    assert not np.any(around[:, 2] > 0.85)  # the box's own surfaces are gone, the table stays
    assert len(around) > 100
    # A 2.5 mm grid thinned to one point per 5 mm cell keeps about a quarter of it.
    assert len(around) < 0.3 * np.sum(
        np.all(np.abs(table[:, :2] - [0.5, 0.0]) <= [0.15, 0.13], axis=1)
    )


def test_scene_around_with_nothing_near_is_empty() -> None:
    box = _box_cloud(0.10, 0.06, 0.0, (0.5, 0.0))
    assert scene_around(np.asarray([[3.0, 3.0, 0.8]]), box, margin=0.1).shape == (0, 3)
