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

import pytest

from dimos.teleop.memory_world.visual_search import Place, cluster_places


def place(x: float, y: float, similarity: float, source_id: int = 0) -> Place:
    return Place(position=(x, y, 0.0), similarity=similarity, source_id=source_id, ts=0.0)


def test_nearby_hits_collapse_to_the_strongest_one() -> None:
    """A robot staring at one object makes many frames; that is still one place."""
    places = cluster_places(
        [place(0.0, 0.0, 0.30), place(0.5, 0.0, 0.42), place(1.0, 0.4, 0.35)],
        radius=2.0,
        max_places=6,
    )
    assert [p.similarity for p in places] == [0.42]


def test_distant_hits_are_separate_places() -> None:
    places = cluster_places(
        [place(0.0, 0.0, 0.30), place(10.0, 0.0, 0.42), place(0.0, 20.0, 0.10)],
        radius=2.0,
        max_places=6,
    )
    assert [p.position for p in places] == [(10.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 20.0, 0.0)]


def test_results_are_ordered_by_similarity_regardless_of_input_order() -> None:
    places = cluster_places(
        [place(0.0, 0.0, 0.1), place(10.0, 0.0, 0.9), place(20.0, 0.0, 0.5)],
        radius=2.0,
        max_places=6,
    )
    assert [p.similarity for p in places] == [0.9, 0.5, 0.1]


def test_max_places_caps_the_answer() -> None:
    candidates = [place(float(i) * 10.0, 0.0, 1.0 - i * 0.01) for i in range(20)]
    places = cluster_places(candidates, radius=2.0, max_places=3)
    assert len(places) == 3
    assert [p.similarity for p in places] == pytest.approx([1.0, 0.99, 0.98])


def test_radius_boundary_is_inclusive_of_exactly_radius_apart() -> None:
    """Two hits exactly `radius` apart are distinct, so the cap is a true minimum spacing."""
    places = cluster_places([place(0.0, 0.0, 0.9), place(2.0, 0.0, 0.8)], radius=2.0, max_places=6)
    assert len(places) == 2


def test_clustering_uses_all_three_axes() -> None:
    """Two floors of a stairwell are different places even at the same x/y."""
    stacked = [
        Place(position=(0.0, 0.0, 0.0), similarity=0.9, source_id=1, ts=0.0),
        Place(position=(0.0, 0.0, 5.0), similarity=0.8, source_id=2, ts=0.0),
    ]
    assert len(cluster_places(stacked, radius=2.0, max_places=6)) == 2


def test_no_candidates_yields_no_places() -> None:
    assert cluster_places([], radius=2.0, max_places=6) == []


@pytest.mark.parametrize(("radius", "max_places"), [(0.0, 6), (-1.0, 6), (2.0, 0), (2.0, -1)])
def test_invalid_parameters_are_rejected(radius: float, max_places: int) -> None:
    with pytest.raises(ValueError):
        cluster_places([place(0.0, 0.0, 0.5)], radius=radius, max_places=max_places)
