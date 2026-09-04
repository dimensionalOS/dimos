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

from pathlib import Path
from typing import Any

import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.suites.pointcloud.lib import generate
from dimos.evals.suites.pointcloud.lib.scorers import coord_list, matched_set
from dimos.evals.types import EvalCase, Outcome


def test_coord_list_reads_points_or_the_word_for_none() -> None:
    assert coord_list("9.0,4.1,+0.25\n(1, 2)") == [(9.0, 4.1, 0.25), (1.0, 2.0)]
    assert coord_list("0 areas at all: none") == []  # one number a line is prose
    assert coord_list("the floor is level") == []
    with pytest.raises(ValueError):
        coord_list("somewhere over there")


def test_matched_set_is_f1_over_paired_points() -> None:
    score = matched_set(0.5)
    assert score([], []) == 1.0  # none, correctly
    assert score([(0.0, 0.0)], []) == 0.0 and score([], [(0.0, 0.0)]) == 0.0
    assert score([(0.0, 0.0), (5.0, 5.0)], [(0.1, 0.0), (5.0, 5.2)]) == 1.0
    assert score([(0.0, 0.0), (5.0, 5.0)], [(0.1, 0.0)]) == pytest.approx(2 / 3)  # a miss
    assert score([(0.0, 0.0)], [(0.1, 0.0), (9.0, 9.0)]) == pytest.approx(2 / 3)  # an extra
    assert score([(0.0, 0.0)], [(0.0, 0.0), (0.1, 0.1)]) == pytest.approx(2 / 3), "no reuse"
    valued = matched_set(0.5, value_band=1.0)
    assert valued([(0.0, 0.0, 2.0)], [(0.0, 0.0, 2.5)]) == 0.5  # right place, half the rise


def test_rows_become_cases_with_coords_and_fused_context(tmp_path: Path) -> None:
    """The point-cloud reader adds the coords type and the fused context entry
    on top of the shared reader; a numeric row still grades as there."""

    def row(**fields: Any) -> generate.Row:
        return {"id": fields["id"], "family": "f", "q": "?", "dataset": "unused", **fields}

    def score(case: EvalCase, answer: str) -> float:
        trajectory = TrajectoryBuilder("?", name="fake", model="fake")
        trajectory.step(message=answer, request=tmp_path / "r", response=tmp_path / "s")
        return case.grade(Outcome(trajectory=trajectory.build("answer"), artifacts={}))

    coords, numeric, fused = generate.cases(
        [
            row(id="c", type="coords", a=[[1.0, 2.0]], radius=0.5, context=[["lidar", [0, 1]]]),
            row(id="n", type="numeric", a=3.0, band=1.0, context=[["lidar", [0, 1]]]),
            row(
                id="f",
                type="numeric",
                a=1.0,
                band=1.0,
                context=[["lidar", [0, 60], {"downsample": 6, "voxel_size": 0.1}]],
            ),
        ],
        tags=frozenset({"pointcloud"}),
    )
    assert coords.tags == {"pointcloud", "f", "coords"}
    assert score(coords, "1.1, 2.0") == 1.0 and score(coords, "none") == 0.0
    assert score(coords, "no idea") == 0.0  # unreadable: wrong, not broken
    assert score(numeric, "about 3.5") == 0.5
    assert len(fused.environment.select) == 1  # the fuse entry was accepted, lazily
