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
from dimos.evals.suites.lib import generate
from dimos.evals.types import EvalCase, Outcome, Trajectory


def _trajectory(answer: str, raw: Path) -> Trajectory:
    trajectory = TrajectoryBuilder("?", name="fake", model="fake")
    trajectory.step(message=answer, request=raw / "r", response=raw / "s")
    return trajectory.build("answer")


def test_generated_rows_become_cases(dataset: str, tmp_path: Path) -> None:
    """Every row type grades its reply; an unreadable reply is 0, not an error.
    Family, type and split are tags; the context selects the stream."""

    def row(**fields: Any) -> generate.Row:
        return {"id": fields["id"], "family": "f", "q": "?", "dataset": dataset, **fields}

    numeric, mcq = generate.cases(
        [
            row(
                id="n",
                type="numeric",
                a=3.0,
                band=1.0,
                context=[["odom", [0.5, 3.5]]],
                split="holdout",
            ),
            row(
                id="m",
                type="mcq",
                a="north",
                choices=["north", "south"],
                context=[["odom", [0, 10]]],
            ),
        ],
        tags=frozenset({"odom"}),
    )

    def score(case: EvalCase, answer: str) -> float:
        return case.grade(Outcome(trajectory=_trajectory(answer, tmp_path), artifacts={}))

    assert numeric.tags == {"odom", "f", "numeric", "holdout"} and mcq.tags == {"odom", "f", "mcq"}
    assert score(numeric, "about 3.5") == 0.5 and score(numeric, "no idea") == 0.0
    assert score(mcq, "South, then north.") == 1.0 and score(mcq, "east") == 0.0
    with pytest.raises(ValueError):
        generate.cases([row(id="x", type="coords", a=[], context=[["odom", [0, 10]]])])
    with pytest.raises(ValueError):
        generate.cases([row(id="x", type="numeric", a=1, band=1, context=[["odom", [0, 1], {}]])])
    try:
        running = numeric.environment.start(())
        window = [1001.0, 1002.0, 1003.0]
        assert [s.name for s in running.streams] == ["odom"]
        assert [o.ts for o in running.streams[0]] == window
    finally:
        numeric.environment.stop()
