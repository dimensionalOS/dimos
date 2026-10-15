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

from collections import Counter
from dataclasses import replace
import json
from pathlib import Path

import pytest

from dimos.navigation.bench.compare import Tally, compare, load_side, to_json, write
from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.suite import Case, Level, Manifest, Rules


def _case(case_id: str, tag: str = "door_width", level: Level | None = None) -> Case:
    return Case(
        family="office",
        seed=int(case_id.split("-s")[1].split("-")[0]) if "-s" in case_id else 1,
        params={},
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, 0.0),
        tag=tag,
        id=case_id,
        split="dev",
        difficulty=Difficulty(0.5, 1, 1.0, 0),
        route_length=3.0,
        scene_digest="d",
        level=level,
    )


def _run(
    path: Path,
    episodes: list[tuple[Case, str | None]],
    *,
    suite: str = "door_width",
) -> Path:
    path.mkdir(parents=True)
    Manifest(suite, Rules(), 0, [c for c, _ in episodes]).save(path / "suite.json")
    meta: dict[str, object] = {
        "split": "dev",
        "blueprint": "fake",
        "policy": "policy.bin",
        "overrides": [],
        "episodes": len(episodes),
        "finished": True,
        "started": 0.0,
        "time": 60.0,
    }
    (path / "run.json").write_text(json.dumps(meta))
    lines = [
        json.dumps(
            {
                "case_id": case.id,
                "outcome": outcome,
                "error": None if outcome else "no recording",
                "metrics": {"spl": 1.0} if outcome == "success" else {},
            }
        )
        for case, outcome in episodes
    ]
    (path / "results.jsonl").write_text("\n".join(lines) + "\n")
    return path


def test_pairs_valid_cases_on_both_sides_and_counts_the_rest(tmp_path: Path) -> None:
    a, b, c, d, e, f = (_case(f"door_width-s{i}-x") for i in range(1, 7))
    moved = replace(c, scene_digest="other")
    base = _run(tmp_path / "base", [(a, "success"), (b, "stuck"), (c, "success"), (d, None)])
    new = tmp_path / "new"
    _run(new / "door_width", [(b, "success"), (moved, "success"), (d, "success")])
    _run(new / "leg_width", [(e, "success"), (f, "fall")], suite="leg_width")
    result = compare(load_side("base", [base]), load_side("new", [new]))
    assert [p.case.id for p in result.pairs] == [b.id]
    assert result.left_out["base"] == Counter(
        {"only on this side": 1, "invalid": 1, "scene differs": 1}
    )
    assert result.left_out["new"] == Counter(
        {"only on this side": 2, "invalid": 1, "scene differs": 1}
    )
    assert result.scene_differs == [c.id]


def test_tally_counts_flips_and_failure_types(tmp_path: Path) -> None:
    cases = [_case(f"door_width-s{i}-x") for i in range(1, 7)]
    base_outcomes = ["stuck", "fall", "success", "success", "stalled", "success"]
    new_outcomes = ["success", "success", "stuck", "success", "stalled", "success"]
    base = _run(tmp_path / "base", list(zip(cases, base_outcomes, strict=True)))
    new = _run(tmp_path / "new", list(zip(cases, new_outcomes, strict=True)))
    result = compare(load_side("base", [base]), load_side("new", [new]))
    (row,) = result.probes
    t = row.tally
    assert (t.cases, t.base_successes, t.new_successes, t.change) == (6, 3, 4, 1)
    assert (t.became_success, t.became_failure, t.unchanged) == (2, 1, 3)
    assert t.base_failures == Counter({"stuck": 1, "fall": 1, "stalled": 1})
    assert t.new_failures == Counter({"stuck": 1, "stalled": 1})
    assert [p.case.id for p in result.changed] == [c.id for c in cases[:3]]


@pytest.mark.parametrize(
    ("up", "down", "p", "verdict"),
    [
        (6, 0, 0.03125, "better"),
        (5, 1, 0.21875, "no clear change"),
        (0, 10, 0.001953125, "worse"),
        (0, 0, 1.0, "no clear change"),
    ],
)
def test_mcnemar_is_exact_on_the_discordant_pairs(
    up: int, down: int, p: float, verdict: str
) -> None:
    t = Tally(20, 10, 10 + up - down, up, down, Counter(), Counter())
    assert t.p == pytest.approx(p)
    assert t.verdict == verdict


def test_the_page_and_its_numbers_are_written(tmp_path: Path) -> None:
    cases = [
        _case("door_width-s1-a", level=Level("door_width", 0.0, control=True)),
        _case("door_width-s1-b", level=Level("door_width", 0.6)),
        _case("door_width-s2-b", level=Level("door_width", 0.6)),
    ]
    base = _run(tmp_path / "base", [(cases[0], "success"), (cases[1], "stuck"), (cases[2], "fall")])
    new = _run(tmp_path / "new", [(c, "success") for c in cases])
    repeat = _run(tmp_path / "repeat", [(c, "success") for c in cases])
    (base / cases[1].id).mkdir()
    (base / cases[1].id / "memory.db").touch()
    sides = [load_side(name, [path]) for name, path in (("a", base), ("b", new), ("c", repeat))]
    result = compare(*sides)
    page = write(result, tmp_path / "page" / "compare.html")
    assert page.read_text().startswith("<!doctype html>")
    assert json.loads((tmp_path / "page" / "compare.html.json").read_text()) == json.loads(
        json.dumps(to_json(result))
    )
