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

from dimos.navigation.bench.compare import (
    Tally,
    compare,
    load_side,
    to_json,
    warnings,
)
from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.suite import Case, Level, Manifest, Rules

DAY_S = 86400.0


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
    started: float = 0.0,
    policy: str | None = "policy.bin",
    metrics: dict[str, dict[str, float]] | None = None,
) -> Path:
    path.mkdir(parents=True)
    Manifest(suite, Rules(), 0, [c for c, _ in episodes]).save(path / "suite.json")
    meta: dict[str, object] = {
        "split": "dev",
        "blueprint": "fake",
        "policy": policy,
        "overrides": [],
        "episodes": len(episodes),
        "finished": True,
        "started": started,
        "time": started + 60.0,
    }
    (path / "run.json").write_text(json.dumps(meta))
    lines = [
        json.dumps(
            {
                "case_id": case.id,
                "outcome": outcome,
                "error": None if outcome else "no recording",
                "metrics": (metrics or {}).get(case.id, {}) if outcome else {},
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


def test_a_case_run_again_keeps_its_latest_episode(tmp_path: Path) -> None:
    case = _case("door_width-s1-x")
    later = _run(tmp_path / "redo", [(case, "success")], started=100.0)
    earlier = _run(tmp_path / "first", [(case, "stuck")], started=0.0)
    side = load_side("new", [later, earlier])
    assert side.episodes[case.id].outcome == "success"
    assert side.replaced == 1


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


def test_levels_are_tallied_with_the_control_first(tmp_path: Path) -> None:
    control = Level("door_width", 0.0, control=True)
    episodes = [
        (_case("door_width-s1-a", level=Level("door_width", 0.6)), "stuck", "success"),
        (_case("door_width-s2-a", level=Level("door_width", 0.6)), "stuck", "stuck"),
        (_case("door_width-s1-b", level=Level("door_width", 0.5)), "success", "fall"),
        (_case("door_width-s1-c", level=control), "success", "success"),
    ]
    base = _run(tmp_path / "base", [(c, b) for c, b, _ in episodes])
    new = _run(tmp_path / "new", [(c, n) for c, _, n in episodes])
    (row,) = compare(load_side("base", [base]), load_side("new", [new])).probes
    assert [level.value for level in row.levels] == [None, 0.5, 0.6]
    assert [(lv.tally.cases, lv.tally.change) for lv in row.levels] == [(1, 0), (1, -1), (2, 1)]


def test_measures_use_only_cases_with_a_value_on_both_sides(tmp_path: Path) -> None:
    cases = [_case(f"door_width-s{i}-x") for i in range(1, 5)]
    base_metrics = {
        cases[0].id: {"spl": 0.5, "first_contact_speed": 0.9},
        cases[1].id: {"spl": 0.7},
        cases[2].id: {"spl": 0.9},
    }
    new_metrics = {
        cases[0].id: {"spl": 0.8, "first_contact_speed": 0.4},
        cases[1].id: {"spl": 0.9, "first_contact_speed": 0.2},
        cases[3].id: {"spl": 1.0},
    }
    base = _run(tmp_path / "base", [(c, "success") for c in cases], metrics=base_metrics)
    new = _run(tmp_path / "new", [(c, "success") for c in cases], metrics=new_metrics)
    (row,) = compare(load_side("base", [base]), load_side("new", [new])).probes
    assert [(m.name, m.n) for m in row.measures] == [("first_contact_speed", 1), ("spl", 2)]
    assert [v for m in row.measures for v in (m.base, m.new)] == pytest.approx(
        [0.9, 0.4, 0.6, 0.85]
    )


def test_the_repeat_gives_each_probe_its_flip_rate(tmp_path: Path) -> None:
    cases = [_case(f"door_width-s{i}-x") for i in range(1, 5)]
    base = _run(tmp_path / "base", [(c, "success") for c in cases])
    repeat = _run(
        tmp_path / "repeat",
        [(c, o) for c, o in zip(cases, ["stuck", "success", "success", "success"], strict=True)],
    )
    new = _run(tmp_path / "new", [(c, "success") for c in cases])
    sides = [load_side(name, [path]) for name, path in (("a", base), ("b", new), ("c", repeat))]
    (row,) = compare(*sides).probes
    assert row.noise is not None
    assert (row.noise.cases, row.noise.flips, row.noise.rate) == (4, 1, 0.25)


def test_json_carries_the_summary_and_per_probe_numbers(tmp_path: Path) -> None:
    cases = [_case(f"door_width-s{i}-x") for i in range(1, 8)]
    base = _run(tmp_path / "base", [(c, "stuck") for c in cases], started=0.0)
    new = _run(tmp_path / "new", [(c, "success") for c in cases], started=DAY_S * 3, policy=None)
    result = compare(load_side("base", [base]), load_side("new", [new]))
    data = json.loads(json.dumps(to_json(result)))
    assert data["summary"] == {
        "cases": 7,
        "base_successes": 0,
        "new_successes": 7,
        "probes_better": 1,
        "probes_worse": 0,
        "probes_no_clear_change": 0,
        "outcomes_changed": 7,
    }
    (probe,) = data["probes"]
    assert (probe["probe"], probe["change"], probe["verdict"]) == ("door_width", 7, "better")
    assert probe["p"] == pytest.approx(2 / 2**7)
    assert probe["base_failures"] == {"stuck": 7} and probe["new_failures"] == {}
    assert data["changed"][0]["base"]["recorded"] is False
    assert len(warnings(result)) == 3
