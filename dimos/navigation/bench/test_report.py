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

import html
import json
import os
from pathlib import Path
import shlex

import pytest

from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.report import (
    by_level,
    by_signature,
    by_stressor,
    crossing,
    encountered,
    load,
    render,
    write,
)
from dimos.navigation.bench.suite import Case, Level, Manifest, Rules


def _case(
    case_id: str, tag: str, clearance: float, level: Level | None = None, seed: int = 1
) -> Case:
    return Case(
        family="office",
        seed=seed,
        params={},
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, 0.0),
        tag=tag,
        id=case_id,
        split="dev",
        difficulty=Difficulty(clearance, 1, 1.0, 0),
        route_length=3.0,
        scene_digest="",
        level=level,
    )


def _result(case: Case, outcome: str | None, **metrics: object) -> str:
    base = {"signature": None, "spl": None, "arrived_s": None, "final_xy": (4.0, 1.0)}
    return json.dumps(
        {
            "case_id": case.id,
            "tag": case.tag,
            "outcome": outcome,
            "error": None if outcome else "no recording",
            "terminal": "arrived",
            "metrics": {**base, **metrics} if outcome else {},
        }
    )


def _run(run: Path, cases: list[Case], results: list[str], **meta: object) -> None:
    run.mkdir()
    Manifest("test", Rules(), 0, cases).save(run / "suite.json")
    base = {
        "split": "dev",
        "blueprint": "fake",
        "policy": None,
        "overrides": ["--set-x=1"],
        "episodes": len(cases),
        "finished": True,
        "started": 0.0,
        "time": 90.0,
    }
    (run / "run.json").write_text(json.dumps({**base, **meta}))
    (run / "results.jsonl").write_text("\n".join(results))


def test_replay_commands_name_the_episode_of_the_reports_own_run(tmp_path: Path) -> None:
    case = _case("mined-s1-a", "mined", 0.8)
    older, newer = tmp_path / "older run", tmp_path / "newer run"
    for run in (older, newer):
        _run(run, [case], [_result(case, "success")])
    for run in (older, newer):
        command = f"dimos nav-bench replay {shlex.quote(str(run.resolve() / case.id))}"
        assert command in html.unescape(render(load(run)))


def test_an_interrupted_run_keeps_the_stop_time_it_recorded(tmp_path: Path) -> None:
    case = _case("mined-s1-a", "mined", 0.8)
    stopped, killed = tmp_path / "stopped", tmp_path / "killed"
    _run(stopped, [case], [_result(case, "success")], finished=False, time=90.0)
    _run(killed, [case], [_result(case, "success")], finished=False, time=None)
    for run in (stopped, killed):
        os.utime(run / "results.jsonl", (600.0, 600.0))
    assert load(stopped).run.time == 90.0
    assert load(killed).run.time == 600.0


def test_report_tallies_outcomes_by_stressor_and_signature(tmp_path: Path) -> None:
    cases = [
        _case("mined-s1-a", "mined", 0.8),
        _case("narrow_door-s1-b", "narrow_door", 0.3),
        _case("narrow_door-s2-c", "narrow_door", 0.5),
    ]
    run = tmp_path / "run"
    _run(
        run,
        cases,
        [
            _result(cases[0], "success", spl=0.9, arrived_s=12.0),
            _result(cases[1], "stalled", signature="local_refused", final_xy=(2.0, 1.5)),
            _result(cases[2], None),
        ],
    )
    report = load(run)
    assert report.outcomes == ["success", "stalled", "no recording"]
    assert by_stressor(report)["narrow_door"] == {"stalled": 1, "no recording": 1}
    assert by_signature(report) == {"local_refused": {"narrow_door": 1}}
    assert report.rows[1].final_xy == (2.0, 1.5) and report.rows[2].spl is None
    page = render(report)
    assert "1 of 3 episodes succeeded, median SPL of successes 0.90" in page
    assert "<dd>1.5 min</dd>" in page and "<dd>test</dd>" in page
    assert "<td>stalled</td>" in page
    assert f"dimos nav-bench replay {run.resolve() / 'narrow_door-s1-b'}" in page
    assert page.count("<circle") == 3 and "<tfoot><tr><td>all</td>" in page
    assert by_level(report) == [] and "Success against" not in page
    assert (
        encountered(report) is None
        and "not encountered" not in page
        and "nearest to feature" not in page
    )
    assert write(run) == run / "report.html" and (run / "report.html").read_text() == page


def test_report_gives_the_success_rate_per_level_and_where_it_crosses(tmp_path: Path) -> None:
    wins = {0.2: 0, 0.3: 1, 0.4: 2}
    cases, results = [], []
    for value, won in wins.items():
        for k in range(2):
            case = _case(f"gap-s{k}-{value}", "gap", 0.5, Level("goal_clearance", value))
            cases.append(case)
            results.append(
                _result(case, "success")
                if k < won
                else _result(case, "stalled", signature="local_refused")
            )
    run = tmp_path / "run"
    _run(run, cases, results)
    report = load(run)
    levels = by_level(report)
    assert [(row.value, row.n, row.rate) for row in levels] == [
        (0.2, 2, 0.0),
        (0.3, 2, 0.5),
        (0.4, 2, 1.0),
    ]
    assert levels[0].signatures == {"local_refused": 2} and levels[2].signatures == {}
    assert crossing(levels, 0.5) == pytest.approx(0.3)
    assert crossing(levels, 0.9) == pytest.approx(0.38)
    assert crossing(levels[:2], 0.9) is None and crossing(levels[2:], 0.9) == 0.4
    page = render(report)
    assert "Success rate 50% at 0.30 m, 90% at 0.38 m" in page
    assert "<h2>Success against goal clearance</h2>" in page and page.count("<rect") == 3
    assert "local_refused 2</td>" in page
    assert ">1/2</text>" in page


def test_report_leaves_out_cases_whose_control_never_met_the_feature(tmp_path: Path) -> None:
    nearest = {1: 0.05, 2: 0.40, 3: 0.10}
    cases, results = [], []
    for seed, feature_m in nearest.items():
        control = _case(f"leg-s{seed}-ctl", "leg", 0.5, Level("leg_width", 0.0, control=True), seed)
        cases.append(control)
        results.append(_result(control, "success", feature_m=feature_m))
        for value in (0.02, 0.08):
            case = _case(f"leg-s{seed}-{value}", "leg", 0.5, Level("leg_width", value), seed)
            cases.append(case)
            hit = value == 0.02 and seed != 2
            results.append(
                _result(case, "reached_with_collision" if hit else "success", feature_m=0.6)
            )
    run = tmp_path / "run"
    _run(run, cases, results)
    report = load(run)
    assert encountered(report) == {1, 3}
    levels = by_level(report)
    assert [(row.value, row.n, row.successes, row.not_encountered) for row in levels] == [
        (0.02, 2, 0, 1),
        (0.08, 2, 2, 1),
    ]
    page = render(report)
    assert "Feature encountered in 2 of 3 control runs" in page
    assert '<th class="n">not encountered</th>' in page and ">0/2</text>" in page
    assert '<td class="n">0.40</td>' in page
