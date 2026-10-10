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

import json
from pathlib import Path

from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.report import by_signature, by_stressor, load, render, write
from dimos.navigation.bench.suite import Case, Manifest, Rules


def _case(case_id: str, tag: str, clearance: float) -> Case:
    return Case(
        family="office",
        seed=1,
        params={},
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, 0.0),
        tag=tag,
        id=case_id,
        split="dev",
        difficulty=Difficulty(clearance, 1, 1.0, 0),
        route_length=3.0,
        scene_digest="",
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


def test_report_tallies_outcomes_by_stressor_and_signature(tmp_path: Path) -> None:
    cases = [
        _case("mined-s1-a", "mined", 0.8),
        _case("narrow_door-s1-b", "narrow_door", 0.3),
        _case("narrow_door-s2-c", "narrow_door", 0.5),
    ]
    Manifest("test", Rules(), 0, None, False, cases, []).save(tmp_path / "suite.json")
    run = tmp_path / "run"
    run.mkdir()
    meta = {
        "suite": str(tmp_path / "suite.json"),
        "suite_name": "test",
        "split": "dev",
        "blueprint": "fake",
        "policy": None,
        "overrides": ["--set-x=1"],
        "git_sha": "abc",
        "git_dirty": True,
        "host": "box",
        "episodes": 3,
        "finished": True,
        "started": 0.0,
        "time": 90.0,
    }
    (run / "run.json").write_text(json.dumps(meta))
    (run / "results.jsonl").write_text(
        "\n".join(
            [
                _result(cases[0], "success", spl=0.9, arrived_s=12.0),
                _result(cases[1], "stalled", signature="local_refused", final_xy=(2.0, 1.5)),
                _result(cases[2], None),
            ]
        )
    )
    report = load(run)
    assert report.outcomes == ["success", "stalled", "no recording"]
    assert by_stressor(report)["narrow_door"] == {"stalled": 1, "no recording": 1}
    assert by_signature(report) == {"local_refused": {"narrow_door": 1}}
    assert report.rows[1].final_xy == (2.0, 1.5) and report.rows[2].spl is None
    page = render(report)
    assert "1 of 3 episodes succeeded, median SPL of successes 0.90" in page
    assert "<dd>1.5 min</dd>" in page and "<dd>abc (dirty)</dd>" in page
    assert "<td>stalled</td>" in page
    assert "dimos nav-bench replay narrow_door-s1-b" in page
    assert page.count("<circle") == 3 and "<tfoot><tr><td>all</td>" in page
    assert write(run) == run / "report.html" and (run / "report.html").read_text() == page
