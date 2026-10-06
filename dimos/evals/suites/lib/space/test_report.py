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

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
from typing import Any, TextIO

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import data, report
from dimos.evals.suites.lib.space.constants import (
    SELECTED_INDICES,
    SMOKE_INDEX,
    SPACE_REVISION,
    TASK,
)
from dimos.evals.suites.lib.space.data import Example, SpacePaths


@dataclass
class SavedRun:
    paths: SpacePaths
    directory: Path
    examples: tuple[Example, ...]

    def results(self) -> list[dict[str, Any]]:
        return [
            json.loads(line) for line in (self.directory / "results.jsonl").read_text().splitlines()
        ]

    def write_results(self, results: list[dict[str, Any]]) -> None:
        (self.directory / "results.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in results)
        )


@pytest.fixture
def make_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Callable[..., SavedRun]:
    rows = [
        {"question": f"Synthetic question {i}", "answer": 1, "metadata": {"env_dir": f"test_{i}"}}
        for i in range(max(SELECTED_INDICES) + 1)
    ]
    for position, index in enumerate(SELECTED_INDICES):
        rows[index]["metadata"] = {"env_dir": f"layout{position // 2}_{position % 2}"}
        rows[index]["answer"] = position % 4 + 1
    payload = json.dumps(rows).encode()
    digest = hashlib.sha256(payload).hexdigest()
    # Pin generated fixture bytes while exercising the real data verifier/selector.
    monkeypatch.setattr(data, "DATA_BYTES", len(payload))
    monkeypatch.setattr(data, "DATA_SHA256", digest)
    monkeypatch.setattr(report, "DATA_SHA256", digest)
    paths = SpacePaths(tmp_path / "cache")
    paths.questions.parent.mkdir(parents=True)
    paths.questions.write_bytes(payload)

    def create(indices: Sequence[int] = SELECTED_INDICES[:4], mode: str = "evaluation") -> SavedRun:
        directory = tmp_path / f"run-{len(list(tmp_path.glob('run-*')))}"
        directory.mkdir()
        examples = data.load_examples(paths, indices)
        manifest = {
            "schema_version": 1,
            "source": {
                "kind": "space",
                "task": TASK,
                "space_revision": SPACE_REVISION,
                "data_sha256": digest,
                "mode": mode,
                "cases": [e.identity() for e in examples],
            },
            "selection": {"case_ids": [e.case_id for e in examples]},
        }
        (directory / "manifest.json").write_text(json.dumps(manifest))
        results = []
        for example in examples:
            case = directory / example.case_id
            case.mkdir()
            reply = f'  {{"answer":1}}\nraw reply for {example.index}'
            trajectory = {
                "steps": [
                    {"source": "user", "message": example.question},
                    {"source": "agent", "message": reply},
                ],
                "extra": {"ended_by": "answer"},
            }
            (case / "trajectory.json").write_text(json.dumps(trajectory))
            results.append(
                {
                    "case_id": example.case_id,
                    "ended_by": "answer",
                    "error": "",
                    "final_answer": reply,
                    "trajectory": str(case / "trajectory.json"),
                    "score": -999.0,
                }
            )
        saved = SavedRun(paths, directory, examples)
        saved.write_results(results)
        return saved

    return create


@pytest.fixture
def worker(mocker: MockerFixture) -> dict[str, Any]:
    state: dict[str, Any] = {"calls": [], "record": None, "duplicate": False}
    process = mocker.MagicMock()
    process.__enter__.return_value = process
    process.wait.return_value = 0
    process.pid = 98765
    state["process"] = process
    mocker.patch.object(os, "killpg")

    def launch(args: list[str], *, stdout: TextIO, stderr: int, start_new_session: bool) -> Any:
        state["calls"].append(args)
        assert start_new_session
        assert stderr == subprocess.STDOUT
        stdout.write("Synthetic worker boundary; no model or benchmark data.\n")
        questions = json.loads(Path(args[args.index("--questions") + 1]).read_text())
        state["questions"] = questions
        state["replies"] = json.loads(Path(args[args.index("--replies") + 1]).read_text())
        output = Path(args[args.index("--output") + 1]) / "dimos_saved_responses" / "timestamp"
        output.mkdir(parents=True)
        record = state["record"]
        if record is None:
            record = {
                "all_metrics": [{"accuracy": 100.0}] * len(questions),
                "all_predictions": [1] * len(questions),
                "mean_metrics": {"accuracy": 100.0},
            }
        (output / "results.json").write_text(json.dumps(record))
        if state["duplicate"]:
            other = output.parent / "other"
            other.mkdir()
            (other / "results.json").write_text(json.dumps(record))
        return process

    mocker.patch.object(subprocess, "Popen", side_effect=launch)
    return state


def test_uses_only_official_metrics_and_preserves_every_raw_reply(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run()
    worker["record"] = {
        "all_metrics": [
            {"accuracy": 100.0},
            {"accuracy": 0.0},
            {"accuracy": 0.0},
            {"accuracy": 0.0},
        ],
        "all_predictions": [1, 1, None, 9],
        "mean_metrics": {"accuracy": 25.0},
    }
    result = report.score_run(saved.paths, saved.directory)
    assert result["complete"]
    assert not result["fixed_subset_complete"]
    assert result["official_percent"] == 25.0
    assert result["scored_denominator"] == 4
    assert [c["status"] for c in result["cases"]] == [
        "correct",
        "wrong",
        "parser_invalid",
        "invalid_choice",
    ]
    assert worker["questions"] == [e.qa for e in saved.examples]
    assert worker["replies"] == {
        e.question_sha256: r["final_answer"]
        for e, r in zip(saved.examples, saved.results(), strict=True)
    }
    assert json.loads(Path(result["report_path"]).read_text()) == result


def test_exact_fixed_subset_is_distinct_from_smoke_and_partial_selection(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES)
    result = report.score_run(saved.paths, saved.directory)
    assert result["fixed_subset_complete"]
    assert result["selected_count"] == result["scored_denominator"] == 20


@pytest.mark.parametrize("mode", ["smoke", "offline-smoke"])
def test_smoke_labels_and_reserved_case_are_explicit(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], mode: str
) -> None:
    saved = make_run([SMOKE_INDEX], mode)
    result = report.score_run(saved.paths, saved.directory)
    assert result["mode"] == mode
    assert result["complete"]
    assert not result["fixed_subset_complete"]


def test_missing_outer_results_keeps_traces_diagnostic_without_scoring(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run()
    (saved.directory / "results.jsonl").unlink()
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert result["counts"]["unreported"] == 4
    assert result["scored_denominator"] == 0
    assert result["official_percent"] is None
    assert all(c["trace"]["valid"] for c in result["cases"])
    assert worker["calls"] == []


def test_errors_timeouts_and_unreported_cases_never_supply_replay_answers(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run()
    rows = saved.results()
    rows[1].update(ended_by="timeout", error="deadline")
    rows[2]["error"] = "grading or provider failure"
    saved.write_results(rows[:3])
    result = report.score_run(saved.paths, saved.directory)
    assert [c["status"] for c in result["cases"]] == [
        "correct",
        "timeout",
        "infrastructure_error",
        "unreported",
    ]
    assert result["scored_denominator"] == 1
    assert not result["complete"]
    assert list(worker["replies"]) == [saved.examples[0].question_sha256]


@pytest.mark.parametrize(
    "field,value", [("final_answer", "altered"), ("trajectory", "other/trajectory.json")]
)
def test_result_must_match_its_saved_trace(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], field: str, value: str
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    rows = saved.results()
    rows[0][field] = value
    saved.write_results(rows)
    result = report.score_run(saved.paths, saved.directory)
    assert result["counts"]["infrastructure_error"] == 1
    assert worker["calls"] == []


@pytest.mark.parametrize("generic", [False, True])
def test_saved_original_prompt_is_required_even_for_generic_suite_provenance(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], generic: bool
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    manifest_path = saved.directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if generic:
        manifest["source"] = {
            "kind": "suite_module",
            "value": "dimos.evals.suites.space_map_sketching",
        }
        manifest_path.write_text(json.dumps(manifest))
    assert report.score_run(saved.paths, saved.directory)["complete"]
    trace = saved.directory / saved.examples[0].case_id / "trajectory.json"
    record = json.loads(trace.read_text())
    record["steps"][0]["message"] += " changed"
    trace.write_text(json.dumps(record))
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert result["counts"]["infrastructure_error"] == 1
    assert len(worker["calls"]) == 1


@pytest.mark.parametrize("bad", ["duplicate", "unknown"])
def test_rejects_ambiguous_result_ids(make_run: Callable[..., SavedRun], bad: str) -> None:
    saved = make_run()
    rows = saved.results()
    extra = dict(rows[0])
    if bad == "unknown":
        extra["case_id"] = "not-selected"
    saved.write_results([*rows, extra])
    with pytest.raises(ValueError, match="Duplicate|Unknown"):
        report.score_run(saved.paths, saved.directory)


@pytest.mark.parametrize("change", ["duplicate", "unknown", "revision", "identity", "mode"])
def test_rejects_wrong_manifest_provenance(make_run: Callable[..., SavedRun], change: str) -> None:
    saved = make_run()
    path = saved.directory / "manifest.json"
    value = json.loads(path.read_text())
    if change == "duplicate":
        value["selection"]["case_ids"].append(value["selection"]["case_ids"][0])
    elif change == "unknown":
        value["selection"]["case_ids"][0] = "space-map-sketching-text-001"
    elif change == "revision":
        value["source"]["space_revision"] = "wrong"
    elif change == "identity":
        value["source"]["cases"][0]["question_sha256"] = "wrong"
    else:
        value["source"]["mode"] = "smoke"
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        report.score_run(saved.paths, saved.directory)


@pytest.mark.parametrize("problem", ["exit", "counts", "duplicate", "aggregate"])
def test_failed_or_malformed_official_replay_remains_incomplete(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], problem: str
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    if problem == "exit":
        worker["process"].wait.return_value = 9
    elif problem == "duplicate":
        worker["duplicate"] = True
    else:
        worker["record"] = {
            "all_metrics": [{"accuracy": 100.0}],
            "all_predictions": [] if problem == "counts" else [1],
            "mean_metrics": {"accuracy": 0.0},
        }
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert result["scoring_error"]
    assert result["counts"]["infrastructure_error"] == 1
    assert result["scored_denominator"] == 0
    assert result["official_percent"] is None


def test_worker_timeout_reaps_its_entire_spawn_group(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], mocker: MockerFixture
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    worker["process"].wait.side_effect = [
        subprocess.TimeoutExpired("worker", 120),
        subprocess.TimeoutExpired("worker", 5),
        0,
    ]
    kill = mocker.patch.object(os, "killpg")
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert kill.call_args_list == [
        mocker.call(98765, signal.SIGTERM),
        mocker.call(98765, signal.SIGKILL),
    ]
    assert worker["process"].wait.call_count == 3


def test_rescoring_never_overwrites_prior_evidence(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    first = report.score_run(saved.paths, saved.directory)
    before = Path(first["report_path"]).read_bytes()
    second = report.score_run(saved.paths, saved.directory)
    assert first["report_path"] != second["report_path"]
    assert Path(first["report_path"]).read_bytes() == before


@pytest.mark.parametrize("problem", ["missing", "termination", "malformed"])
def test_bad_completed_trajectory_is_not_scored(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], problem: str
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    path = saved.directory / saved.examples[0].case_id / "trajectory.json"
    if problem == "missing":
        path.unlink()
    elif problem == "malformed":
        path.write_text("{broken")
    else:
        record = json.loads(path.read_text())
        record["extra"]["ended_by"] = "timeout"
        path.write_text(json.dumps(record))
    result = report.score_run(saved.paths, saved.directory)
    assert result["counts"]["infrastructure_error"] == 1
    assert not result["complete"]
    assert worker["calls"] == []


def test_nonfinite_prediction_is_diagnostic_without_changing_official_score(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    worker["record"] = {
        "all_metrics": [{"accuracy": 0.0}],
        "all_predictions": [float("nan")],
        "mean_metrics": {"accuracy": 0.0},
    }
    result = report.score_run(saved.paths, saved.directory)
    assert result["complete"]
    assert result["official_percent"] == 0.0
    assert result["counts"]["invalid_choice"] == 1
    assert result["cases"][0]["prediction_repr"] == "nan"
    assert json.loads(Path(result["report_path"]).read_text()) == result
