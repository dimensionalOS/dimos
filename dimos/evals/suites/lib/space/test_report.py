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
from pathlib import Path
import subprocess
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import data, report
from dimos.evals.suites.lib.space.constants import SELECTED_INDICES
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
    payload = json.dumps(rows).encode()
    digest = hashlib.sha256(payload).hexdigest()
    # Pin generated fixture bytes while exercising the real data verifier/selector.
    monkeypatch.setattr(data, "DATA_BYTES", len(payload))
    monkeypatch.setattr(data, "DATA_SHA256", digest)
    monkeypatch.setattr(report, "DATA_SHA256", digest)
    paths = SpacePaths(tmp_path / "cache")
    paths.questions.parent.mkdir(parents=True)
    paths.questions.write_bytes(payload)
    by_index = {example.index: example for example in data.load_examples(paths)}

    def create(indices: Sequence[int] = SELECTED_INDICES[:4]) -> SavedRun:
        directory = tmp_path / f"run-{len(list(tmp_path.glob('run-*')))}"
        directory.mkdir()
        examples = tuple(by_index[index] for index in indices)
        manifest = {
            "schema_version": 1,
            "source": {
                "kind": "suite_module",
                "value": "dimos.evals.suites.space_map_sketching",
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
    state: dict[str, Any] = {"record": None, "duplicate": False, "returncode": 0}

    def launch(args: list[str], log: Path, timeout_s: float) -> int:
        assert timeout_s == report.WORKER_TIMEOUT_S
        log.write_text("Synthetic SPACE worker.\n")
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
        return int(state["returncode"])

    state["run_process"] = mocker.patch.object(report, "run_process", side_effect=launch)
    return state


@pytest.mark.parametrize("final_newline", ["\n", ""], ids=["newline", "no-newline"])
def test_uses_only_official_metrics_and_preserves_every_raw_reply(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], final_newline: str
) -> None:
    saved = make_run(SELECTED_INDICES[3::-1])
    path = saved.directory / "results.jsonl"
    path.write_text(path.read_text().removesuffix("\n") + final_newline)
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
    assert [c["case_id"] for c in result["cases"]] == [e.case_id for e in saved.examples]
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


def test_exact_fixed_subset_is_complete(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES)
    result = report.score_run(saved.paths, saved.directory)
    assert result["fixed_subset_complete"]
    assert result["selected_count"] == result["scored_denominator"] == 20


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
    worker["run_process"].assert_not_called()


@pytest.mark.parametrize(
    ("position", "fragment"),
    [(1, b"{\n"), (4, b"{\n"), (4, b"{"), (4, b"\xe2\x82")],
    ids=["interior", "terminated-tail", "truncated-json", "truncated-utf8"],
)
def test_corrupt_results_are_rejected_without_modifying_evidence(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], position: int, fragment: bytes
) -> None:
    saved = make_run()
    path = saved.directory / "results.jsonl"
    lines = path.read_bytes().splitlines(keepends=True)
    lines.insert(position, fragment)
    corrupted = b"".join(lines)
    path.write_bytes(corrupted)

    with pytest.raises((json.JSONDecodeError, UnicodeDecodeError)):
        report.score_run(saved.paths, saved.directory)
    assert path.read_bytes() == corrupted
    worker["run_process"].assert_not_called()
    assert list(saved.directory.glob("space-score-*")) == []


@pytest.mark.parametrize("problem", ["duplicate", "unknown", "invalid_field"])
def test_invalid_final_record_without_newline_is_rejected(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], problem: str
) -> None:
    saved = make_run()
    rows = saved.results()
    if problem == "duplicate":
        rows[-1] = dict(rows[0])
    elif problem == "unknown":
        rows[-1]["case_id"] = "not-selected"
    else:
        rows[-1]["final_answer"] = None
    saved.write_results(rows)
    path = saved.directory / "results.jsonl"
    path.write_bytes(path.read_bytes().removesuffix(b"\n"))

    with pytest.raises(ValueError, match="Duplicate|Unknown|invalid final_answer"):
        report.score_run(saved.paths, saved.directory)
    worker["run_process"].assert_not_called()


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
    worker["run_process"].assert_not_called()


def test_saved_original_prompt_is_required(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    trace = saved.directory / saved.examples[0].case_id / "trajectory.json"
    record = json.loads(trace.read_text())
    record["steps"][0]["message"] += " changed"
    trace.write_text(json.dumps(record))
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert result["counts"]["infrastructure_error"] == 1
    worker["run_process"].assert_not_called()


@pytest.mark.parametrize("change", ["duplicate", "unknown", "source"])
def test_rejects_wrong_manifest_provenance(make_run: Callable[..., SavedRun], change: str) -> None:
    saved = make_run()
    path = saved.directory / "manifest.json"
    value = json.loads(path.read_text())
    if change == "duplicate":
        value["selection"]["case_ids"].append(value["selection"]["case_ids"][0])
    elif change == "unknown":
        value["selection"]["case_ids"][0] = "space-map-sketching-text-001"
    else:
        value["source"]["value"] = "dimos.evals.suites.other"
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        report.score_run(saved.paths, saved.directory)


@pytest.mark.parametrize("problem", ["exit", "missing", "counts", "duplicate", "aggregate"])
def test_failed_or_malformed_official_replay_remains_incomplete(
    make_run: Callable[..., SavedRun], worker: dict[str, Any], problem: str
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    if problem == "exit":
        worker["returncode"] = 9
    elif problem == "missing":
        worker["run_process"].side_effect = None
        worker["run_process"].return_value = 0
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


def test_worker_timeout_is_an_infrastructure_failure(
    make_run: Callable[..., SavedRun], worker: dict[str, Any]
) -> None:
    saved = make_run(SELECTED_INDICES[:1])
    worker["run_process"].side_effect = subprocess.TimeoutExpired("worker", 120)
    result = report.score_run(saved.paths, saved.directory)
    assert not result["complete"]
    assert "timed out" in result["scoring_error"]
    assert result["counts"]["infrastructure_error"] == 1
    assert result["scored_denominator"] == 0
    assert result["official_percent"] is None


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
    worker["run_process"].assert_not_called()


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
