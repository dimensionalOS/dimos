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

"""Replay completed saved replies through SPACE; preserve incomplete-run evidence."""

from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any

from dimos.evals.suites.lib.space.constants import (
    DATA_SHA256,
    SELECTED_INDICES,
    SMOKE_INDEX,
    SPACE_REVISION,
    TASK,
)
from dimos.evals.suites.lib.space.data import Example, SpacePaths, load_examples
from dimos.evals.suites.lib.space.process import run_process

WORKER_TIMEOUT_S = 120.0
_STATUSES = (
    "correct",
    "wrong",
    "parser_invalid",
    "invalid_choice",
    "timeout",
    "infrastructure_error",
    "unreported",
)


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path.name}")
    return value


def _selection(paths: SpacePaths, manifest: dict[str, Any]) -> tuple[tuple[Example, ...], str]:
    if manifest.get("schema_version") != 1:
        raise ValueError("Unsupported EvalRunner manifest schema")
    selection = manifest.get("selection")
    ids = selection.get("case_ids") if isinstance(selection, dict) else None
    if not isinstance(ids, list) or not ids or any(not isinstance(i, str) for i in ids):
        raise ValueError("Manifest must select a nonempty list of case IDs")
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate selected case IDs")
    indices: list[int] = []
    for case_id in ids:
        match = re.fullmatch(r"space-map-sketching-text-(\d{3})", case_id)
        if match is None or int(match[1]) not in {*SELECTED_INDICES, SMOKE_INDEX}:
            raise ValueError(f"Unknown SPACE case ID: {case_id}")
        indices.append(int(match[1]))
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise ValueError("Missing SPACE source provenance")
    if source.get("kind") == "space":
        expected = {"task": TASK, "space_revision": SPACE_REVISION, "data_sha256": DATA_SHA256}
        if any(source.get(key) != value for key, value in expected.items()):
            raise ValueError("SPACE source provenance does not match pinned inputs")
        mode = source.get("mode")
        if not isinstance(mode, str) or mode not in {"evaluation", "smoke", "offline-smoke"}:
            raise ValueError("Unknown SPACE evaluation mode")
    elif source == {"kind": "suite_module", "value": "dimos.evals.suites.space_map_sketching"}:
        mode = "evaluation"
    else:
        raise ValueError("Manifest is not a supported SPACE source")
    if mode == "evaluation" and any(index not in SELECTED_INDICES for index in indices):
        raise ValueError("Evaluation selection must use the fixed subset")
    if mode != "evaluation" and indices != [SMOKE_INDEX]:
        raise ValueError("Smoke runs must select only the reserved smoke case")
    examples = load_examples(paths, indices)
    if source.get("kind") == "space" and source.get("cases") != [e.identity() for e in examples]:
        raise ValueError("SPACE case provenance does not match the selected questions")
    return examples, mode


def _results(run_dir: Path, selected: set[str]) -> dict[str, dict[str, Any]]:
    path = run_dir / "results.jsonl"
    if not path.exists():
        return {}
    results: dict[str, dict[str, Any]] = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        value = json.loads(line)
        if (
            not isinstance(value, dict)
            or not isinstance(value.get("case_id"), str)
            or value["case_id"] not in selected
        ):
            raise ValueError("Unknown or malformed result case ID")
        case_id = value["case_id"]
        if case_id in results:
            raise ValueError(f"Duplicate result for {case_id}")
        for key in ("error", "ended_by", "final_answer", "trajectory"):
            if not isinstance(value.get(key), str):
                raise ValueError(f"Result {case_id} has invalid {key}")
        results[case_id] = value
    return results


def _trace(run_dir: Path, example: Example, result: dict[str, Any] | None) -> dict[str, Any]:
    path = run_dir / example.case_id / "trajectory.json"
    diagnostic: dict[str, Any] = {"path": str(path), "present": path.is_file(), "valid": False}
    if not diagnostic["present"]:
        diagnostic["error"] = "Saved trajectory is missing"
        return diagnostic
    try:
        record = _object(path)
        steps = record.get("steps")
        if not isinstance(steps, list) or any(not isinstance(step, dict) for step in steps):
            raise ValueError("Malformed trajectory steps")
        users = [step for step in steps if step.get("source") == "user"]
        finals = [
            step for step in steps if step.get("source") == "agent" and not step.get("tool_calls")
        ]
        if not users or users[0].get("message") != example.question:
            raise ValueError("Saved first user message differs from the pinned question")
        if not finals or not isinstance(finals[-1].get("message"), str):
            raise ValueError("Saved trajectory has no final reply")
        extra = record.get("extra")
        if not isinstance(extra, dict):
            raise ValueError("Saved trajectory has no termination record")
        diagnostic.update(final_answer=finals[-1]["message"], ended_by=extra.get("ended_by"))
        if result is not None:
            reference = Path(result["trajectory"])
            if reference.parts[-2:] != (example.case_id, "trajectory.json"):
                raise ValueError("Result does not reference its case trajectory")
            if finals[-1]["message"] != result["final_answer"]:
                raise ValueError("Saved final reply differs from the recorded result")
            if extra.get("ended_by") != result["ended_by"] or extra.get("error"):
                raise ValueError("Saved termination record differs from the completed result")
        diagnostic["valid"] = True
    except (OSError, ValueError, TypeError) as exc:
        diagnostic["error"] = str(exc)
    return diagnostic


def _worker(paths: SpacePaths, replay: Path) -> Path:
    output = replay / "upstream"
    args = [
        sys.executable,
        "-m",
        "dimos.evals.suites.lib.space.worker",
        "--source",
        str(paths.source.resolve()),
        "--questions",
        str((replay / "questions.json").resolve()),
        "--replies",
        str((replay / "replies.json").resolve()),
        "--output",
        str(output.resolve()),
    ]
    returncode = run_process(args, replay / "worker.log", WORKER_TIMEOUT_S)
    if returncode:
        raise RuntimeError(
            f"Official SPACE worker failed with exit code {returncode}; see worker.log"
        )
    files = list(output.rglob("results.json"))
    if len(files) != 1:
        raise ValueError("Expected exactly one official results.json")
    return files[0]


def _official(path: Path, count: int) -> tuple[list[dict[str, Any]], list[Any], float]:
    value = _object(path)
    metrics, predictions = value.get("all_metrics"), value.get("all_predictions")
    if not isinstance(metrics, list) or not isinstance(predictions, list):
        raise ValueError("Malformed official result arrays")
    if len(metrics) != count or len(predictions) != count:
        raise ValueError("Official result counts differ from the replayed cases")
    if any(
        not isinstance(m, dict)
        or type(m.get("accuracy")) not in (int, float)
        or m["accuracy"] not in (0.0, 100.0)
        for m in metrics
    ):
        raise ValueError("Malformed official per-case accuracy")
    mean = value.get("mean_metrics")
    accuracy = mean.get("accuracy") if isinstance(mean, dict) else None
    if (
        isinstance(accuracy, bool)
        or not isinstance(accuracy, (int, float))
        or not math.isfinite(accuracy)
    ):
        raise ValueError("Malformed official aggregate accuracy")
    if not math.isclose(accuracy, sum(m["accuracy"] for m in metrics) / count, abs_tol=1e-9):
        raise ValueError("Official aggregate disagrees with its per-case metrics")
    return metrics, predictions, float(accuracy)


def score_run(paths: SpacePaths, run_dir: Path) -> dict[str, Any]:
    """Score only completed, matching saved replies; never call a model or fill missing answers."""
    run_dir = run_dir.resolve()
    examples, mode = _selection(paths, _object(run_dir / "manifest.json"))
    results = _results(run_dir, {example.case_id for example in examples})
    replay = Path(tempfile.mkdtemp(prefix="space-score-", dir=run_dir))
    cases: list[dict[str, Any]] = []
    eligible: list[Example] = []
    for example in examples:
        result = results.get(example.case_id)
        case = {**example.identity(), "status": "unreported", "official_accuracy": None}
        case["trace"] = _trace(run_dir, example, result)
        if result is not None:
            case["ended_by"], case["error"] = result["ended_by"], result["error"]
            if result["ended_by"] == "timeout":
                case["status"] = "timeout"
            elif result["error"] or result["ended_by"] != "answer":
                case["status"] = "infrastructure_error"
            elif not case["trace"]["valid"]:
                case["status"] = "infrastructure_error"
                case["error"] = case["trace"]["error"]
            else:
                case["status"] = "pending_score"
                eligible.append(example)
        cases.append(case)
    official_percent: float | None = None
    official_path: Path | None = None
    scoring_error: str | None = None
    if eligible:
        (replay / "questions.json").write_text(json.dumps([e.qa for e in eligible]))
        (replay / "replies.json").write_text(
            json.dumps({e.question_sha256: results[e.case_id]["final_answer"] for e in eligible})
        )
        try:
            official_path = _worker(paths, replay)
            metrics, predictions, official_percent = _official(official_path, len(eligible))
            for case, metric, prediction in zip(
                (case for case in cases if case["status"] == "pending_score"),
                metrics,
                predictions,
                strict=True,
            ):
                case["official_accuracy"] = metric["accuracy"]
                try:
                    json.dumps(prediction, allow_nan=False)
                except ValueError:
                    case["prediction_repr"] = repr(prediction)
                else:
                    case["prediction"] = prediction
                if prediction is None:
                    case["status"] = "parser_invalid"
                elif type(prediction) is not int or prediction not in (1, 2, 3, 4):
                    case["status"] = "invalid_choice"
                else:
                    case["status"] = "correct" if metric["accuracy"] == 100.0 else "wrong"
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            scoring_error = str(exc)
            official_percent = None
            for case in cases:
                if case["status"] == "pending_score":
                    case.update(status="infrastructure_error", error=scoring_error)
    counts = Counter(case["status"] for case in cases)
    scored = sum(case["official_accuracy"] is not None for case in cases)
    complete = scored == len(examples)
    report: dict[str, Any] = {
        "schema_version": 1,
        "task": TASK,
        "mode": mode,
        "evidence_label": "offline software contract" if mode == "offline-smoke" else mode,
        "space_revision": SPACE_REVISION,
        "data_sha256": DATA_SHA256,
        "run_dir": str(run_dir),
        "complete": complete,
        "fixed_subset_complete": complete and {e.index for e in examples} == set(SELECTED_INDICES),
        "selected_count": len(examples),
        "scored_denominator": scored,
        "official_percent": official_percent,
        "cases": cases,
        "counts": {status: counts[status] for status in _STATUSES},
        "scoring_error": scoring_error,
        "official_results_path": str(official_path) if official_path is not None else None,
        "report_path": str(replay / "report.json"),
    }
    Path(report["report_path"]).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report
