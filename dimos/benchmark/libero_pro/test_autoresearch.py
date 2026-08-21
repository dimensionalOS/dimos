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

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Literal

import pytest

from dimos.benchmark.evaluation.models import (
    EvaluationIdentity,
    EvaluationReport,
    EvaluationRun,
    EvaluationRunError,
    EvaluationRunSpecification,
    InlineNativeResult,
    RuntimeCondition,
    RuntimeIdentity,
)
import dimos.benchmark.libero_pro.autoresearch as autoresearch
from dimos.benchmark.libero_pro.autoresearch import (
    InvalidMeasurementError,
    PanelCase,
    ResearchPanel,
    publish_evo_result,
    run_panel,
)
from dimos.benchmark.libero_pro.models import LiberoTaskManifest

CASES = Path(__file__).parent / "cases" / "autoresearch"


def _panel(tmp_path: Path) -> Path:
    for name in ("goal", "spatial", "object", "long"):
        (tmp_path / f"{name}.json").write_text(json.dumps({"case": name}) + "\n")
    path = tmp_path / "panel.json"
    path.write_text(
        """{
  "schema_version": "1.0",
  "name": "test-panel",
  "cases": [
    {"id": "goal", "family": "goal", "specification": "goal.json"},
    {"id": "spatial", "family": "spatial", "specification": "spatial.json"},
    {"id": "object", "family": "object", "specification": "object.json"},
    {"id": "long", "family": "libero_10", "specification": "long.json"}
  ]
}
"""
    )
    return path


def _run(
    *,
    success: bool = False,
    score: float | None = None,
    status: Literal["completed", "failed"] = "completed",
) -> EvaluationRun:
    now = datetime.now(timezone.utc)
    specification = EvaluationRunSpecification(
        runtime=RuntimeCondition(model="model", thinking_level="medium"),
        evaluation={"name": "libero-pro", "config": {}},
    )
    common = {
        "run_id": "run",
        "specification": specification,
        "evaluation": EvaluationIdentity(name="libero-pro", provider="dimos", version="1"),
        "runtime": RuntimeIdentity(
            profile="code-policy-v1",
            driver="pi",
            driver_version="1",
            model="model",
            thinking_level="medium",
        ),
        "started_at": now,
        "finished_at": now,
        "duration_seconds": 0.0,
    }
    if status == "completed":
        return EvaluationRun(
            **common,
            status="completed",
            report=EvaluationReport(
                native_result=InlineNativeResult(
                    value={"success": success, "score": float(success) if score is None else score}
                )
            ),
        )
    return EvaluationRun(
        **common,
        status="failed",
        error=EvaluationRunError(stage="evaluation", error_type="Failure", message="failed"),
    )


def test_panel_counts_only_native_successes_and_continues_failures(tmp_path: Path) -> None:
    calls = 0

    def execute(*args: object, **kwargs: object) -> EvaluationRun:
        nonlocal calls
        del args, kwargs
        calls += 1
        if calls == 1:
            return _run(success=True)
        if calls == 2:
            return _run(status="failed")
        if calls == 3:
            raise RuntimeError("preflight")
        return _run(success=False)

    output = tmp_path / "output"
    result = run_panel(_panel(tmp_path), output=output, executor=execute)

    assert result.native_successes == 1
    assert result.attempts == 4
    assert result.macro_score == 0.25
    assert result.infrastructure_or_policy_failures == 2
    assert [case.status for case in result.cases] == [
        "completed",
        "failed",
        "preflight_error",
        "completed",
    ]
    assert (output / "panel-score.json").is_file()


def test_panel_allows_configurable_size_and_families(tmp_path: Path) -> None:
    payload = ResearchPanel.model_validate_json(_panel(tmp_path).read_bytes()).model_dump()
    payload["cases"] = payload["cases"][:1]
    payload["cases"][0]["family"] = "custom-suite"

    panel = ResearchPanel.model_validate(payload)

    assert len(panel.cases) == 1
    assert panel.cases[0].family == "custom-suite"


def test_panel_score_is_macro_average_of_native_scores(tmp_path: Path) -> None:
    scores = iter((0.25, 0.5, 0.75, 1.0))

    result = run_panel(
        _panel(tmp_path),
        output=tmp_path / "output",
        executor=lambda *_args, **_kwargs: _run(score=next(scores)),
    )

    assert result.macro_score == 0.625


def test_panel_rejects_empty_duplicate_and_unsafe_cases() -> None:
    with pytest.raises(ValueError, match="at least one"):
        ResearchPanel(name="empty", cases=())
    duplicate = PanelCase(id="same", family="a", specification="a.json")
    with pytest.raises(ValueError, match="unique"):
        ResearchPanel(name="duplicate", cases=(duplicate, duplicate))
    with pytest.raises(ValueError, match="safe relative"):
        PanelCase(id="unsafe", family="a", specification="../a.json")


def test_panel_hash_changes_with_resolved_specification(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    first = run_panel(panel, output=tmp_path / "first", executor=lambda *_args, **_kwargs: _run())
    (tmp_path / "goal.json").write_text('{"case": "changed"}\n')
    second = run_panel(panel, output=tmp_path / "second", executor=lambda *_args, **_kwargs: _run())

    assert first.panel_hash != second.panel_hash


def test_evo_publishes_atomic_score_tasks_and_traces(tmp_path: Path) -> None:
    outcomes = iter((True, False, True, False))

    def execute(*_args: object, **kwargs: object) -> EvaluationRun:
        case_output = Path(str(kwargs["output"]))
        trial = case_output / "runtime" / "trial"
        trial.mkdir(parents=True)
        for name in ("policy.py", "main.jsonl", "trial.mp4", "recording.db", "score.json"):
            (trial / name).write_text(name)
        return _run(success=next(outcomes))

    score = run_panel(
        _panel(tmp_path),
        output=tmp_path / "panel-output",
        executor=execute,
    )
    result_path = tmp_path / "evo" / "result.json"
    traces = tmp_path / "evo" / "traces"

    publish_evo_result(
        score,
        result_path=result_path,
        traces_dir=traces,
        experiment_id="experiment-1",
    )

    result = json.loads(result_path.read_text())
    assert result["score"] == 0.5
    assert result["tasks"] == {"goal": 1.0, "spatial": 0.0, "object": 1.0, "long": 0.0}
    assert result["panel_hash"] == score.panel_hash
    assert {path.stem for path in traces.glob("*.json")} == {
        "task_goal",
        "task_spatial",
        "task_object",
        "task_long",
    }
    trace = json.loads((traces / "task_spatial.json").read_text())
    assert trace["failure_reason"] == "native_failure"
    assert trace["score"] == 0.0
    assert trace["status"] == "failed"
    assert trace["runtime"]["profile"] == "code-policy-v1"
    assert Path(trace["artifacts"]["policy"][0]).is_absolute()
    assert Path(trace["artifacts"]["videos"][0]).name == "trial.mp4"
    with pytest.raises(FileExistsError, match="already claimed"):
        publish_evo_result(
            score,
            result_path=result_path,
            traces_dir=traces,
            experiment_id="experiment-2",
        )


def test_evo_rejects_invalid_measurement_but_preserves_trace(tmp_path: Path) -> None:
    score = run_panel(
        _panel(tmp_path),
        output=tmp_path / "panel-output",
        executor=lambda *_args, **_kwargs: _run(status="failed"),
    )
    result_path = tmp_path / "result.json"
    traces = tmp_path / "traces"

    with pytest.raises(InvalidMeasurementError, match="invalid panel measurement"):
        publish_evo_result(
            score,
            result_path=result_path,
            traces_dir=traces,
            experiment_id="unknown",
        )

    assert not result_path.exists()
    assert json.loads((traces / "task_goal.json").read_text())["failure_reason"] == (
        "measurement_failure"
    )


def test_atomic_publication_failure_does_not_claim_final_path(mocker, tmp_path: Path) -> None:
    result_path = tmp_path / "result.json"
    mocker.patch.object(autoresearch.os, "link", side_effect=OSError("link failed"))

    with pytest.raises(OSError, match="link failed"):
        autoresearch._atomic_publish_json(result_path, {"score": 1.0})

    assert not result_path.exists()
    assert not tuple(tmp_path.glob(".*.tmp"))


@pytest.mark.parametrize(
    ("panel_name", "expected_horizons"),
    [
        ("dev-panel.json", {"goal": 300, "spatial": 220, "object": 280, "libero_10": 520}),
        (
            "heldout-panel.json",
            {"goal": 300, "spatial": 220, "object": 280, "libero_10": 520},
        ),
    ],
)
def test_checked_in_panel_freezes_representative_contracts(
    panel_name: str,
    expected_horizons: dict[str, int],
) -> None:
    panel = ResearchPanel.model_validate_json((CASES / panel_name).read_bytes())

    for case in panel.cases:
        specification_path = CASES / case.specification
        specification = EvaluationRunSpecification.model_validate_json(
            specification_path.read_bytes()
        )
        manifest_path = specification_path.parent / specification.evaluation.config["task_manifest"]
        manifest = LiberoTaskManifest.model_validate_json(manifest_path.read_bytes())

        assert specification.runtime == RuntimeCondition(
            model="gpt-5.6-luna", thinking_level="medium"
        )
        expected_suite = (
            "libero_10_task" if case.family == "libero_10" else f"libero_{case.family}_task"
        )
        assert manifest.task.suite == expected_suite
        assert manifest.contract.horizon_ticks == expected_horizons[case.family]
        assert manifest.episodes.debug_init_state_indices == (1, 2, 3, 4, 5)
        assert manifest.episodes.scored_init_state_index == 0


def test_main_bootstraps_through_shared_evaluation_container(mocker, tmp_path: Path) -> None:
    panel = CASES / "dev-panel.json"
    baked_panel = Path("/opt/dimos/dimos/benchmark/libero_pro/cases/autoresearch/dev-panel.json")
    output = tmp_path / "panel-output"
    mocker.patch.object(autoresearch.container, "inside_container", return_value=False)
    mocker.patch.object(autoresearch.container, "candidate_path", return_value=baked_panel)
    run = mocker.patch.object(autoresearch.container, "run_in_container", return_value=0)

    autoresearch.main(
        [
            "--panel",
            str(panel),
            "--output",
            str(output),
            "--api-key-env",
            "TEST_API_KEY",
            "--json",
        ]
    )

    command = run.call_args.args[0](Path("/staging/result"))
    assert command == [
        "/opt/dimos/.venv/bin/python",
        "-m",
        "dimos.benchmark.libero_pro.autoresearch",
        "--panel",
        str(baked_panel),
        "--output",
        "/staging/result",
        "--api-key-env",
        "TEST_API_KEY",
        "--json",
    ]
    assert run.call_args.kwargs == {
        "output": output,
        "input_directories": (),
        "forwarded_environment": (
            "TEST_API_KEY",
            "EVO_RESULT_PATH",
            "EVO_TRACES_DIR",
            "EVO_EXPERIMENT_ID",
        ),
    }
