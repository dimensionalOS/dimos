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
from dimos.benchmark.libero_pro.autoresearch import ResearchPanel, run_panel
from dimos.benchmark.libero_pro.models import LiberoTaskManifest

CASES = Path(__file__).parent / "cases" / "autoresearch"


def _panel(tmp_path: Path) -> Path:
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
                    value={"success": success, "score": float(success)}
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
    assert result.success_rate == 0.25
    assert result.infrastructure_or_policy_failures == 2
    assert [case.status for case in result.cases] == [
        "completed",
        "failed",
        "preflight_error",
        "completed",
    ]
    assert (output / "panel-score.json").is_file()


def test_panel_requires_one_case_per_family(tmp_path: Path) -> None:
    payload = ResearchPanel.model_validate_json(_panel(tmp_path).read_bytes()).model_dump()
    payload["cases"][3]["family"] = "goal"

    with pytest.raises(ValueError, match="one case from each suite family"):
        ResearchPanel.model_validate(payload)


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
