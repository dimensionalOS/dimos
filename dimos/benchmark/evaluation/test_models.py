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

from pydantic import ValidationError
import pytest

from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    EvaluationIdentity,
    EvaluationReference,
    EvaluationReport,
    EvaluationRun,
    EvaluationRunSpecification,
    InlineNativeResult,
    RuntimeIdentity,
)


def test_artifact_reference_rejects_paths_outside_run() -> None:
    with pytest.raises(ValidationError, match="safe relative"):
        ArtifactReference(path="../private.json", label="Private")


def test_completed_run_requires_evaluation_report() -> None:
    now = datetime.now(timezone.utc)

    with pytest.raises(ValidationError, match="require a report"):
        EvaluationRun(
            run_id="run",
            specification=EvaluationRunSpecification(
                evaluation=EvaluationReference(name="fixture")
            ),
            evaluation=EvaluationIdentity(name="fixture", provider="tests", version="1"),
            runtime=RuntimeIdentity(
                driver_version="test",
                model="gpt-5.6-luna",
                thinking_level="medium",
            ),
            status="completed",
            started_at=now,
            finished_at=now,
            duration_seconds=0,
        )


def test_native_result_preserves_nested_benchmark_payload() -> None:
    payload = {"metrics": {"success_rate": 0.75}, "episodes": [True, False]}

    report = EvaluationReport(native_result=InlineNativeResult(value=payload))

    assert report.model_dump(mode="json")["native_result"]["value"] == payload
