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

from .scheduler_models import (
    AttemptContext,
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    JobIdentity,
    NamedCondition,
    ReviewDecision,
    TerminalOutcome,
)
from .scheduler_plan import expand_plan, job_id, selected_inputs_digest, validate_plan
from .scheduler_store import FilesystemExperimentStore


def plan() -> ExperimentPlan:
    return expand_plan(
        "exp", [ExpandedCase(case_id="case", payload={})], [NamedCondition(name="condition")]
    )


def manifest_for(value: ExperimentPlan) -> ExperimentManifest:
    fingerprints = {
        name: "b" * 64
        for name in (
            "executor_fingerprint",
            "model_fingerprint",
            "prompt_fingerprint",
            "tools_fingerprint",
            "corpus_fingerprint",
            "runner_image_fingerprint",
            "scorer_fingerprint",
            "limits_fingerprint",
            "worker_fingerprint",
        )
    }
    return ExperimentManifest(
        experiment_id="exp",
        plan_digest=value.plan_digest,
        executor_kind="generic",
        executor_snapshot_digest="c" * 64,
        selected_inputs_digest=selected_inputs_digest(value),
        **fingerprints,
    )


def test_plan_rejects_self_attested_digest_and_duplicate_jobs() -> None:
    value = plan()
    invalid = value.model_copy(update={"plan_digest": "c" * 64})
    with pytest.raises(ValidationError):
        validate_plan(invalid)
    duplicate = value.model_copy(update={"jobs": (value.jobs[0], value.jobs[0])})
    with pytest.raises(ValidationError):
        validate_plan(duplicate)


def test_plan_expansion_rejects_duplicate_case_and_condition_names() -> None:
    case = ExpandedCase(case_id="case", payload={})
    condition = NamedCondition(name="condition")
    with pytest.raises(ValueError):
        expand_plan("exp", [case, case], [condition])
    with pytest.raises(ValueError):
        expand_plan("exp", [case], [condition, condition])


def test_attempt_outcome_and_manifest_snapshot_are_immutable(tmp_path) -> None:
    value = plan()
    store = FilesystemExperimentStore(tmp_path / "exp")
    store.create(manifest_for(value), value)
    identity = JobIdentity(
        experiment_id="exp",
        case_id="case",
        condition_name="condition",
        job_id=job_id(value, value.cases[0], value.conditions[0]),
    )
    context = AttemptContext(
        identity=identity, attempt_id="attempt-1", attempt_number=1, directory_name="attempt-1"
    )
    with store.coordinator_lock():
        store.create_attempt(context, value.cases[0], value.conditions[0])
        outcome = TerminalOutcome(status="failed", reason="executor_failed")
        store.write_outcome(context, outcome)
    with pytest.raises(FileExistsError):
        with store.coordinator_lock():
            store.write_outcome(context, TerminalOutcome(status="succeeded", reason="completed"))
    attempt = tmp_path / "exp" / "attempts" / identity.job_id / "attempt-1"
    assert (attempt / "attempt-manifest.v1.json").is_file()
    assert (attempt / "outcome.v1.json").is_file()


def test_review_decision_is_immutable_and_report_authorization_ready() -> None:
    decision = ReviewDecision(
        experiment_id="exp",
        manifest_digest="d" * 64,
        reviewer="reviewer@example.test",
        decision="approved",
        decided_at=datetime.now(timezone.utc),
    )
    with pytest.raises(ValidationError):
        ReviewDecision.model_validate(decision.model_dump(mode="python") | {"decision": "maybe"})
