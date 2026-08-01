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

import pytest

from .scheduler_models import (
    AttemptContext,
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    JobIdentity,
    JobSummary,
    NamedCondition,
    OperationalEvent,
)
from .scheduler_plan import expand_plan, job_id, selected_inputs_digest, validate_manifest
from .scheduler_store import FilesystemExperimentStore


def _plan() -> ExperimentPlan:
    return expand_plan(
        "exp",
        [
            ExpandedCase(case_id="case-1", payload={"n": 1}),
            ExpandedCase(case_id="case-2", payload={"n": 2}),
        ],
        [NamedCondition(name="plain"), NamedCondition(name="visual")],
    )


def _context() -> AttemptContext:
    value = _plan()
    identity = JobIdentity(
        experiment_id="exp",
        case_id="case-1",
        condition_name="plain",
        job_id=job_id(value, value.cases[0], value.conditions[0]),
    )
    return AttemptContext(
        identity=identity, attempt_id="attempt-1", attempt_number=1, directory_name="attempt-1"
    )


def test_plan_hash_and_expansion_are_deterministic() -> None:
    first = _plan()
    second = _plan()
    assert first == second
    assert len(first.jobs) == 4
    assert first.workers == 10


def test_plan_rejects_drift() -> None:
    plan = _plan()
    manifest = ExperimentManifest(
        **{
            name: "a" * 64
            for name in (
                "plan_digest",
                "executor_snapshot_digest",
                "selected_inputs_digest",
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
        },
        experiment_id="exp",
        executor_kind="generic",
    )
    with pytest.raises(ValueError):
        validate_manifest(manifest, plan)


def test_filesystem_attempts_events_and_atomic_summary(tmp_path) -> None:
    store = FilesystemExperimentStore(tmp_path / "exp")
    plan = _plan()
    manifest = ExperimentManifest(
        **{
            name: "a" * 64
            for name in (
                "executor_snapshot_digest",
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
        },
        experiment_id="exp",
        plan_digest=plan.plan_digest,
        selected_inputs_digest=selected_inputs_digest(plan),
        executor_kind="generic",
    )
    store.create(manifest, plan)
    context = _context()
    with store.coordinator_lock():
        store.create_attempt(context, plan.cases[0], plan.conditions[0])
        event = OperationalEvent(
            kind="created", occurred_at=datetime.now(timezone.utc), message="scheduled"
        )
        store.append_event(context, event)
        store.append_event(
            context,
            OperationalEvent(
                kind="progress", occurred_at=datetime.now(timezone.utc), message="executor_progress"
            ),
        )
    assert [item.message for item in store.events(context)] == ["scheduled", "executor_progress"]
    summary = JobSummary(
        identity=context.identity, state="running", latest_attempt_id=context.attempt_id
    )
    with store.coordinator_lock():
        store.write_summary(summary)
    assert store.summaries("exp")[0] == summary
    with pytest.raises(FileExistsError):
        with store.coordinator_lock():
            store.create_attempt(context, plan.cases[0], plan.conditions[0])
