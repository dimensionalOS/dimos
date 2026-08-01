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

"""Deterministic expansion and manifest drift checks."""

from __future__ import annotations

from collections.abc import Sequence
import hashlib
from typing import cast

from dimos.benchmark.spatial.utilities import JsonValue, canonical_json

from .scheduler_models import (
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    NamedCondition,
    PlanJob,
)


def _digest(value: JsonValue) -> str:
    return hashlib.sha256(canonical_json(cast("JsonValue", value))).hexdigest()


def canonical_manifest_bytes(manifest: ExperimentManifest) -> bytes:
    """Return the sole canonical byte representation used for manifest identity."""
    return canonical_json(manifest.model_dump(mode="json")) + b"\n"


def manifest_digest(value: ExperimentManifest | bytes) -> str:
    """Hash a canonical manifest model or verify and hash canonical on-disk bytes."""
    if isinstance(value, bytes):
        parsed = ExperimentManifest.model_validate_json(value)
        if value != canonical_manifest_bytes(parsed):
            raise ValueError("manifest bytes are not canonical")
        return hashlib.sha256(value).hexdigest()
    return hashlib.sha256(canonical_manifest_bytes(value)).hexdigest()


def expand_plan(
    experiment_id: str,
    cases: Sequence[ExpandedCase],
    conditions: Sequence[NamedCondition],
    *,
    workers: int = 10,
    sample: int | None = None,
    shard: int = 0,
    shards: int = 1,
) -> ExperimentPlan:
    """Expand cases x conditions in stable input order, with deterministic selection."""
    if workers <= 0 or shards <= 0 or not 0 <= shard < shards:
        raise ValueError("workers must be positive and shard must be within shards")
    if len({case.case_id for case in cases}) != len(cases):
        raise ValueError("case IDs must be unique")
    if len({condition.name for condition in conditions}) != len(conditions):
        raise ValueError("condition names must be unique")
    expanded = tuple((case, condition) for case in cases for condition in conditions)
    if not expanded:
        raise ValueError("at least one case and condition are required")
    selected = expanded[shard::shards]
    if sample is not None:
        if sample < 0:
            raise ValueError("sample must not be negative")
        selected = selected[:sample]
    if not selected:
        raise ValueError("selection produced an empty plan")
    case_ids: set[str] = set()
    condition_names: set[str] = set()
    selected_cases = tuple(
        case for case, _ in selected if not (case.case_id in case_ids or case_ids.add(case.case_id))
    )
    selected_conditions = tuple(
        condition
        for _, condition in selected
        if not (condition.name in condition_names or condition_names.add(condition.name))
    )
    jobs = tuple(
        PlanJob(case_id=case.case_id, condition_name=condition.name) for case, condition in selected
    )
    payload = {
        "experiment_id": experiment_id,
        "cases": [case.model_dump(mode="json") for case in selected_cases],
        "conditions": [condition.model_dump(mode="json") for condition in selected_conditions],
        "jobs": [job.model_dump(mode="json") for job in jobs],
        "workers": workers,
    }
    return ExperimentPlan(
        experiment_id=experiment_id,
        cases=selected_cases,
        conditions=selected_conditions,
        jobs=jobs,
        workers=workers,
        plan_digest=_digest(payload),
    )


def validate_plan(plan: ExperimentPlan) -> None:
    """Reparse and validate an immutable plan, including its self-attested digest."""
    ExperimentPlan.model_validate(plan.model_dump(mode="python"))


def selected_inputs_digest(plan: ExperimentPlan) -> str:
    """Digest only the immutable case and condition inputs selected for execution."""
    return _digest(
        {
            "cases": [case.model_dump(mode="json") for case in plan.cases],
            "conditions": [condition.model_dump(mode="json") for condition in plan.conditions],
        }
    )


def job_id(plan: ExperimentPlan, case: ExpandedCase, condition: NamedCondition) -> str:
    """Return a stable, artifact-safe job identity."""
    return _digest(
        {"experiment": plan.experiment_id, "case": case.case_id, "condition": condition.name}
    )


def validate_manifest(manifest: ExperimentManifest, plan: ExperimentPlan) -> None:
    validate_plan(plan)
    if manifest.experiment_id != plan.experiment_id or manifest.plan_digest != plan.plan_digest:
        raise ValueError("manifest does not match the recorded immutable plan")
    if manifest.workers != plan.workers:
        raise ValueError("worker count drift requires a fork")
    if manifest.selected_inputs_digest != selected_inputs_digest(plan):
        raise ValueError("manifest selected-inputs digest does not match the plan")
    required = (
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
    if any(not getattr(manifest, field) for field in required):
        raise ValueError("manifest is missing a required fingerprint")


def validate_fork(original: ExperimentManifest, candidate: ExperimentManifest) -> None:
    """Reject in-place execution-affecting changes; callers must create a new experiment."""
    if original != candidate:
        raise ValueError("execution-affecting manifest drift requires a fork")
