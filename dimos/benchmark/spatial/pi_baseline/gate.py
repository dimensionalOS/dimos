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

# Copyright 2026 Dimensional Inc.
"""Fail-closed records and release decision for the PI baseline smoke gate.

The network audit recorded here is deliberately only an observation.  It is
not a claim that the workload made no online requests.
"""

from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import Field

from dimos.benchmark.spatial.models import Sha256, SpatialModel

InfrastructureName = Literal[
    "oauth-model-resolution",
    "image-digest",
    "rootless-isolation",
    "read-only-input",
    "oracle-absence",
    "tool-digest",
    "network-availability",
    "audit-collection",
    "resource-limits",
    "submission-immutability",
    "scorer-isolation",
    "export-integrity",
    "destruction",
]
InfrastructureStatus = Literal["passed", "failed"]
SmokeMode = Literal["visualization-forbidden", "visualization-encouraged"]
ReviewerDecision = Literal["approved", "rejected"]

REQUIRED_INFRASTRUCTURE_CHECKS: tuple[InfrastructureName, ...] = (
    "oauth-model-resolution",
    "image-digest",
    "rootless-isolation",
    "read-only-input",
    "oracle-absence",
    "tool-digest",
    "network-availability",
    "audit-collection",
    "resource-limits",
    "submission-immutability",
    "scorer-isolation",
    "export-integrity",
    "destruction",
)
AUDIT_LIMITATION = "This audit cannot prove absence of online use."


class ArtifactReference(SpatialModel):
    """A retained, content-addressed review artifact."""

    path: str = Field(min_length=1)
    sha256: Sha256


class InfrastructureCheck(SpatialModel):
    """One explicit infrastructure result; omission is not a pass."""

    name: InfrastructureName
    status: InfrastructureStatus
    evidence: ArtifactReference | None = None
    detail: str = ""


class SmokeRunEvidence(SpatialModel):
    """Evidence references for one run of the fixed paired smoke case."""

    run_id: str = Field(min_length=1)
    mode: SmokeMode
    case_sha256: Sha256
    manifest_sha256: Sha256
    review_bundle: ArtifactReference | None = None
    private_score: ArtifactReference | None = None
    transcript: ArtifactReference | None = None
    tool_trace: ArtifactReference | None = None
    audit: ArtifactReference | None = None


class HumanReleaseRecord(SpatialModel):
    """The complete human-controlled release record."""

    record_type: Literal["pi-human-release"] = "pi-human-release"
    release_id: str = Field(min_length=1)
    infrastructure: tuple[InfrastructureCheck, ...]
    smoke_runs: tuple[SmokeRunEvidence, ...]
    required_review_artifact: ArtifactReference | None = None
    reviewer: str | None = None
    reviewed_at: datetime | None = None
    decision: ReviewerDecision | None = None
    blockers: tuple[str, ...] = ()
    audit_limitation: str = AUDIT_LIMITATION


class ReleaseEvaluation(SpatialModel):
    """Deterministic result of evaluating a human release record."""

    permitted: bool
    blockers: tuple[str, ...] = ()


class ReleaseBlockedError(ValueError):
    """Raised when a release record does not satisfy every gate requirement."""


def evaluate_release(record: HumanReleaseRecord) -> ReleaseEvaluation:
    """Evaluate the record without treating heuristic network auditing as proof."""
    blockers: list[str] = list(record.blockers)
    checks = {check.name: check for check in record.infrastructure}
    if len(checks) != len(record.infrastructure):
        blockers.append("duplicate infrastructure check")
    for name in REQUIRED_INFRASTRUCTURE_CHECKS:
        check = checks.get(name)
        if check is None:
            blockers.append(f"missing infrastructure check: {name}")
        elif check.status != "passed":
            blockers.append(f"failed infrastructure check: {name}")

    runs = {run.mode: run for run in record.smoke_runs}
    if len(runs) != len(record.smoke_runs):
        blockers.append("duplicate smoke mode")
    for mode in ("visualization-forbidden", "visualization-encouraged"):
        run = runs.get(mode)
        if run is None:
            blockers.append(f"missing smoke mode: {mode}")
        else:
            _require_run_artifacts(run, blockers)
    forbidden = runs.get("visualization-forbidden")
    encouraged = runs.get("visualization-encouraged")
    if forbidden is not None and encouraged is not None:
        if forbidden.case_sha256 != encouraged.case_sha256:
            blockers.append("smoke runs use different cases")
        if forbidden.mode == encouraged.mode:
            blockers.append("smoke runs do not have distinct modes")

    if record.required_review_artifact is None:
        blockers.append("missing required review artifact")
    if record.audit_limitation != AUDIT_LIMITATION:
        blockers.append("missing honest network-audit limitation")
    if record.decision != "approved":
        blockers.append("no explicit human approval")
    if not record.reviewer:
        blockers.append("missing reviewer")
    if record.reviewed_at is None:
        blockers.append("missing review timestamp")
    return ReleaseEvaluation(permitted=not blockers, blockers=tuple(dict.fromkeys(blockers)))


def require_release(record: HumanReleaseRecord) -> None:
    """Raise instead of permitting an incomplete or failed release record."""
    result = evaluate_release(record)
    if not result.permitted:
        raise ReleaseBlockedError("release blocked: " + "; ".join(result.blockers))


def _require_run_artifacts(run: SmokeRunEvidence, blockers: list[str]) -> None:
    for field, label in (
        (run.review_bundle, "review bundle"),
        (run.private_score, "private score reference"),
        (run.transcript, "transcript"),
        (run.tool_trace, "tool trace"),
        (run.audit, "audit"),
    ):
        if field is None:
            blockers.append(f"{run.mode} missing {label}")
