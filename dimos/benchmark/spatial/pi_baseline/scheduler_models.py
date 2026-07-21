"""Immutable, agent-neutral records used by the local experiment scheduler."""

from __future__ import annotations

from datetime import datetime
import hashlib
import re
from typing import Annotated, Literal, cast

from pydantic import Field, model_serializer, model_validator

from dimos.benchmark.spatial.models import SpatialModel
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json

Digest = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
SafeId = Annotated[str, Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9._-]*$")]
ExecutorKind = Literal["generic", "pi"]


class ExpandedCase(SpatialModel):
    case_id: SafeId
    payload: dict[str, object]
    fingerprint: Digest | None = None


class NamedCondition(SpatialModel):
    name: SafeId
    payload: dict[str, object] = Field(default_factory=dict)
    fingerprint: Digest | None = None


class JobIdentity(SpatialModel):
    experiment_id: SafeId
    case_id: SafeId
    condition_name: SafeId
    job_id: SafeId


class PlanJob(SpatialModel):
    case_id: SafeId
    condition_name: SafeId


class AttemptContext(SpatialModel):
    identity: JobIdentity
    attempt_id: SafeId
    attempt_number: int = Field(gt=0)
    directory_name: SafeId
    manifest_digest: Digest | None = None


class AttemptManifestSnapshot(SpatialModel):
    """The scheduler manifest captured when an attempt is created."""

    identity: JobIdentity
    attempt_id: SafeId
    manifest_digest: Digest
    manifest: dict[str, object]


class QuarantineMetadata(SpatialModel):
    """Typed reservation metadata for an original quarantined attempt."""

    quarantined_name: str = Field(min_length=34, max_length=34, pattern=r"^q-[0-9a-f]{32}$")
    original_name: str = Field(min_length=1, max_length=255, pattern=r"^[A-Za-z0-9._-]+$")
    original_attempt_number: int = Field(gt=0, le=1_000_000_000)


EventKind = Literal["created", "started", "progress", "artifact", "finished", "interrupted"]
OperationalEventKind = Literal["created", "progress", "artifact", "finished", "interrupted"]
OperationalCode = Literal[
    "scheduled",
    "retry_requested",
    "executor_progress",
    "artifact_recorded",
    "completed",
    "executor_failed",
    "executor_interrupted",
    "executor_cancelled",
    "container_cleanup_failed",
    "post_image_policy_violation",
    "pre_image_policy_violation",
    "coordinator_cancelled",
    "coordinator_restart",
    "missing_terminal_outcome",
]

PolicyFailureKind = Literal[
    "prompt_returned_without_image",
    "sandbox_limit_exceeded",
    "premature_submission",
    "image_attempt_failed",
    "malformed_image_call",
    "budget_exhausted_before_image",
    "image_attempts_exhausted",
    "image_retry_not_attempted",
]
ImageFailureCategory = Literal[
    "argument_invalid",
    "artifact_not_found_or_unsafe",
    "artifact_too_large",
    "not_png",
    "png_invalid",
    "dimensions_exceeded",
    "reply_delivery_failed",
    "adapter_result_rejected",
    "unexpected_infrastructure_error",
]


class PreImagePolicyTelemetry(SpatialModel):
    """Evaluation-neutral, public-safe encouraged-mode policy state."""

    policy_revision: Literal["pre-image-2-sandbox-plus-1-repair-2-image-post-image-1-submit-v1"]
    mode: Literal["visualization-encouraged"]
    terminal_code: Literal["pre_image_policy_violation"]
    failure_kind: PolicyFailureKind
    sandbox_attempts: int = Field(ge=0, le=3)
    image_attempted: bool
    image_attempts: int = Field(ge=0, le=2)
    image_delivered: bool
    submission_attempted: bool
    image_failure_category: ImageFailureCategory | None = None
    repair_attempted: bool = False

    @model_serializer(mode="wrap")
    def serialize_safe(self, handler):
        return {key: value for key, value in handler(self).items() if value is not None}

    @model_validator(mode="after")
    def validate_state(self) -> PreImagePolicyTelemetry:
        if self.image_attempts < int(self.image_attempted):
            raise ValueError("image attempts must include attempted image reads")
        if self.image_delivered and not self.image_attempted:
            raise ValueError("delivered image must have been attempted")
        if self.failure_kind == "sandbox_limit_exceeded" and self.sandbox_attempts != 2:
            raise ValueError("sandbox limit telemetry requires two attempts")
        if self.repair_attempted and self.sandbox_attempts < 3:
            raise ValueError("repair telemetry requires the third sandbox attempt")
        if self.failure_kind == "image_attempts_exhausted" and self.image_attempts != 2:
            raise ValueError("exhausted image telemetry requires two attempts")
        if self.failure_kind == "image_retry_not_attempted" and self.image_attempts != 1:
            raise ValueError("unattempted image retry telemetry requires one attempt")
        return self


class LifecycleEvent(SpatialModel):
    """Scheduler-owned lifecycle data; executors must not emit this type."""

    kind: EventKind
    occurred_at: datetime
    message: str = ""
    payload: dict[str, object] = Field(default_factory=dict)


class ExecutorProgressEvent(SpatialModel):
    kind: Literal["progress"]
    code: Literal["executor_progress"]


class ExecutorArtifactEvent(SpatialModel):
    kind: Literal["artifact"]
    code: Literal["artifact_recorded"]
    artifact_id: SafeId
    artifact_sha256: Digest


ExecutorEvent = Annotated[
    ExecutorProgressEvent | ExecutorArtifactEvent,
    Field(discriminator="kind"),
]


class OperationalEvent(SpatialModel):
    """Allowlisted scheduler event persisted on the public operational stream."""

    kind: OperationalEventKind
    occurred_at: datetime
    message: OperationalCode
    payload: dict[str, str | int | float | bool | None] = Field(default_factory=dict)
    policy_telemetry: PreImagePolicyTelemetry | None = Field(
        default=None, exclude_if=lambda value: value is None
    )

    @model_validator(mode="after")
    def validate_payload(self) -> OperationalEvent:
        allowed = {
            "created": {"reason_sha256"},
            "progress": set(),
            "artifact": {"artifact_id", "artifact_sha256"},
            "finished": {"status"},
            "interrupted": {"status"},
        }[self.kind]
        codes = {
            "created": {"scheduled", "retry_requested"},
            "progress": {"executor_progress"},
            "artifact": {"artifact_recorded"},
            "finished": {
                "completed",
                "executor_failed",
                "executor_interrupted",
                "executor_cancelled",
                "container_cleanup_failed",
                "post_image_policy_violation",
                "pre_image_policy_violation",
                "missing_terminal_outcome",
            },
            "interrupted": {"coordinator_cancelled", "coordinator_restart", "executor_interrupted"},
        }[self.kind]
        if self.message not in codes:
            raise ValueError("operational event code is not allowlisted")
        if self.message == "pre_image_policy_violation" and self.policy_telemetry is None:
            raise ValueError("pre-image policy event requires safe telemetry")
        if self.policy_telemetry is not None and self.message != "pre_image_policy_violation":
            raise ValueError("policy telemetry is only valid for pre-image policy events")
        if any(key not in allowed for key in self.payload):
            raise ValueError("operational event contains a non-allowlisted field")
        if self.kind == "created" and any(
            key == "reason_sha256"
            and (not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value))
            for key, value in self.payload.items()
        ):
            raise ValueError("created event reason must be a SHA-256 digest")
        if self.kind in {"finished", "interrupted"} and self.payload.get("status") not in {
            "succeeded",
            "failed",
            "interrupted",
            "cancelled",
        }:
            raise ValueError("terminal event status is invalid")
        if self.kind == "artifact" and any(
            key == "artifact_id"
            and (
                not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", value)
            )
            for key, value in self.payload.items()
        ):
            raise ValueError("artifact ID is invalid")
        if self.kind == "artifact" and any(
            key == "artifact_sha256"
            and (not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value))
            for key, value in self.payload.items()
        ):
            raise ValueError("artifact digest is invalid")
        return self


OutcomeStatus = Literal["succeeded", "failed", "interrupted", "cancelled"]


class TerminalOutcome(SpatialModel):
    status: OutcomeStatus
    reason: str
    exit_code: int | None = None
    policy_telemetry: PreImagePolicyTelemetry | None = Field(
        default=None, exclude_if=lambda value: value is None
    )


class ExperimentPlan(SpatialModel):
    experiment_id: SafeId
    cases: tuple[ExpandedCase, ...] = Field(min_length=1)
    conditions: tuple[NamedCondition, ...] = Field(min_length=1)
    jobs: tuple[PlanJob, ...] = Field(min_length=1)
    workers: int = Field(default=10, gt=0)
    plan_digest: Digest

    @model_validator(mode="after")
    def validate_integrity(self) -> ExperimentPlan:
        case_ids = [case.case_id for case in self.cases]
        condition_names = [condition.name for condition in self.conditions]
        if len(case_ids) != len(set(case_ids)):
            raise ValueError("plan contains duplicate case IDs")
        if len(condition_names) != len(set(condition_names)):
            raise ValueError("plan contains duplicate condition names")
        case_set = set(case_ids)
        condition_set = set(condition_names)
        references = [(job.case_id, job.condition_name) for job in self.jobs]
        if len(references) != len(set(references)):
            raise ValueError("plan contains duplicate case-condition jobs")
        if any(
            case_id not in case_set or condition_name not in condition_set
            for case_id, condition_name in references
        ):
            raise ValueError("plan contains an invalid job reference")
        payload = self.model_dump(mode="json")
        payload.pop("plan_digest", None)
        expected = hashlib.sha256(canonical_json(cast("JsonValue", payload))).hexdigest()
        if self.plan_digest != expected:
            raise ValueError("plan digest does not match canonical plan contents")
        return self


class ExperimentManifest(SpatialModel):
    experiment_id: SafeId
    plan_digest: Digest
    executor_kind: ExecutorKind
    executor_snapshot_digest: Digest
    selected_inputs_digest: Digest
    executor_fingerprint: Digest
    model_fingerprint: Digest
    prompt_fingerprint: Digest
    tools_fingerprint: Digest
    corpus_fingerprint: Digest
    runner_image_fingerprint: Digest
    scorer_fingerprint: Digest
    limits_fingerprint: Digest
    worker_fingerprint: Digest
    workers: int = Field(default=10, gt=0)


class JobSummary(SpatialModel):
    identity: JobIdentity
    state: Literal["pending", "running", "succeeded", "failed", "interrupted", "cancelled"]
    latest_attempt_id: SafeId | None = None
    outcome: TerminalOutcome | None = None


class OperationalCount(SpatialModel):
    """Counts reconstructed from the immutable attempt records."""

    pending: int = Field(ge=0)
    running: int = Field(ge=0)
    succeeded: int = Field(ge=0)
    failed: int = Field(ge=0)
    interrupted: int = Field(ge=0)
    cancelled: int = Field(ge=0)

    @model_validator(mode="after")
    def validate_total(self) -> OperationalCount:
        if (
            min(
                self.pending,
                self.running,
                self.succeeded,
                self.failed,
                self.interrupted,
                self.cancelled,
            )
            < 0
        ):
            raise ValueError("operational counts must be non-negative")
        return self


class OperationalFailure(SpatialModel):
    """Public-safe failure classification with no internal details."""

    job_id: SafeId
    state: Literal["failed", "interrupted", "cancelled"]
    reason: Literal[
        "executor_failed",
        "executor_interrupted",
        "executor_cancelled",
        "container_cleanup_failed",
        "post_image_policy_violation",
        "pre_image_policy_violation",
        "coordinator_cancelled",
        "coordinator_restart",
        "missing_terminal_outcome",
    ]
    policy_telemetry: PreImagePolicyTelemetry | None = None

    @model_serializer(mode="wrap")
    def serialize_public(self, handler):
        return {key: value for key, value in handler(self).items() if value is not None}


class OperationalSnapshot(SpatialModel):
    """Authoritative, public operational observation."""

    record_type: Literal["pi-operational-snapshot"] = "pi-operational-snapshot"
    schema_version: Literal["1.0"] = "1.0"
    experiment_id: SafeId
    workers: int = Field(gt=0)
    observation: Literal["reconciled", "busy_read_only", "busy_read_only_summary"]
    counts: OperationalCount
    jobs: int = Field(ge=0)
    active: int = Field(ge=0)
    failures: tuple[OperationalFailure, ...] = Field(max_length=3)

    @model_validator(mode="after")
    def validate_invariants(self) -> OperationalSnapshot:
        values = self.counts.model_dump()
        if self.jobs != sum(values.values()) or self.active != values["running"]:
            raise ValueError("operational snapshot counts are inconsistent")
        failure_ids = [failure.job_id for failure in self.failures]
        if len(failure_ids) != len(set(failure_ids)):
            raise ValueError("operational failures must identify distinct jobs")
        if failure_ids != sorted(failure_ids):
            raise ValueError("operational failures must be deterministically ordered")
        compatible = {
            "executor_failed": "failed",
            "executor_interrupted": "interrupted",
            "executor_cancelled": "cancelled",
            "container_cleanup_failed": "failed",
            "post_image_policy_violation": "failed",
            "pre_image_policy_violation": "failed",
            "coordinator_cancelled": "cancelled",
            "coordinator_restart": "interrupted",
            "missing_terminal_outcome": "interrupted",
        }
        for failure in self.failures:
            if values[failure.state] == 0:
                raise ValueError("failure state must have a non-zero snapshot count")
            if compatible[failure.reason] != failure.state:
                raise ValueError("failure reason is incompatible with failure state")
        return self


class ReviewDecision(SpatialModel):
    """Authorization record consumed by a later private report generator."""

    experiment_id: SafeId
    manifest_digest: Digest
    reviewer: str = Field(min_length=1)
    decision: Literal["approved", "rejected"]
    decided_at: datetime
