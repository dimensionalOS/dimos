"""Non-blocking, candidate-only visual verification gateway."""

from __future__ import annotations

import base64
from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
from threading import Event, RLock, Thread, Timer
import time
from typing import Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field

from dimos.perception.target_verification import (
    CandidateEvidenceBundle,
    TargetVerification,
    VisionVerifier,
    apply_verification_policy,
)

VisionJobState = Literal["pending", "completed"]


class VisionGatewayError(RuntimeError):
    """Base class for fail-closed gateway admission errors."""


class VisionGatewayBusyError(VisionGatewayError):
    """Raised when all bounded provider worker slots are occupied."""


class VisionGatewayConflictError(VisionGatewayError):
    """Raised when one candidate ID is reused with different evidence."""


class VisionGatewayJobNotFoundError(VisionGatewayError):
    """Raised when a caller requests an unknown job ID."""


class ProviderMetadata(BaseModel):
    """Sanitized provider execution metadata without image content."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    provider: str = Field(min_length=1, max_length=200)
    model: str = Field(min_length=1, max_length=200)
    selected_frame_ids: list[str] = Field(min_length=1, max_length=3)
    timeout_s: float = Field(gt=0.0, le=300.0)
    gateway_invocations: int = Field(ge=0, le=1)
    submitted_at: float = Field(ge=0.0)
    completed_at: float | None = Field(default=None, ge=0.0)
    latency_ms: float | None = Field(default=None, ge=0.0)
    error_code: Literal["provider_timeout", "provider_exception"] | None = None
    error_type: str | None = Field(default=None, max_length=200)


class VisionJobSnapshot(BaseModel):
    """Immutable public state for one candidate verification job."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    job_id: str = Field(min_length=1, max_length=200)
    candidate_id: str = Field(min_length=1, max_length=200)
    state: VisionJobState
    verification: TargetVerification | None
    provider: ProviderMetadata


@dataclass
class _VisionJob:
    fingerprint: str
    submitted_monotonic: float
    deadline_monotonic: float
    snapshot: VisionJobSnapshot
    completed: Event
    provider_running: bool = True
    timer: Timer | None = None


class CandidateVisionGateway:
    """Run a bounded provider call away from the local robot-control thread."""

    def __init__(
        self,
        verifier: VisionVerifier,
        *,
        provider: str,
        model: str,
        timeout_s: float = 8.0,
        max_inflight: int = 1,
        monotonic_clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        normalized_provider = provider.strip()
        normalized_model = model.strip()
        if not normalized_provider:
            raise ValueError("provider must not be empty")
        if not normalized_model:
            raise ValueError("model must not be empty")
        if timeout_s <= 0 or timeout_s > 300:
            raise ValueError("timeout_s must be between zero and 300 seconds")
        if max_inflight < 1 or max_inflight > 32:
            raise ValueError("max_inflight must be between one and 32")

        self._verifier = verifier
        self.provider = normalized_provider
        self.model = normalized_model
        self.timeout_s = float(timeout_s)
        self.max_inflight = max_inflight
        self._monotonic_clock = monotonic_clock
        self._wall_clock = wall_clock
        self._lock = RLock()
        self._jobs: dict[str, _VisionJob] = {}
        self._candidate_jobs: dict[str, tuple[str, str]] = {}

    def submit(self, bundle: CandidateEvidenceBundle) -> VisionJobSnapshot:
        """Admit one candidate and return before the provider finishes."""

        fingerprint = _bundle_fingerprint(bundle)
        with self._lock:
            existing = self._candidate_jobs.get(bundle.candidate_id)
            if existing is not None:
                existing_fingerprint, job_id = existing
                if existing_fingerprint != fingerprint:
                    raise VisionGatewayConflictError(
                        "candidate_id was already submitted with different evidence"
                    )
                return self._snapshot_locked(job_id)

            inflight = sum(job.provider_running for job in self._jobs.values())
            if inflight >= self.max_inflight:
                raise VisionGatewayBusyError(
                    "candidate vision gateway has no free provider slot"
                )

            job_id = f"vision-{uuid4()}"
            submitted_monotonic = self._monotonic_clock()
            submitted_at = max(0.0, self._wall_clock())
            selected_frame_ids = [view.frame_id for view in bundle.views]
            pending = VisionJobSnapshot(
                job_id=job_id,
                candidate_id=bundle.candidate_id,
                state="pending",
                verification=None,
                provider=ProviderMetadata(
                    provider=self.provider,
                    model=self.model,
                    selected_frame_ids=selected_frame_ids,
                    timeout_s=self.timeout_s,
                    gateway_invocations=0,
                    submitted_at=submitted_at,
                ),
            )
            job = _VisionJob(
                fingerprint=fingerprint,
                submitted_monotonic=submitted_monotonic,
                deadline_monotonic=submitted_monotonic + self.timeout_s,
                snapshot=pending,
                completed=Event(),
            )
            self._jobs[job_id] = job
            self._candidate_jobs[bundle.candidate_id] = (fingerprint, job_id)

            timer = Timer(self.timeout_s, self._expire_job, args=(job_id,))
            timer.daemon = True
            job.timer = timer
            worker = Thread(
                target=self._run_provider,
                args=(job_id, bundle),
                name=f"candidate-vision-{job_id[-8:]}",
                daemon=True,
            )
            timer.start()
            worker.start()
            return pending

    def get(self, job_id: str) -> VisionJobSnapshot:
        """Return current state, expiring a provider call past its deadline."""

        with self._lock:
            job = self._job_locked(job_id)
            if (
                job.snapshot.state == "pending"
                and self._monotonic_clock() >= job.deadline_monotonic
            ):
                self._expire_locked(job)
            return job.snapshot

    def wait(self, job_id: str, *, timeout_s: float) -> VisionJobSnapshot:
        """Optionally wait for a result; robot control must use ``submit`` only."""

        if timeout_s < 0:
            raise ValueError("timeout_s must not be negative")
        with self._lock:
            completed = self._job_locked(job_id).completed
        completed.wait(timeout=timeout_s)
        return self.get(job_id)

    def _run_provider(
        self,
        job_id: str,
        bundle: CandidateEvidenceBundle,
    ) -> None:
        error_code: Literal["provider_exception"] | None = None
        error_type: str | None = None
        try:
            raw_result = self._verifier.verify(bundle)
            if not isinstance(raw_result, TargetVerification):
                raise TypeError("provider returned an invalid verification type")
            result = apply_verification_policy(bundle, raw_result)
        except Exception as error:
            error_code = "provider_exception"
            error_type = type(error).__name__
            result = TargetVerification.uncertain(
                f"provider raised {error_type}"
            )

        with self._lock:
            job = self._jobs.get(job_id)
            if job is None:
                return
            job.provider_running = False
            if job.snapshot.state == "completed":
                return
            if self._monotonic_clock() >= job.deadline_monotonic:
                self._expire_locked(job)
                return
            self._complete_locked(
                job,
                result,
                error_code=error_code,
                error_type=error_type,
            )

    def _expire_job(self, job_id: str) -> None:
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job.snapshot.state == "completed":
                return
            self._expire_locked(job)

    def _expire_locked(self, job: _VisionJob) -> None:
        self._complete_locked(
            job,
            TargetVerification.uncertain("provider deadline exceeded"),
            error_code="provider_timeout",
            error_type="TimeoutError",
        )

    def _complete_locked(
        self,
        job: _VisionJob,
        verification: TargetVerification,
        *,
        error_code: Literal["provider_timeout", "provider_exception"] | None,
        error_type: str | None,
    ) -> None:
        if job.snapshot.state == "completed":
            return
        completed_monotonic = self._monotonic_clock()
        completed_at = max(0.0, self._wall_clock())
        latency_ms = max(
            0.0,
            (completed_monotonic - job.submitted_monotonic) * 1000.0,
        )
        original = job.snapshot.provider
        job.snapshot = VisionJobSnapshot(
            job_id=job.snapshot.job_id,
            candidate_id=job.snapshot.candidate_id,
            state="completed",
            verification=verification,
            provider=ProviderMetadata(
                provider=original.provider,
                model=original.model,
                selected_frame_ids=list(original.selected_frame_ids),
                timeout_s=original.timeout_s,
                gateway_invocations=1,
                submitted_at=original.submitted_at,
                completed_at=completed_at,
                latency_ms=round(latency_ms, 3),
                error_code=error_code,
                error_type=error_type,
            ),
        )
        if job.timer is not None:
            job.timer.cancel()
        job.completed.set()

    def _snapshot_locked(self, job_id: str) -> VisionJobSnapshot:
        job = self._job_locked(job_id)
        if (
            job.snapshot.state == "pending"
            and self._monotonic_clock() >= job.deadline_monotonic
        ):
            self._expire_locked(job)
        return job.snapshot

    def _job_locked(self, job_id: str) -> _VisionJob:
        job = self._jobs.get(job_id)
        if job is None:
            raise VisionGatewayJobNotFoundError(
                f"unknown candidate vision job: {job_id}"
            )
        return job


def _bundle_fingerprint(bundle: CandidateEvidenceBundle) -> str:
    payload = {
        "candidate_id": bundle.candidate_id,
        "target_description": bundle.target_description,
        "views": [
            {
                "frame_id": view.frame_id,
                "image_sha256": hashlib.sha256(
                    base64.b64decode(view.jpeg_base64)
                ).hexdigest(),
                "width": view.width,
                "height": view.height,
                "mime_type": view.mime_type,
            }
            for view in bundle.views
        ],
    }
    serialized = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()
