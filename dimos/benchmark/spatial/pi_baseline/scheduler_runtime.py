"""Bounded, single-coordinator runtime for the filesystem experiment store."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from concurrent.futures import Future, ThreadPoolExecutor, wait
from datetime import datetime, timezone
import hashlib
import shutil
import threading
from typing import cast

from pydantic import BaseModel, TypeAdapter

from .scheduler_executor import ExecutionInterrupted, Executor
from .scheduler_models import (
    AttemptContext,
    ExecutorArtifactEvent,
    ExecutorEvent,
    ExecutorProgressEvent,
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    JobIdentity,
    JobSummary,
    NamedCondition,
    OperationalCode,
    OperationalEvent,
    OperationalFailure,
    TerminalOutcome,
)
from .scheduler_operational import collect_operational_snapshot, reconcile_idle_locked
from .scheduler_plan import job_id, validate_manifest
from .scheduler_store import (
    CoordinatorLeaseCapability,
    CoordinatorLockError,
    FilesystemExperimentStore,
)

_EXECUTOR_EVENT_ADAPTER = TypeAdapter(ExecutorEvent)


class PreflightError(RuntimeError):
    """Raised when execution cannot safely begin."""


class SchedulerRuntime:
    """Own scheduling state for exactly one experiment and one local coordinator."""

    def __init__(
        self,
        store: FilesystemExperimentStore,
        executor: Executor,
        *,
        prerequisite: Callable[[], bool | None] | None = None,
        minimum_free_bytes: int = 0,
        diagnostic: Callable[[object], None] | None = None,
    ) -> None:
        self.store = store
        self.executor = executor
        self.prerequisite = prerequisite
        self.minimum_free_bytes = minimum_free_bytes
        self.diagnostic = diagnostic
        self._cancel_requested = threading.Event()
        self._publication_lock = threading.Lock()
        self.manifest, self.plan = self._load_definition()
        validate_manifest(self.manifest, self.plan)
        self._summaries = {
            identifier: JobSummary(identity=identity, state="pending")
            for identifier, identity in self._planned_identities()
        }

    def preflight(self) -> None:
        """Fail closed before admitting any executor work."""
        try:
            definition = self.store.load_definition()
            validate_manifest(definition.manifest, definition.plan)
        except (ValueError, OSError) as error:
            raise PreflightError(str(error)) from error
        if self.minimum_free_bytes < 0:
            raise PreflightError("minimum free disk budget must not be negative")
        usage = shutil.disk_usage(self.store.root)
        if usage.free < self.minimum_free_bytes:
            raise PreflightError("insufficient free disk budget")
        if self.prerequisite is not None:
            try:
                available = self.prerequisite()
            except Exception as error:
                raise PreflightError("executor prerequisite failed") from error
            if available is False:
                raise PreflightError("executor prerequisite is unavailable")

    def run(self) -> tuple[JobSummary, ...]:
        """Run jobs that are pending; terminal jobs are never silently rerun."""
        return internal_operation(self, "run")

    def resume(self) -> tuple[JobSummary, ...]:
        """Resume only pending or interrupted jobs from immutable state."""
        return internal_operation(self, "resume")

    def cancel(self) -> None:
        """Stop admitting work; active executors are allowed to clean up."""
        with self._publication_lock:
            self._cancel_requested.set()

    def retry(self, job_id_value: str, reason: str) -> JobSummary:
        """Create and execute a fresh attempt, retaining every previous artifact."""
        return internal_operation(self, "retry", job_id_value=job_id_value, reason=reason)[0]

    def summaries(self) -> tuple[JobSummary, ...]:
        """Return summaries rebuilt from the authoritative artifact state."""
        try:
            with self.store.coordinator_lease():
                self._reload_reconcile_locked()
                return tuple(self._summaries.values())
        except CoordinatorLockError:
            collect_operational_snapshot(self.store)
            definition, attempts = self.store.observe_experiment_read_only(None, None)
            result: list[JobSummary] = []
            for identifier, identity in self._planned_identities_for(
                definition.manifest, definition.plan
            ):
                records = attempts.get(identifier, ())
                latest = max(
                    records, key=lambda record: record.context.attempt_number, default=None
                )
                if latest is None:
                    result.append(JobSummary(identity=identity, state="pending"))
                elif latest.outcome is None:
                    result.append(
                        JobSummary(
                            identity=identity,
                            state="running",
                            latest_attempt_id=latest.context.attempt_id,
                        )
                    )
                else:
                    result.append(
                        JobSummary(
                            identity=identity,
                            state=latest.outcome.status,
                            latest_attempt_id=latest.context.attempt_id,
                            outcome=latest.outcome,
                        )
                    )
            return tuple(result)

    def recover(self) -> tuple[JobSummary, ...]:
        """Explicitly recover and reconcile attempt state under the coordinator lease."""
        with self.store.coordinator_lease():
            self._reload_reconcile_locked()
            return tuple(self._summaries.values())

    def retryable(
        self, statuses: Iterable[str] = ("failed", "interrupted")
    ) -> tuple[JobSummary, ...]:
        allowed = frozenset(statuses)
        return tuple(summary for summary in self.summaries() if summary.state in allowed)

    def triage(self) -> tuple[OperationalFailure, ...]:
        """Return public-safe failure classifications from reconstruction."""
        return tuple(collect_operational_snapshot(self.store).failures)

    def operational_status(self) -> dict[str, object]:
        """Return lifecycle health only; correctness and score fields are not represented."""
        snapshot = collect_operational_snapshot(self.store)
        counts = snapshot.counts.model_dump()
        return {
            "experiment_id": snapshot.experiment_id,
            "workers": snapshot.workers,
            "jobs": snapshot.jobs,
            "states": counts,
            "cancel_requested": self._cancel_requested.is_set(),
        }

    def _execute(
        self,
        job_ids: Iterable[str],
        *,
        allowed_states: set[str],
        capability: CoordinatorLeaseCapability,
        retry_reason: str | None = None,
    ) -> tuple[JobSummary, ...]:
        selected = tuple(job_ids)
        pending: list[str] = []
        for identifier in selected:
            summary = self._summaries[identifier]
            if retry_reason is not None:
                pending.append(identifier)
                continue
            if summary.state in allowed_states:
                pending.append(identifier)

        active: dict[Future[JobSummary], str] = {}
        with ThreadPoolExecutor(
            max_workers=self.manifest.workers, thread_name_prefix="spatial-scheduler"
        ) as pool:
            remaining = iter(pending)
            while active or (not self._cancel_requested.is_set() and pending):
                while not self._cancel_requested.is_set() and len(active) < self.manifest.workers:
                    try:
                        identifier = next(remaining)
                    except StopIteration:
                        break
                    future = pool.submit(self._run_one, identifier, retry_reason, capability)
                    active[future] = identifier
                if not active:
                    break
                done, _ = wait(tuple(active), return_when="FIRST_COMPLETED")
                for future in done:
                    identifier = active.pop(future)
                    try:
                        self._summaries[identifier] = future.result()
                    except Exception:
                        self._summaries[identifier] = self._record_failure(identifier, capability)

        return tuple(self._summaries[identifier] for identifier in selected)

    def _run_one(
        self,
        identifier: str,
        retry_reason: str | None,
        capability: CoordinatorLeaseCapability,
    ) -> JobSummary:
        with self.store.lease_mutation(capability):
            disk_summary = next(
                (
                    summary
                    for summary in self.store.summaries(self.manifest.experiment_id)
                    if summary.identity.job_id == identifier
                ),
                self._summaries[identifier],
            )
            allowed = (
                {"failed", "interrupted", "cancelled"}
                if retry_reason is not None
                else {"pending", "interrupted"}
            )
            if disk_summary.state not in allowed:
                return disk_summary
            old = disk_summary
            case, condition = self._case_condition(identifier)
            if self._cancel_requested.is_set():
                return old
            attempt_number = self._next_attempt_number(identifier)
            attempt_id = f"attempt-{attempt_number}"
            context = AttemptContext(
                identity=old.identity,
                attempt_id=attempt_id,
                attempt_number=attempt_number,
                directory_name=attempt_id,
                manifest_digest=self._manifest_digest(),
            )
            self.store.create_attempt(context, case, condition)
            self.store.append_event(
                context,
                OperationalEvent(
                    kind="created",
                    occurred_at=datetime.now(timezone.utc),
                    message="retry_requested" if retry_reason is not None else "scheduled",
                    payload=(
                        {"reason_sha256": hashlib.sha256(retry_reason.encode()).hexdigest()}
                        if retry_reason is not None
                        else {}
                    ),
                ),
            )
            running = old.model_copy(
                update={"state": "running", "latest_attempt_id": attempt_id, "outcome": None}
            )
            self.store.write_summary(running)

        def emit(event: ExecutorEvent) -> None:
            try:
                raw_event = (
                    event.model_dump(mode="python") if isinstance(event, BaseModel) else event
                )
                validated = _EXECUTOR_EVENT_ADAPTER.validate_python(raw_event)
            except Exception:
                if self.diagnostic is not None:
                    self.diagnostic(event)
                raise ValueError("executor emitted a malformed event")
            if isinstance(validated, ExecutorProgressEvent):
                operational = OperationalEvent(
                    kind="progress",
                    occurred_at=datetime.now(timezone.utc),
                    message=validated.code,
                    payload={},
                )
            elif isinstance(validated, ExecutorArtifactEvent):
                operational = OperationalEvent(
                    kind="artifact",
                    occurred_at=datetime.now(timezone.utc),
                    message=validated.code,
                    payload={
                        "artifact_id": validated.artifact_id,
                        "artifact_sha256": validated.artifact_sha256,
                    },
                )
            else:
                if self.diagnostic is not None:
                    self.diagnostic(event)
                raise ValueError("executor emitted a non-allowlisted operational event")
            with self._publication_lock:
                if self._cancel_requested.is_set():
                    raise ExecutionInterrupted
                with self.store.lease_mutation(capability):
                    self.store.append_event(context, operational)

        try:
            if self._cancel_requested.is_set():
                raise ExecutionInterrupted
            outcome = self.executor.run(
                case, condition, context, emit, self._cancel_requested, self._publication_lock
            )
        except ExecutionInterrupted:
            outcome = TerminalOutcome(status="interrupted", reason="interrupted")
        except Exception:
            outcome = TerminalOutcome(status="failed", reason="executor_exception")
        outcome = self._safe_outcome(outcome)
        with self._publication_lock:
            with self.store.lease_mutation(capability):
                durable_outcome = self.store.read_outcome(context)
                if durable_outcome is None:
                    self.store.write_outcome(context, outcome)
                    self.store.append_event(
                        context,
                        OperationalEvent(
                            kind="finished",
                            occurred_at=datetime.now(timezone.utc),
                            message=cast("OperationalCode", outcome.reason),
                            payload={"status": outcome.status},
                            policy_telemetry=outcome.policy_telemetry,
                        ),
                    )
                else:
                    outcome = durable_outcome
                final = running.model_copy(update={"state": outcome.status, "outcome": outcome})
                self.store.write_summary(final)
        return final

    def _record_failure(
        self, identifier: str, capability: CoordinatorLeaseCapability
    ) -> JobSummary:
        with self.store.lease_mutation(capability):
            current = self._summaries[identifier]
            outcome = TerminalOutcome(status="failed", reason="scheduler_worker_failure")
            result = current.model_copy(update={"state": "failed", "outcome": outcome})
            self.store.write_summary(result)
        return result

    def _record_interruption(
        self, identifier: str, capability: CoordinatorLeaseCapability
    ) -> JobSummary:
        with self.store.lease_mutation(capability):
            current = self._summaries[identifier]
            outcome = TerminalOutcome(status="interrupted", reason="coordinator_cancelled")
            result = current.model_copy(update={"state": "interrupted", "outcome": outcome})
            self.store.write_summary(result)
        return result

    def _load_definition(self) -> tuple[ExperimentManifest, ExperimentPlan]:
        definition = self.store.load_definition()
        return definition.manifest, definition.plan

    def _reload_reconcile_locked(self) -> None:
        """Reload immutable bytes and reconcile only after owning the lock."""
        definition, summaries, _ = reconcile_idle_locked(self.store)
        self.manifest, self.plan = definition.manifest, definition.plan
        self._summaries = summaries

    def _safe_outcome(self, outcome: TerminalOutcome) -> TerminalOutcome:
        if outcome.status == "failed" and outcome.reason in {
            "container_cleanup_failed",
            "container_runtime_failed",
            "post_image_policy_violation",
            "pre_image_policy_violation",
        }:
            return outcome
        reasons = {
            "succeeded": "completed",
            "failed": "executor_failed",
            "interrupted": "executor_interrupted",
            "cancelled": "executor_cancelled",
        }
        return outcome.model_copy(update={"reason": reasons[outcome.status]})

    def _manifest_digest(self) -> str:
        return self.store.load_definition().manifest_digest

    def _case_condition(self, identifier: str) -> tuple[ExpandedCase, NamedCondition]:
        summary = self._summaries[identifier]
        case = next(item for item in self.plan.cases if item.case_id == summary.identity.case_id)
        condition = next(
            item for item in self.plan.conditions if item.name == summary.identity.condition_name
        )
        return case, condition

    def _job_ids_with_state(self, states: set[str]) -> tuple[str, ...]:
        return tuple(
            identifier for identifier, summary in self._summaries.items() if summary.state in states
        )

    def _planned_identities(self) -> tuple[tuple[str, JobIdentity], ...]:
        return self._planned_identities_for(self.manifest, self.plan)

    @staticmethod
    def _planned_identities_for(
        manifest: ExperimentManifest, plan: ExperimentPlan
    ) -> tuple[tuple[str, JobIdentity], ...]:
        result: list[tuple[str, JobIdentity]] = []
        for plan_job in plan.jobs:
            case = next(item for item in plan.cases if item.case_id == plan_job.case_id)
            condition = next(
                item for item in plan.conditions if item.name == plan_job.condition_name
            )
            identifier = job_id(plan, case, condition)
            result.append(
                (
                    identifier,
                    JobIdentity(
                        experiment_id=manifest.experiment_id,
                        case_id=case.case_id,
                        condition_name=condition.name,
                        job_id=identifier,
                    ),
                )
            )
        return tuple(result)

    def _next_attempt_number(self, identifier: str) -> int:
        numbers = self.store.used_attempt_numbers(identifier)
        return max(numbers, default=0) + 1


def internal_operation(
    runtime: SchedulerRuntime,
    operation: str,
    *,
    job_id_value: str | None = None,
    reason: str | None = None,
) -> tuple[JobSummary, ...]:
    """Run a generic scheduler operation under one uninterrupted lease."""
    from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import PiSchedulerExecutor

    if isinstance(runtime.executor, PiSchedulerExecutor):
        raise RuntimeError("Pi execution must use execute_pi_operation")
    with runtime.store.coordinator_lease() as capability:
        manifest, plan = runtime._load_definition()
        validate_manifest(manifest, plan)
        runtime.manifest, runtime.plan = manifest, plan
        runtime.preflight()
        runtime._reload_reconcile_locked()
        if operation == "retry":
            if job_id_value is None or reason is None or not reason.strip():
                raise ValueError("retry requires an explicit reason")
            summary = runtime._summaries.get(job_id_value)
            if summary is None:
                raise KeyError(job_id_value)
            if summary.outcome is None or summary.outcome.status not in {
                "failed",
                "interrupted",
                "cancelled",
            }:
                raise ValueError(
                    "only the latest failed, interrupted, or cancelled outcome may be retried"
                )
        if operation == "run":
            selected = runtime._job_ids_with_state({"pending"})
            return runtime._execute(selected, allowed_states={"pending"}, capability=capability)
        if operation == "resume":
            selected = runtime._job_ids_with_state({"pending", "interrupted"})
            return runtime._execute(
                selected, allowed_states={"pending", "interrupted"}, capability=capability
            )
        if operation == "retry":
            assert job_id_value is not None and reason is not None
            runtime._execute(
                (job_id_value,),
                retry_reason=reason,
                allowed_states=set(),
                capability=capability,
            )
            return (runtime._summaries[job_id_value],)
        raise ValueError(f"unknown internal scheduler operation: {operation}")
