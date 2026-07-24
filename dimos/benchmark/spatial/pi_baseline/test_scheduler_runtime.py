from collections.abc import Callable
from threading import Event, Lock, Thread
import time

import pytest

from .scheduler_executor import EventSink, ExecutionInterrupted, Executor
from .scheduler_models import (
    AttemptContext,
    ExecutorArtifactEvent,
    ExecutorProgressEvent,
    ExpandedCase,
    ExperimentManifest,
    ExperimentPlan,
    NamedCondition,
    TerminalOutcome,
)
from .scheduler_plan import expand_plan, selected_inputs_digest
from .scheduler_runtime import PreflightError, SchedulerRuntime
from .scheduler_store import FilesystemExperimentStore


def test_post_image_policy_outcome_survives_scheduler_normalization() -> None:
    outcome = SchedulerRuntime._safe_outcome(
        object.__new__(SchedulerRuntime),
        TerminalOutcome(status="failed", reason="post_image_policy_violation"),
    )
    assert outcome == TerminalOutcome(status="failed", reason="post_image_policy_violation")


def test_post_image_policy_outcome_persists_finished_event_and_projection(tmp_path) -> None:
    runtime = make_runtime(
        tmp_path,
        FakeExecutor(TerminalOutcome(status="failed", reason="post_image_policy_violation")),
        workers=1,
        case_count=1,
    )
    result = runtime.run()[0]
    assert result.outcome == TerminalOutcome(status="failed", reason="post_image_policy_violation")
    assert result.latest_attempt_id is not None
    attempt = AttemptContext(
        identity=result.identity,
        attempt_id=result.latest_attempt_id,
        attempt_number=1,
        directory_name=result.latest_attempt_id,
    )
    finished = [event for event in runtime.store.events(attempt) if event.kind == "finished"]
    assert finished[0].message == "post_image_policy_violation"


def test_container_runtime_failure_persists_terminal_outcome_and_finished_event(tmp_path) -> None:
    runtime = make_runtime(
        tmp_path,
        FakeExecutor(TerminalOutcome(status="failed", reason="container_runtime_failed")),
        workers=1,
        case_count=1,
    )
    result = runtime.run()[0]

    assert result.state == "failed"
    assert result.outcome == TerminalOutcome(status="failed", reason="container_runtime_failed")
    assert result.latest_attempt_id is not None
    context = AttemptContext(
        identity=result.identity,
        attempt_id=result.latest_attempt_id,
        attempt_number=1,
        directory_name=result.latest_attempt_id,
    )
    assert runtime.store.read_outcome(context) == result.outcome
    finished = [event for event in runtime.store.events(context) if event.kind == "finished"]
    assert [(event.message, event.payload) for event in finished] == [
        ("container_runtime_failed", {"status": "failed"})
    ]
    restarted = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    assert restarted.recover()[0].outcome == result.outcome


def make_plan(workers: int = 2, case_count: int = 4) -> ExperimentPlan:
    return expand_plan(
        "experiment",
        [
            ExpandedCase(case_id=f"case-{index}", payload={"index": index})
            for index in range(case_count)
        ],
        [NamedCondition(name="baseline")],
        workers=workers,
    )


def make_runtime(
    tmp_path,
    executor: Executor,
    workers: int = 2,
    prerequisite: Callable[[], bool] | None = None,
    case_count: int = 4,
) -> SchedulerRuntime:
    plan = make_plan(workers, case_count)
    fingerprints = {
        name: "a" * 64
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
    manifest = ExperimentManifest(
        experiment_id="experiment",
        plan_digest=plan.plan_digest,
        executor_kind="generic",
        executor_snapshot_digest="a" * 64,
        selected_inputs_digest=selected_inputs_digest(plan),
        workers=workers,
        **fingerprints,
    )
    store = FilesystemExperimentStore(tmp_path / "experiment")
    store.create(manifest, plan)
    return SchedulerRuntime(store, executor, prerequisite=prerequisite)


class FakeExecutor:
    def __init__(self, outcome: TerminalOutcome | None = None, delay: float = 0.0) -> None:
        self.outcome = outcome or TerminalOutcome(status="succeeded", reason="done")
        self.delay = delay
        self.active = 0
        self.maximum_active = 0
        self.calls = 0
        self.lock = Lock()
        self.started = Event()

    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> TerminalOutcome:
        with self.lock:
            self.calls += 1
            self.active += 1
            self.maximum_active = max(self.maximum_active, self.active)
            self.started.set()
        emit(ExecutorProgressEvent(kind="progress", code="executor_progress"))
        time.sleep(self.delay)
        with self.lock:
            self.active -= 1
        if cancel_requested.is_set():
            raise ExecutionInterrupted
        return self.outcome


def test_runtime_caps_concurrency_and_completes_all_jobs(tmp_path) -> None:
    executor = FakeExecutor(delay=0.02)
    runtime = make_runtime(tmp_path, executor, workers=2)
    results = runtime.run()
    assert executor.maximum_active == 2
    assert len(results) == 4
    assert all(result.state == "succeeded" for result in results)


def test_public_projections_ignore_forged_runtime_cache(tmp_path) -> None:
    runtime = make_runtime(
        tmp_path,
        FakeExecutor(TerminalOutcome(status="failed", reason="executor_failed")),
        workers=1,
        case_count=1,
    )
    completed = runtime.run()[0]
    runtime._summaries[completed.identity.job_id] = completed.model_copy(
        update={"state": "pending", "outcome": None}
    )
    runtime.manifest = runtime.manifest.model_copy(update={"workers": 99})

    status = runtime.operational_status()
    summaries = runtime.summaries()
    retryable = runtime.retryable()
    triage = runtime.triage()

    assert status["workers"] == 1
    assert status["states"]["failed"] == 1
    assert summaries[0].state == "failed"
    assert retryable[0].state == "failed"
    assert triage[0].reason == "executor_failed"


def test_resume_does_not_rerun_terminal_and_retry_creates_attempt(tmp_path) -> None:
    failed = FakeExecutor(TerminalOutcome(status="failed", reason="transient"))
    runtime = make_runtime(tmp_path, failed, workers=1)
    first = runtime.run()
    failed_job = first[0].identity.job_id
    assert first[0].state == "failed"

    successful_executor = FakeExecutor()
    successful = SchedulerRuntime(runtime.store, successful_executor, prerequisite=lambda: True)
    successful.resume()
    assert successful_executor.calls == 0
    retried = successful.retry(failed_job, "operator requested retry")
    assert retried.state == "succeeded"
    attempt_root = runtime.store.root / "attempts" / failed_job
    assert sorted(path.name for path in attempt_root.iterdir()) == ["attempt-1", "attempt-2"]


def test_resume_runs_interrupted_jobs_but_not_failed_terminal_jobs(tmp_path) -> None:
    executor = FakeExecutor(TerminalOutcome(status="failed", reason="transient"), delay=0.03)
    runtime = make_runtime(tmp_path, executor, workers=1)
    thread = Thread(target=runtime.run)
    thread.start()
    assert executor.started.wait(1)
    runtime.cancel()
    thread.join(2)

    interrupted = [summary for summary in runtime.summaries() if summary.state == "interrupted"]
    failed = [summary for summary in runtime.summaries() if summary.state == "failed"]
    assert interrupted
    assert not failed
    pending = [summary for summary in runtime.summaries() if summary.state == "pending"]
    assert pending

    successful_executor = FakeExecutor()
    resumed = SchedulerRuntime(runtime.store, successful_executor, prerequisite=lambda: True)
    resumed.resume()

    assert successful_executor.calls == len(interrupted) + len(pending)
    assert all(
        summary.state == "succeeded"
        for summary in resumed.summaries()
        if summary.identity.job_id in {item.identity.job_id for item in interrupted + pending}
    )
    assert all(
        summary.state == "failed"
        for summary in resumed.summaries()
        if summary.identity.job_id in {item.identity.job_id for item in failed}
    )
    for summary in interrupted:
        attempt_root = runtime.store.root / "attempts" / summary.identity.job_id
        assert sorted(path.name for path in attempt_root.iterdir()) == ["attempt-1", "attempt-2"]
    for summary in pending:
        attempt_root = runtime.store.root / "attempts" / summary.identity.job_id
        assert sorted(path.name for path in attempt_root.iterdir()) == ["attempt-1"]


def test_cancellation_stops_admission_and_interrupts_cooperative_worker(tmp_path) -> None:
    executor = FakeExecutor(delay=0.05)
    runtime = make_runtime(tmp_path, executor, workers=1)
    thread = Thread(target=runtime.run)
    thread.start()
    assert executor.started.wait(1)
    runtime.cancel()
    thread.join(2)
    assert not thread.is_alive()
    assert any(summary.state == "interrupted" for summary in runtime.summaries())
    assert any(summary.state == "pending" for summary in runtime.summaries())


def test_cancel_before_admission_is_pending_and_sticky(tmp_path) -> None:
    executor = FakeExecutor()
    runtime = make_runtime(tmp_path, executor, workers=1, case_count=1)
    runtime.cancel()
    assert runtime.run()[0].state == "pending"
    assert runtime.run()[0].state == "pending"
    assert executor.calls == 0
    assert not tuple((runtime.store.root / "attempts").rglob("context.json"))


def test_cancel_forwards_event_and_worker_can_interrupt(tmp_path) -> None:
    observed: list[Event] = []

    class InterruptingExecutor(FakeExecutor):
        def run(self, case, condition, context, emit, cancel_requested, publication_lock):
            observed.append(cancel_requested)
            raise ExecutionInterrupted

    runtime = make_runtime(tmp_path, InterruptingExecutor(), workers=1, case_count=1)
    result = runtime.run()[0]
    assert result.state == "interrupted"
    assert observed == [runtime._cancel_requested]
    assert result.outcome is not None
    assert result.outcome.reason == "executor_interrupted"


def test_completion_wins_when_cancel_arrives_after_executor_completion(tmp_path) -> None:
    completed = Event()
    release = Event()

    class CompletionExecutor(FakeExecutor):
        def run(self, case, condition, context, emit, cancel_requested, publication_lock):
            completed.set()
            release.wait(1)
            return TerminalOutcome(status="succeeded", reason="private")

    executor = CompletionExecutor()
    runtime = make_runtime(tmp_path, executor, workers=1, case_count=1)
    thread = Thread(target=runtime.run)
    thread.start()
    assert completed.wait(1)
    runtime.cancel()
    release.set()
    thread.join(2)
    summary = runtime.summaries()[0]
    assert summary.state == "succeeded"
    assert summary.outcome is not None
    assert summary.outcome.reason == "completed"


def test_cleanup_reason_is_preserved_only_for_failed_outcomes(tmp_path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    cleanup = "container_cleanup_failed"
    assert runtime._safe_outcome(TerminalOutcome(status="failed", reason=cleanup)).reason == cleanup
    assert (
        runtime._safe_outcome(TerminalOutcome(status="succeeded", reason=cleanup)).reason
        == "completed"
    )
    assert (
        runtime._safe_outcome(TerminalOutcome(status="interrupted", reason=cleanup)).reason
        == "executor_interrupted"
    )


def test_artifact_cancellation_wins_before_scheduler_publication(tmp_path) -> None:
    ready = Event()
    release = Event()

    class ArtifactExecutor(FakeExecutor):
        def run(self, case, condition, context, emit, cancel_requested, publication_lock):
            ready.set()
            release.wait(1)
            emit(
                ExecutorArtifactEvent(
                    kind="artifact",
                    code="artifact_recorded",
                    artifact_id="artifact-1",
                    artifact_sha256="a" * 64,
                )
            )
            return self.outcome

    runtime = make_runtime(tmp_path, ArtifactExecutor(), workers=1, case_count=1)
    thread = Thread(target=runtime.run)
    thread.start()
    assert ready.wait(1)
    runtime.cancel()
    release.set()
    thread.join(2)
    assert runtime.summaries()[0].state == "interrupted"
    with runtime.store.coordinator_lock():
        attempts = runtime.store.recover_attempts(
            "experiment", runtime.summaries()[0].identity.job_id
        )
        assert not any(
            event.kind == "artifact" for event in runtime.store.events(attempts[0].context)
        )


def test_artifact_publication_wins_when_cancel_waits_for_publication_lock(tmp_path) -> None:
    entered = Event()
    release = Event()
    original_append = FilesystemExperimentStore.append_event

    def append_event(store, context, event):
        if event.kind == "artifact":
            entered.set()
            release.wait(1)
        original_append(store, context, event)

    class ArtifactExecutor(FakeExecutor):
        def run(self, case, condition, context, emit, cancel_requested, publication_lock):
            emit(
                ExecutorArtifactEvent(
                    kind="artifact",
                    code="artifact_recorded",
                    artifact_id="artifact-1",
                    artifact_sha256="a" * 64,
                )
            )
            return self.outcome

    runtime = make_runtime(tmp_path, ArtifactExecutor(), workers=1, case_count=1)
    runtime.store.append_event = append_event.__get__(runtime.store, FilesystemExperimentStore)
    thread = Thread(target=runtime.run)
    thread.start()
    assert entered.wait(1)
    canceller = Thread(target=runtime.cancel)
    canceller.start()
    assert canceller.is_alive()
    release.set()
    canceller.join(1)
    thread.join(2)
    assert runtime.summaries()[0].state == "succeeded"
    with runtime.store.coordinator_lock():
        attempt = runtime.store.recover_attempts(
            "experiment", runtime.summaries()[0].identity.job_id
        )[0]
        artifacts = [
            event for event in runtime.store.events(attempt.context) if event.kind == "artifact"
        ]
    assert len(artifacts) == 1
    assert artifacts[0].payload["artifact_id"] == "artifact-1"


def test_preflight_denial_writes_no_attempts(tmp_path) -> None:
    executor = FakeExecutor()
    runtime = make_runtime(tmp_path, executor, prerequisite=lambda: False)
    with pytest.raises(PreflightError):
        runtime.run()
    assert executor.calls == 0
    assert not tuple((runtime.store.root / "attempts").rglob("context.json"))


def test_restart_ignores_running_summary_without_attempt_artifacts(tmp_path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    running = summary.model_copy(update={"state": "running"})
    with runtime.store.coordinator_lock():
        runtime.store.write_summary(running)
    restarted = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    restarted.recover()
    reconciled = next(
        item for item in restarted.summaries() if item.identity.job_id == summary.identity.job_id
    )
    assert reconciled.state == "pending"


def test_restart_rebuilds_terminal_summary_from_attempt_artifacts(tmp_path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    summary_path = runtime.store.root / "jobs" / f"{completed.identity.job_id}.json"
    summary_path.unlink()
    restarted = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    restarted.recover()
    rebuilt = next(
        item for item in restarted.summaries() if item.identity.job_id == completed.identity.job_id
    )
    assert rebuilt.state == "succeeded"
    assert rebuilt.latest_attempt_id == "attempt-1"
