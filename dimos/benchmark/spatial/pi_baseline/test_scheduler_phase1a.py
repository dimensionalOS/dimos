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
import json
from multiprocessing import Process
import os
from pathlib import Path
import shutil
from threading import Event, Lock, Thread

from pydantic import ValidationError
import pytest

from .scheduler_executor import EventSink
from .scheduler_models import (
    AttemptContext,
    ExecutorProgressEvent,
    ExpandedCase,
    JobIdentity,
    JobSummary,
    LifecycleEvent,
    NamedCondition,
    OperationalEvent,
    QuarantineMetadata,
    TerminalOutcome,
)
from .scheduler_plan import job_id, selected_inputs_digest
from .scheduler_runtime import SchedulerRuntime
from .scheduler_store import (
    CoordinatorLeaseCapability,
    CoordinatorLockError,
    FilesystemExperimentStore,
    StoreMutationError,
)
from .test_scheduler_runtime import FakeExecutor, make_runtime


def _hold_lock(root: str, started, release) -> None:
    store = FilesystemExperimentStore(Path(root))
    with store.coordinator_lock():
        started.set()
        release.wait(2)


def _die_with_unfinished_attempt(root: str) -> None:
    store = FilesystemExperimentStore(Path(root))
    with store.coordinator_lease():
        definition = store.load_definition()
        case = definition.plan.cases[0]
        condition = definition.plan.conditions[0]
        identity = definition.plan.jobs[0]

        job_identity = JobIdentity(
            experiment_id=definition.manifest.experiment_id,
            case_id=identity.case_id,
            condition_name=identity.condition_name,
            job_id=job_id(definition.plan, case, condition),
        )
        context = AttemptContext(
            identity=job_identity,
            attempt_id="attempt-1",
            attempt_number=1,
            directory_name="attempt-1",
            manifest_digest=definition.manifest_digest,
        )
        store.create_attempt(context, case, condition)
        store.write_summary(
            JobSummary(identity=job_identity, state="running", latest_attempt_id="attempt-1")
        )
        os._exit(0)


class BlockingExecutor(FakeExecutor):
    def __init__(self) -> None:
        super().__init__()
        self.release = Event()

    def run(
        self, case, condition, context, emit, cancel_requested, publication_lock
    ) -> TerminalOutcome:
        with self.lock:
            self.calls += 1
            self.active += 1
            self.maximum_active = max(self.maximum_active, self.active)
            self.started.set()
        emit(ExecutorProgressEvent(kind="progress", code="executor_progress"))
        self.release.wait(2)
        with self.lock:
            self.active -= 1
        return self.outcome


def test_coordinator_lock_rejects_second_process(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    from multiprocessing import Event

    started = Event()
    release = Event()
    process = Process(target=_hold_lock, args=(str(runtime.store.root), started, release))
    process.start()
    assert started.wait(2)
    try:
        with pytest.raises(CoordinatorLockError, match="coordinator lock is held"):
            with runtime.store.coordinator_lock():
                pass
    finally:
        release.set()
        process.join(2)
    assert process.exitcode == 0


def test_process_death_releases_lease_and_recovery_interrupts_attempt(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    process = Process(target=_die_with_unfinished_attempt, args=(str(runtime.store.root),))
    process.start()
    process.join(2)
    assert process.exitcode == 0

    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    summary = recovered.summaries()[0]
    assert summary.state == "interrupted"
    assert summary.outcome is not None
    assert summary.outcome.reason == "missing_terminal_outcome"


def test_failed_same_store_lock_acquisition_does_not_clear_owner(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    other = FilesystemExperimentStore(runtime.store.root)
    with runtime.store.coordinator_lock():
        with pytest.raises(CoordinatorLockError):
            with other.coordinator_lock():
                pass
        runtime.store.write_summary(runtime.summaries()[0])


def test_same_store_cross_thread_lock_failure_does_not_clear_owner(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    entered = Event()
    release = Event()

    def owner() -> None:
        with runtime.store.coordinator_lock():
            entered.set()
            release.wait(2)

    thread = Thread(target=owner)
    thread.start()
    assert entered.wait(2)
    try:
        with pytest.raises(CoordinatorLockError):
            with runtime.store.coordinator_lock():
                pass
    finally:
        release.set()
        thread.join(2)
    assert not thread.is_alive()


def test_same_store_cross_thread_mutator_is_denied(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    errors: list[BaseException] = []
    with runtime.store.coordinator_lock():
        thread = Thread(
            target=lambda: _capture_error(
                errors, lambda: runtime.store.write_summary(runtime.summaries()[0])
            )
        )
        thread.start()
        thread.join(2)
    assert not thread.is_alive()
    assert errors and isinstance(errors[0], StoreMutationError)


def test_worker_mutation_requires_unforgeable_lease_capability(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    other = make_runtime(tmp_path / "other", FakeExecutor(), workers=1)
    errors: list[BaseException] = []
    with runtime.store.coordinator_lease() as capability:
        with pytest.raises(TypeError, match="not constructible"):
            CoordinatorLeaseCapability(runtime.store, object())
        with pytest.raises(TypeError):
            runtime.store.lease_mutation()  # type: ignore[call-arg]

        def unrelated() -> None:
            try:
                with runtime.store.lease_mutation(None):  # type: ignore[arg-type]
                    pass
            except BaseException as error:
                errors.append(error)

        thread = Thread(target=unrelated)
        thread.start()
        thread.join(2)
        assert isinstance(errors[0], StoreMutationError)
        with other.store.coordinator_lease() as other_capability:
            with pytest.raises(StoreMutationError):
                with runtime.store.lease_mutation(other_capability):
                    pass
        with runtime.store.lease_mutation(capability):
            pass
    with pytest.raises(StoreMutationError):
        with runtime.store.lease_mutation(capability):
            pass


def _capture_error(errors: list[BaseException], operation) -> None:
    try:
        operation()
    except BaseException as error:
        errors.append(error)


def test_stale_preconstructed_runtime_reloads_before_execution(tmp_path: Path) -> None:
    first = make_runtime(tmp_path, FakeExecutor(), workers=1)
    second = SchedulerRuntime(first.store, FakeExecutor(), prerequisite=lambda: True)
    assert second.run()[0].state == "succeeded"
    assert first.run() == ()


def test_concurrent_preconstructed_runtimes_claim_a_job_once(tmp_path: Path) -> None:
    first_executor = BlockingExecutor()
    second_executor = FakeExecutor()
    first = make_runtime(tmp_path, first_executor, workers=1, case_count=1)
    second = SchedulerRuntime(first.store, second_executor, prerequisite=lambda: True)
    results: list[tuple[object, ...]] = []
    errors: list[BaseException] = []

    def run(runtime: SchedulerRuntime) -> None:
        try:
            results.append(runtime.run())
        except BaseException as error:
            errors.append(error)

    first_thread = Thread(target=run, args=(first,))
    first_thread.start()
    assert first_executor.started.wait(1)
    second_thread = Thread(target=run, args=(second,))
    second_thread.start()
    second_thread.join(2)
    first_executor.release.set()
    first_thread.join(2)
    assert not first_thread.is_alive() and not second_thread.is_alive()
    assert first_executor.calls + second_executor.calls == 1
    assert len(results) == 1
    assert len(errors) == 1 and isinstance(errors[0], CoordinatorLockError)


def test_one_job_barrier_admits_exactly_one_runtime(tmp_path: Path) -> None:
    first_executor = BlockingExecutor()
    second_executor = FakeExecutor()
    first = make_runtime(tmp_path, first_executor, workers=1, case_count=1)
    second = SchedulerRuntime(first.store, second_executor, prerequisite=lambda: True)
    results: list[tuple[object, ...]] = []
    errors: list[BaseException] = []

    def run(runtime: SchedulerRuntime) -> None:
        try:
            results.append(runtime.run())
        except BaseException as error:
            errors.append(error)

    winner = Thread(target=run, args=(first,))
    loser = Thread(target=run, args=(second,))
    winner.start()
    assert first_executor.started.wait(1)
    loser.start()
    loser.join(2)
    first_executor.release.set()
    winner.join(2)
    threads = [winner, loser]
    assert all(not thread.is_alive() for thread in threads)
    assert first_executor.calls + second_executor.calls == 1
    assert len(results) == 1 and len(errors) == 1
    assert isinstance(errors[0], CoordinatorLockError)


def test_coordinator_lease_remains_held_while_executor_is_active(tmp_path: Path) -> None:
    executor = BlockingExecutor()
    runtime = make_runtime(tmp_path, executor, workers=1)
    other = FilesystemExperimentStore(runtime.store.root)
    thread = Thread(target=runtime.run)
    thread.start()
    assert executor.started.wait(1)
    try:
        with pytest.raises(CoordinatorLockError):
            with other.coordinator_lock():
                pass
    finally:
        executor.release.set()
        thread.join(2)
    assert not thread.is_alive()


def test_quarantine_metadata_requires_q_uuid_reservation_name() -> None:
    with pytest.raises(ValidationError):
        QuarantineMetadata(
            quarantined_name="quarantine-1",
            original_name="attempt-1",
            original_attempt_number=1,
        )
    metadata = QuarantineMetadata(
        quarantined_name="q-0123456789abcdef0123456789abcdef",
        original_name="attempt-1",
        original_attempt_number=1,
    )
    assert metadata.quarantined_name.startswith("q-")


def test_invalid_explicit_experiment_root_fails_closed_without_quarantine(tmp_path: Path) -> None:
    outside = tmp_path / "outside"
    outside.mkdir()
    root = tmp_path / "experiment"
    root.symlink_to(outside, target_is_directory=True)
    store = FilesystemExperimentStore(root)
    with pytest.raises(ValueError):
        store.load_definition()
    assert not (tmp_path / "quarantine").exists()


def test_invalid_attempts_root_is_quarantined_not_treated_as_missing(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    attempts = runtime.store.root / "attempts"
    attempts.rmdir()
    attempts.write_text("invalid", encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert tuple((runtime.store.root / "quarantine" / "attempts").iterdir())
    assert recovered.run()[0].state == "succeeded"


def test_missing_attempts_root_is_recreated_under_lock(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    (runtime.store.root / "attempts").rmdir()
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert (runtime.store.root / "attempts").is_dir()
    assert recovered.run()[0].state == "succeeded"


def test_invalid_job_entry_is_quarantined(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    invalid = runtime.store.root / "attempts" / "not-a-job"
    invalid.write_text("invalid", encoding="utf-8")
    SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True).recover()
    assert tuple((runtime.store.root / "quarantine" / "unknown-job").iterdir())


def test_quarantined_attempt_number_is_not_reused_on_retry_allocation(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    attempt = next((runtime.store.root / "attempts" / completed.identity.job_id).glob("attempt-1"))
    (attempt / "context.json").write_text("{}", encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    result = recovered.run()[0]
    assert result.latest_attempt_id == "attempt-2"


def test_quarantined_transactional_attempt_number_is_not_reused(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    job_root = runtime.store.root / "attempts" / summary.identity.job_id
    job_root.mkdir()
    (job_root / ".attempt-7.deadbeef.tmp").mkdir()
    result = runtime.run()[0]
    assert result.latest_attempt_id == "attempt-8"


def test_active_allocation_symlink_is_quarantined(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    external = tmp_path / "external-active"
    external.mkdir()
    (external / "attempt-4").mkdir()
    link = runtime.store.root / "attempts" / summary.identity.job_id
    link.symlink_to(external, target_is_directory=True)
    with runtime.store.coordinator_lock():
        assert runtime.store.used_attempt_numbers(summary.identity.job_id) == ()
    assert not link.is_symlink()


def test_quarantine_allocation_symlink_is_quarantined(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    quarantine = runtime.store.root / "quarantine"
    quarantine.mkdir()
    external = tmp_path / "external-quarantine"
    external.mkdir()
    link = quarantine / summary.identity.job_id
    link.symlink_to(external, target_is_directory=True)
    with runtime.store.coordinator_lock():
        assert runtime.store.used_attempt_numbers(summary.identity.job_id) == ()
    assert not link.is_symlink()


def test_load_definition_rejects_manifest_and_plan_byte_drift(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    experiment = runtime.store.root
    plan_path = experiment / "plan.json"
    plan_path.write_bytes(plan_path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="plan bytes are not canonical"):
        runtime.store.load_definition()

    plan_path.write_bytes(plan_path.read_bytes().rstrip() + b"\n")
    manifest_path = experiment / "manifest.json"
    manifest_path.write_bytes(manifest_path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="manifest bytes are not canonical"):
        runtime.store.load_definition()


def test_plan_and_selected_input_digests_are_distinct(tmp_path: Path) -> None:
    first = make_runtime(tmp_path / "one", FakeExecutor(), workers=1)
    second = make_runtime(tmp_path / "two", FakeExecutor(), workers=2)
    assert first.plan.plan_digest != second.plan.plan_digest
    assert selected_inputs_digest(first.plan) == selected_inputs_digest(second.plan)


def test_store_is_experiment_scoped_with_sibling_directories(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path / "one", FakeExecutor(), workers=1)
    shutil.copytree(runtime.store.root, tmp_path / "sibling")
    with pytest.raises(ValueError, match="store root"):
        FilesystemExperimentStore(tmp_path).load_definition()


def test_mutators_require_coordinator_lock(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    with pytest.raises(StoreMutationError, match="coordinator_lock"):
        runtime.store.write_summary(summary)


class MaliciousEventExecutor:
    def run(
        self, case, condition, context, emit, cancel_requested, publication_lock
    ) -> TerminalOutcome:
        malicious = LifecycleEvent.model_construct(
            kind="progress",
            occurred_at=datetime.now(timezone.utc),
            message="secret message",
            payload={"secret": {"answer": "private"}},
        )
        emit(malicious)
        return TerminalOutcome(status="succeeded", reason="completed")


def test_malicious_executor_event_is_private_and_not_persisted(tmp_path: Path) -> None:
    diagnostics: list[LifecycleEvent] = []
    runtime = make_runtime(tmp_path, MaliciousEventExecutor(), workers=1)
    runtime.diagnostic = diagnostics.append
    result = runtime.run()[0]
    assert result.state == "failed"
    assert diagnostics and diagnostics[0].payload["secret"]
    attempt = next((runtime.store.root / "attempts" / result.identity.job_id).glob("attempt-*"))
    assert "secret" not in (attempt / "events.jsonl").read_text()


def test_events_append_rejects_symlink_replacement_without_following(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    case = runtime.plan.cases[0]
    condition = runtime.plan.conditions[0]
    context = AttemptContext(
        identity=summary.identity,
        attempt_id="attempt-1",
        attempt_number=1,
        directory_name="attempt-1",
    )
    with runtime.store.coordinator_lock():
        runtime.store.create_attempt(context, case, condition)
        events = (
            runtime.store.root / "attempts" / summary.identity.job_id / "attempt-1" / "events.jsonl"
        )
        outside = tmp_path / "outside-events.jsonl"
        outside.write_text("private\n", encoding="utf-8")
        events.unlink()
        events.symlink_to(outside)
        with pytest.raises(OSError):
            runtime.store.append_event(
                context,
                OperationalEvent(
                    kind="progress",
                    occurred_at=datetime.now(timezone.utc),
                    message="executor_progress",
                ),
            )
    assert outside.read_text(encoding="utf-8") == "private\n"


def test_malformed_constructed_typed_event_is_private_and_not_persisted(tmp_path: Path) -> None:
    class MalformedExecutor:
        def run(
            self, case, condition, context, emit, cancel_requested, publication_lock
        ) -> TerminalOutcome:
            emit(ExecutorProgressEvent.model_construct(kind="progress", code="secret-code"))
            return TerminalOutcome(status="succeeded", reason="completed")

    diagnostics: list[object] = []
    runtime = make_runtime(tmp_path, MalformedExecutor(), workers=1)
    runtime.diagnostic = diagnostics.append
    result = runtime.run()[0]
    assert result.state == "failed"
    assert diagnostics
    attempt = next((runtime.store.root / "attempts" / result.identity.job_id).glob("attempt-*"))
    assert "secret-code" not in (attempt / "events.jsonl").read_text()


def test_quarantine_symlink_does_not_change_target_permissions(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    external = tmp_path / "external-attempt"
    external.mkdir(mode=0o755)
    before = external.stat().st_mode & 0o777
    link = runtime.store.root / "attempts" / summary.identity.job_id / "attempt-99"
    link.parent.mkdir(parents=True)
    link.symlink_to(external, target_is_directory=True)
    SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True).recover()
    assert external.stat().st_mode & 0o777 == before


def test_embedded_snapshot_manifest_mutation_is_quarantined(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    attempt = runtime.store.root / "attempts" / completed.identity.job_id / "attempt-1"
    path = attempt / "attempt-manifest.v1.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["manifest"]["experiment_id"] = "forged"
    path.write_text(json.dumps(document) + "\n", encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert tuple((runtime.store.root / "quarantine" / completed.identity.job_id).iterdir())


def test_record_symlink_is_quarantined_and_excluded_from_recovery(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1, case_count=1)
    completed = runtime.run()[0]
    attempt = runtime.store.root / "attempts" / completed.identity.job_id / "attempt-1"
    outside = tmp_path / "outside-record"
    outside.write_text("private\n", encoding="utf-8")
    (attempt / "case.json").unlink()
    (attempt / "case.json").symlink_to(outside)
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert recovered.run()[0].latest_attempt_id == "attempt-2"
    assert outside.read_text(encoding="utf-8") == "private\n"


def test_complete_attempts_legacy_api_is_absent(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    assert not hasattr(runtime.store, "complete_attempts")


def test_arbitrary_quarantine_name_and_forged_metadata_do_not_reserve(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    job_id_value = runtime.summaries()[0].identity.job_id
    with runtime.store.coordinator_lock():
        job_root = runtime.store.root / "quarantine" / job_id_value
        job_root.mkdir(parents=True)
        (job_root / "attempt-99").mkdir()
        (job_root / "q-0123456789abcdef0123456789abcdef").mkdir()
        (job_root / "q-0123456789abcdef0123456789abcdef.metadata.json").write_text(
            json.dumps(
                {
                    "quarantined_name": "q-0123456789abcdef0123456789abcdef",
                    "original_name": "attempt-7",
                    "original_attempt_number": 8,
                }
            )
            + "\n",
            encoding="utf-8",
        )
        assert runtime.store.used_attempt_numbers(job_id_value) == ()


def test_partial_final_attempt_is_ignored_on_recovery(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    summary = runtime.summaries()[0]
    partial = runtime.store.root / "attempts" / summary.identity.job_id / "attempt-99"
    partial.mkdir(parents=True)
    (partial / "context.json").write_text("{}", encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert tuple((runtime.store.root / "quarantine" / summary.identity.job_id).iterdir())


def test_cross_record_attempt_is_quarantined(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    attempt = next((runtime.store.root / "attempts" / completed.identity.job_id).glob("attempt-*"))
    context_path = attempt / "context.json"
    context = context_path.read_text().replace(completed.identity.job_id, "wrong-job")
    context_path.write_text(context, encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert tuple((runtime.store.root / "quarantine" / completed.identity.job_id).iterdir())


@pytest.mark.parametrize(
    ("record", "fields", "value"),
    [
        ("context.json", ("identity", "experiment_id"), "wrong-experiment"),
        ("context.json", ("identity", "case_id"), "wrong-case"),
        ("context.json", ("identity", "condition_name"), "wrong-condition"),
        ("context.json", ("identity", "job_id"), "wrong-job"),
        ("context.json", ("attempt_id",), "attempt-2"),
        ("context.json", ("attempt_number",), 2),
        ("context.json", ("directory_name",), "attempt-2"),
        ("context.json", ("manifest_digest",), "a" * 64),
        ("attempt-manifest.v1.json", ("identity", "experiment_id"), "wrong-experiment"),
        ("attempt-manifest.v1.json", ("identity", "case_id"), "wrong-case"),
        ("attempt-manifest.v1.json", ("identity", "condition_name"), "wrong-condition"),
        ("attempt-manifest.v1.json", ("identity", "job_id"), "wrong-job"),
        ("attempt-manifest.v1.json", ("attempt_id",), "attempt-2"),
        ("attempt-manifest.v1.json", ("manifest_digest",), "a" * 64),
        ("case.json", ("case_id",), "wrong-case"),
        ("case.json", ("payload", "index"), 99),
        ("condition.json", ("name",), "wrong-condition"),
        ("condition.json", ("payload", "changed"), True),
    ],
)
def test_every_cross_record_identity_field_is_quarantined(
    tmp_path: Path, record: str, fields: tuple[str, ...], value: object
) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    attempt = next((runtime.store.root / "attempts" / completed.identity.job_id).glob("attempt-1"))
    path = attempt / record
    document = json.loads(path.read_text(encoding="utf-8"))
    target = document
    for field in fields[:-1]:
        target = target[field]
    target[fields[-1]] = value
    path.write_text(json.dumps(document) + "\n", encoding="utf-8")
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert recovered.summaries()[0].state == "pending"
    assert tuple((runtime.store.root / "quarantine" / completed.identity.job_id).iterdir())


@pytest.mark.parametrize(
    "record",
    [
        "context.json",
        "attempt-manifest.v1.json",
        "case.json",
        "condition.json",
        "events.jsonl",
        "outcome.v1.json",
    ],
)
def test_record_symlink_is_quarantined_without_following(tmp_path: Path, record: str) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    completed = runtime.run()[0]
    attempt = next((runtime.store.root / "attempts" / completed.identity.job_id).glob("attempt-1"))
    outside = tmp_path / f"outside-{record.replace('.', '-')}"
    outside.write_text("outside\n", encoding="utf-8")
    (attempt / record).unlink()
    (attempt / record).symlink_to(outside)
    SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True).recover()
    assert outside.read_text(encoding="utf-8") == "outside\n"


def test_attempts_root_symlink_is_replaced_and_recreated(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    attempts = runtime.store.root / "attempts"
    external = tmp_path / "external-attempts"
    external.mkdir()
    attempts.rmdir()
    attempts.symlink_to(external, target_is_directory=True)
    recovered = SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True)
    recovered.recover()
    assert attempts.is_dir() and not attempts.is_symlink()
    assert recovered.run()[0].state == "succeeded"


def test_quarantine_fault_is_not_silently_recovered(tmp_path: Path, monkeypatch) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    invalid = runtime.store.root / "attempts" / "invalid-entry"
    invalid.write_text("invalid", encoding="utf-8")
    from . import scheduler_store

    original = scheduler_store._fsync_directory_fd
    calls = 0

    def fail_once(descriptor: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("injected fsync failure")
        original(descriptor)

    monkeypatch.setattr(scheduler_store, "_fsync_directory_fd", fail_once)
    with pytest.raises(OSError, match="injected fsync failure"):
        SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True).recover()


@pytest.mark.parametrize("stage", ["inode", "source-parent", "destination-parent"])
def test_post_rename_quarantine_fsync_stages_propagate(
    tmp_path: Path, monkeypatch, stage: str
) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    invalid = runtime.store.root / "attempts" / "invalid-entry"
    invalid.write_text("invalid", encoding="utf-8")
    from . import scheduler_store

    if stage == "inode":

        def fail_inode(parent_fd: int, name: str) -> None:
            raise OSError("injected inode fsync failure")

        monkeypatch.setattr(scheduler_store, "_set_safe_mode_and_fsync", fail_inode)
        message = "injected inode fsync failure"
    else:
        original = scheduler_store._fsync_directory_fd
        original_rename = scheduler_store.os.rename
        renamed = False
        post_rename_calls = 0
        failure_call = 1 if stage == "source-parent" else 2

        def mark_rename(
            source: str,
            destination: str,
            *,
            src_dir_fd: int | None = None,
            dst_dir_fd: int | None = None,
        ) -> None:
            nonlocal renamed
            original_rename(
                source,
                destination,
                src_dir_fd=src_dir_fd,
                dst_dir_fd=dst_dir_fd,
            )
            renamed = True

        def fail_directory(descriptor: int) -> None:
            nonlocal post_rename_calls
            if renamed:
                post_rename_calls += 1
            if post_rename_calls == failure_call:
                raise OSError(f"injected {stage} fsync failure")
            original(descriptor)

        monkeypatch.setattr(scheduler_store.os, "rename", mark_rename)
        monkeypatch.setattr(scheduler_store, "_fsync_directory_fd", fail_directory)
        message = f"injected {stage} fsync failure"

    with pytest.raises(OSError, match=message):
        SchedulerRuntime(runtime.store, FakeExecutor(), prerequisite=lambda: True).recover()


def test_recovery_api_is_strict_and_no_lenient_alias_exists(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FakeExecutor(), workers=1)
    assert not hasattr(runtime.store, "recover")
    with pytest.raises(StoreMutationError):
        runtime.store.recover_attempts("experiment", runtime.summaries()[0].identity.job_id)


class FatalExecutor:
    def run(
        self,
        case: ExpandedCase,
        condition: NamedCondition,
        context: AttemptContext,
        emit: EventSink,
        cancel_requested: Event,
        publication_lock: Lock,
    ) -> TerminalOutcome:
        raise KeyboardInterrupt


def test_fatal_executor_exception_propagates(tmp_path: Path) -> None:
    runtime = make_runtime(tmp_path, FatalExecutor(), workers=1)
    with pytest.raises(KeyboardInterrupt):
        runtime.run()
