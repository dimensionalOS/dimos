import json
from pathlib import Path
import subprocess
from threading import Event, Thread, current_thread

import pytest

from dimos.benchmark.spatial.pi_baseline.cli_support import (
    create_experiment,
    execute_pi_operation,
    validate_pi_definition,
)
from dimos.benchmark.spatial.pi_baseline.scheduler_models import TerminalOutcome
from dimos.benchmark.spatial.pi_baseline.scheduler_pi_executor import PiSchedulerExecutor
from dimos.benchmark.spatial.pi_baseline.scheduler_store import CoordinatorLockError

from .test_scheduler_pi_executor import (
    _SELECTION,
    _config,
    _preflight_fixture,
    _result,
)


def _definition_spec(tmp_path: Path) -> tuple[Path, Path]:
    config = _config(tmp_path)
    config_path = tmp_path / "config.json"
    config_path.write_text(json.dumps(config.model_dump(mode="json")), encoding="utf-8")
    spec_path = tmp_path / "spec.json"
    spec_path.write_text(
        json.dumps(
            {
                "experiment_id": "experiment-1",
                "pi_config": config_path.name,
                "cases": [{"case_id": "case-1", "selection": _SELECTION}],
                "conditions": [{"name": "pi-condition", "prompt_mode": "visualization-forbidden"}],
            }
        ),
        encoding="utf-8",
    )
    return spec_path, tmp_path / "experiment-1"


def _tree_snapshot(root: Path) -> dict[str, bytes]:
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): path.read_bytes()
        for path in root.rglob("*")
        if path.is_file() and not path.name.endswith(".coordinator.lock")
    }


def _podman_popen(command: list[str], markers: list[str]):
    """Model the completed Podman commands used by scheduler cleanup."""

    class PodmanProcess:
        def __init__(self) -> None:
            self.stdout = None
            self.stderr = None
            if command[1:3] == ["container", "exists"]:
                self.returncode = 1
                markers.append("container_absence_verified")
            else:
                self.returncode = 0
                if len(command) > 1 and command[1] == "rm":
                    markers.append("container_removed")

        def poll(self) -> int:
            return self.returncode

        def communicate(self) -> tuple[str, str]:
            if len(command) > 1 and command[1] == "inspect":
                return ("true\n", "")
            if len(command) > 1 and command[1] == "info":
                return ("true", "")
            if len(command) > 1 and command[1] == "exec":
                if "print('READY')" in command[-1]:
                    return ("READY\n", "")
                return ("", "")
            return ("", "")

        def wait(self, timeout: float | None = None) -> int:
            del timeout
            return self.returncode

        def terminate(self) -> None:
            pass

        def kill(self) -> None:
            pass

    return PodmanProcess()


def test_create_experiment_publishes_to_explicit_root(tmp_path: Path) -> None:
    spec, experiment = _definition_spec(tmp_path)
    create_experiment(experiment, spec, workers=1, sample=None, shard=0, shards=1)
    assert (experiment / "manifest.json").is_file()
    assert (experiment / "plan.json").is_file()
    assert not (experiment.parent / "manifest.json").exists()
    assert validate_pi_definition(experiment).manifest.experiment_id == "experiment-1"


def test_factory_run_uses_typed_host_and_isolates_multi_job_state(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime, experiment, bindings = _preflight_fixture(tmp_path, case_count=2)
    configs = []
    from dimos.benchmark.spatial.pi_baseline import scheduler_pi_executor

    def capture_outputs(config, mode, cancel_requested, publication_lock):
        configs.append(config)
        public = Path(config.output_root)
        private = Path(config.private_root)
        public.mkdir(parents=True, exist_ok=True)
        private.mkdir(parents=True, exist_ok=True)
        (public / "marker").write_text(config.run_id, encoding="utf-8")
        (private / "marker").write_text(config.run_id, encoding="utf-8")
        return _result(mode)

    monkeypatch.setattr(scheduler_pi_executor, "run_condition", capture_outputs)
    results = execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    assert results
    assert all(item.identity.experiment_id == experiment.name for item in results)
    attempts = list((experiment / "attempts").rglob("attempt-*/context.json"))
    assert len(attempts) == len(results)
    assert len({path.parent.parent.name for path in attempts}) == len(results)
    assert len(configs) == 2
    assert len({config.run_id for config in configs}) == 2
    assert all("/experiment-1/" in config.private_root for config in configs)
    assert all("/attempt-1/private" in config.private_root for config in configs)
    assert all("/experiment-1/" in config.output_root for config in configs)
    assert all("/attempt-1/public" in config.output_root for config in configs)
    assert len({config.output_root for config in configs}) == 2
    assert len({config.private_root for config in configs}) == 2
    assert {
        Path(config.output_root, "marker").read_text(encoding="utf-8") for config in configs
    } == {config.run_id for config in configs}
    assert {
        Path(config.private_root, "marker").read_text(encoding="utf-8") for config in configs
    } == {config.run_id for config in configs}
    assert not (bindings.ledger_path.parent / "ledger.jsonl").exists()


def test_factory_resume_and_retry_use_reconciled_immutable_outcomes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime, _, bindings = _preflight_fixture(tmp_path)
    monkeypatch.setattr(
        PiSchedulerExecutor,
        "run",
        lambda self,
        case,
        condition,
        context,
        emit,
        cancel_requested,
        publication_lock: TerminalOutcome(status="succeeded", reason="completed"),
    )
    first = execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    assert first and first[0].state == "succeeded"
    assert execute_pi_operation(runtime, bindings, "resume", host_prerequisite=lambda: True) == ()
    with pytest.raises(ValueError, match="only the latest"):
        execute_pi_operation(
            runtime,
            bindings,
            "retry",
            job_id_value=first[0].identity.job_id,
            reason="retry succeeded",
            host_prerequisite=lambda: True,
        )

    for status in ("failed", "interrupted", "cancelled"):
        status_root = tmp_path / status
        status_root.mkdir()
        runtime, _, bindings = _preflight_fixture(status_root)
        monkeypatch.setattr(
            PiSchedulerExecutor,
            "run",
            lambda self,
            case,
            condition,
            context,
            emit,
            cancel_requested,
            publication_lock,
            status=status: TerminalOutcome(status=status, reason=status),
        )
        initial = execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)[0]
        summary_path = runtime.store.root / "jobs" / f"{initial.identity.job_id}.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["state"] = "succeeded"
        summary_path.write_text(json.dumps(summary) + "\n", encoding="utf-8")
        retried = execute_pi_operation(
            runtime,
            bindings,
            "retry",
            job_id_value=initial.identity.job_id,
            reason=f"retry {status}",
            host_prerequisite=lambda: True,
        )[0]
        assert retried.state == status
        assert retried.latest_attempt_id == "attempt-2"
        assert (runtime.store.root / "attempts" / initial.identity.job_id / "attempt-2").is_dir()


def test_scheduler_cleanup_order_persists_one_interrupted_outcome_last(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime, _, bindings = _preflight_fixture(tmp_path)
    markers: list[str] = []
    stdout_started = Event()
    stderr_started = Event()
    release_readers = Event()

    class Stream:
        def __init__(self, name: str) -> None:
            self.name = name
            self.closed = False

        def __iter__(self):
            markers.append(f"{self.name}_reader_started")
            (stdout_started if self.name == "stdout" else stderr_started).set()
            release_readers.wait(2)
            markers.append(f"{self.name}_reader_exit")
            return iter(())

        def read(self, _size: int = -1) -> str:
            markers.append(f"{self.name}_reader_started")
            (stdout_started if self.name == "stdout" else stderr_started).set()
            release_readers.wait(2)
            markers.append(f"{self.name}_reader_exit")
            return ""

        def close(self) -> None:
            self.closed = True

    class NodeProcess:
        def __init__(self) -> None:
            self.stdin = type(
                "Stdin",
                (),
                {
                    "write": lambda self, _: None,
                    "flush": lambda self: None,
                    "close": lambda self: None,
                },
            )()
            self.stdout = Stream("stdout")
            self.stderr = Stream("stderr")
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def terminate(self) -> None:
            markers.append("process_terminated")
            self.returncode = 0
            release_readers.set()

        def kill(self) -> None:
            self.terminate()

        def wait(self, timeout: float | None = None) -> int:
            del timeout
            self.returncode = 0
            return 0

    def popen(command, **_kwargs):
        if command[0] != "podman" or command[1:2] == ["run"]:
            return NodeProcess()
        if (
            command[1:2] == ["exec"]
            and "python -m venv" not in command[-1]
            and "print('READY')" not in command[-1]
        ):
            return NodeProcess()
        return _podman_popen(command, markers)

    monkeypatch.setattr(subprocess, "Popen", popen)
    operation_result: list[object] = []
    errors: list[BaseException] = []
    original_write = runtime.store.write_outcome

    def write_outcome(context, outcome):
        assert markers.index("process_terminated") < markers.index("stdout_reader_exit")
        assert markers.index("process_terminated") < markers.index("stderr_reader_exit")
        assert markers.index("stdout_reader_exit") < markers.index("container_removed")
        assert markers.index("stderr_reader_exit") < markers.index("container_removed")
        assert markers.index("container_removed") < markers.index("container_absence_verified")
        markers.append("scheduler_outcome_persisted")
        return original_write(context, outcome)

    monkeypatch.setattr(runtime.store, "write_outcome", write_outcome)

    def operate() -> None:
        try:
            operation_result.append(
                execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
            )
        except BaseException as error:
            errors.append(error)

    operation = Thread(target=operate)
    operation.start()
    operation.join(0.1)
    assert operation.is_alive(), (errors, operation_result, markers, runtime.summaries())
    assert stdout_started.wait(2), (errors, operation_result, runtime.summaries())
    assert stderr_started.wait(2)
    runtime.cancel()
    operation.join(5)
    assert not operation.is_alive()
    assert not errors, errors
    assert markers[-1] == "scheduler_outcome_persisted"
    assert markers.index("process_terminated") < markers.index("stdout_reader_exit")
    assert markers.index("stdout_reader_exit") < markers.index("container_removed")
    assert markers.index("container_removed") < markers.index("container_absence_verified")
    attempt = next((runtime.store.root / "attempts").rglob("attempt-1"))
    outcome = json.loads((attempt / "outcome.v1.json").read_text())
    events = [json.loads(line) for line in (attempt / "events.jsonl").read_text().splitlines()]
    assert outcome["status"] == "interrupted"
    assert outcome["reason"] == "executor_interrupted"
    assert len(list((runtime.store.root / "attempts").rglob("outcome.v1.json"))) == 1
    finished = [event for event in events if event["kind"] == "finished"]
    assert len(finished) == 1 and finished[0]["message"] == "executor_interrupted"


def test_stubborn_reader_cleanup_is_failed_and_never_interrupted(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime, _, bindings = _preflight_fixture(tmp_path)
    markers: list[str] = []
    stdout_started = Event()
    stderr_started = Event()
    release_stderr = Event()
    release_stubborn = Event()
    stubborn_thread: Thread | None = None

    class Stream:
        def __init__(self, name: str, stubborn: bool = False) -> None:
            self.name = name
            self.stubborn = stubborn

        def __iter__(self):
            markers.append(f"{self.name}_reader_started")
            nonlocal stubborn_thread
            if self.name == "stdout":
                stdout_started.set()
                stubborn_thread = current_thread()
            else:
                stderr_started.set()
            (release_stubborn if self.stubborn else release_stderr).wait(2)
            markers.append(f"{self.name}_reader_exit")
            return iter(())

        def read(self, _size: int = -1) -> str:
            markers.append(f"{self.name}_reader_started")
            nonlocal stubborn_thread
            if self.name == "stdout":
                stdout_started.set()
                stubborn_thread = current_thread()
            else:
                stderr_started.set()
            (release_stubborn if self.stubborn else release_stderr).wait(2)
            markers.append(f"{self.name}_reader_exit")
            return ""

        def close(self) -> None:
            markers.append(f"{self.name}_close_attempted")
            if not self.stubborn:
                release_stubborn.set()

    class NodeProcess:
        def __init__(self) -> None:
            self.stdin = type(
                "Stdin",
                (),
                {
                    "write": lambda self, _: None,
                    "flush": lambda self: None,
                    "close": lambda self: None,
                },
            )()
            self.stdout = Stream("stdout", stubborn=True)
            self.stderr = Stream("stderr")
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def terminate(self) -> None:
            markers.append("process_terminated")
            self.returncode = 0
            release_stderr.set()

        def kill(self) -> None:
            self.terminate()

        def wait(self, timeout: float | None = None) -> int:
            del timeout
            self.returncode = 0
            return 0

    def popen(command, **_kwargs):
        if command[0] != "podman" or command[1:2] == ["run"]:
            return NodeProcess()
        if (
            command[1:2] == ["exec"]
            and "python -m venv" not in command[-1]
            and "print('READY')" not in command[-1]
        ):
            return NodeProcess()
        return _podman_popen(command, markers)

    monkeypatch.setattr(subprocess, "Popen", popen)
    operation_result: list[object] = []
    errors: list[BaseException] = []
    original_write = runtime.store.write_outcome

    def write_outcome(context, outcome):
        assert markers.index("container_removed") < markers.index("container_absence_verified")
        markers.append("scheduler_outcome_persisted")
        return original_write(context, outcome)

    monkeypatch.setattr(runtime.store, "write_outcome", write_outcome)

    def operate() -> None:
        try:
            operation_result.append(
                execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
            )
        except BaseException as error:
            errors.append(error)

    operation = Thread(target=operate)
    operation.start()
    operation.join(0.1)
    assert operation.is_alive(), (errors, operation_result, runtime.summaries())
    assert stdout_started.wait(2), (errors, operation_result, runtime.summaries())
    assert stderr_started.wait(2)
    operation.join(8)
    assert not operation.is_alive()
    assert markers[-1] == "scheduler_outcome_persisted"
    attempt = next((runtime.store.root / "attempts").rglob("attempt-1"))
    outcome = json.loads((attempt / "outcome.v1.json").read_text())
    events = [json.loads(line) for line in (attempt / "events.jsonl").read_text().splitlines()]
    assert outcome["status"] == "failed"
    assert outcome["reason"] == "executor_failed"
    assert outcome["status"] != "interrupted"
    finished = [event for event in events if event["kind"] == "finished"]
    assert len(finished) == 1 and finished[0]["message"] == "executor_failed"
    assert not any(event["message"] == "executor_interrupted" for event in finished)
    assert markers.index("container_absence_verified") < markers.index(
        "scheduler_outcome_persisted"
    )
    release_stubborn.set()
    # The stubborn reader is intentionally released only after persistence assertions.
    assert stubborn_thread is not None
    stubborn_thread.join(2)
    assert markers.count("stdout_reader_exit") == 1


def test_factory_host_rejection_is_state_free(tmp_path: Path) -> None:
    runtime, experiment, bindings = _preflight_fixture(tmp_path)
    before_experiment = _tree_snapshot(experiment)
    before_private = _tree_snapshot(bindings.private_root)
    with pytest.raises(RuntimeError, match="host prerequisite"):
        execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: False)
    assert _tree_snapshot(experiment) == before_experiment
    assert _tree_snapshot(bindings.private_root) == before_private


def test_factory_public_material_drift_is_state_free(tmp_path: Path) -> None:
    runtime, experiment, bindings = _preflight_fixture(tmp_path)
    manifest_path = bindings.corpus_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["release_version"] = "drifted"
    manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")
    before_experiment = _tree_snapshot(experiment)
    before_private = _tree_snapshot(bindings.private_root)
    with pytest.raises(
        ValueError, match="selected public input drift|staging inventory drift|release"
    ):
        execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    assert _tree_snapshot(experiment) == before_experiment
    assert _tree_snapshot(bindings.private_root) == before_private


def test_factory_private_drift_rejection_is_state_free(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime, experiment, bindings = _preflight_fixture(tmp_path)
    from dimos.benchmark.spatial.pi_baseline import cli_support

    original = cli_support.validate_pi_definition

    def validate_then_drift(path: Path, current_bindings):
        result = original(path, current_bindings)
        answer = next(current_bindings.oracle_root.rglob("answers.jsonl"))
        answer.write_bytes(answer.read_bytes() + b"drift")
        return result

    monkeypatch.setattr(cli_support, "validate_pi_definition", validate_then_drift)
    before_experiment = _tree_snapshot(experiment)
    before_private = _tree_snapshot(bindings.private_root)
    with pytest.raises(ValueError, match="private binding changed"):
        execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    assert _tree_snapshot(experiment) == before_experiment
    assert _tree_snapshot(bindings.private_root) == before_private


def test_factory_rejected_coordinator_is_state_free(tmp_path: Path) -> None:
    runtime, experiment, bindings = _preflight_fixture(tmp_path)
    entered = Event()
    release = Event()

    def owner() -> None:
        with runtime.store.coordinator_lease():
            entered.set()
            release.wait(2)

    thread = Thread(target=owner)
    thread.start()
    assert entered.wait(2)
    before_experiment = _tree_snapshot(experiment)
    before_private = _tree_snapshot(bindings.private_root)
    try:
        with pytest.raises(CoordinatorLockError):
            execute_pi_operation(runtime, bindings, "run", host_prerequisite=lambda: True)
    finally:
        release.set()
        thread.join(2)
    assert _tree_snapshot(experiment) == before_experiment
    assert _tree_snapshot(bindings.private_root) == before_private


def test_factory_rejects_generic_pi_runtime_bypass(tmp_path: Path) -> None:
    runtime, _, _ = _preflight_fixture(tmp_path)
    with pytest.raises(RuntimeError, match="execute_pi_operation"):
        runtime.run()
