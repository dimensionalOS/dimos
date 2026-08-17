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

from __future__ import annotations

import json
import multiprocessing
from pathlib import Path
import threading

import cloudpickle
import pytest

from dimos.benchmark.evaluation.models import RuntimeCondition
from dimos.benchmark.evaluation.pi_process import PiRunError, PiRunResult
from dimos.benchmark.evaluation.protocol import PolicyArtifact, TrialOutcome, TrialRun
import dimos.benchmark.evaluation.runtime as runtime_module
from dimos.benchmark.evaluation.runtime import (
    CodePolicyRuntimeFactory,
    _execute_policy_worker,
    _PolicyExecutionProcess,
    _SubmissionManager,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.porcelain.dimos import Dimos


def policy(app: Dimos) -> None:
    del app


class _FakeCodePolicyServer:
    current = None
    mcp_url = "http://127.0.0.1:1/mcp"

    def __init__(self, submission_handler) -> None:
        self.submission_handler = submission_handler
        _FakeCodePolicyServer.current = self

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _send_large_policy_error(messages, start_event, result_sending) -> None:
    start_event.wait()
    result_sending.set()
    messages.send(("policy_error", "x" * 1_000_000))
    messages.close()


def _trial(path: Path, number: int) -> TrialRun:
    path.mkdir(exist_ok=True)
    log_path = path / "main.jsonl"
    log_path.write_text(json.dumps({"module": "Planner", "number": number}) + "\n")
    memory_path = path / "recording.db"
    with SqliteStore(path=str(memory_path)) as memory:
        memory.stream("events", int).append(number)
    return TrialRun(
        run_id=f"debug-{number}",
        outcome=TrialOutcome(
            success=number == 5,
            reward=float(number),
            status="completed",
            error=None,
            duration_seconds=1.0,
        ),
        artifacts=path,
        log_path=log_path,
        memory_path=memory_path,
    )


def test_submission_manager_caps_trials_and_selects_last_policy(tmp_path: Path) -> None:
    artifacts = []
    manager = _SubmissionManager(
        workspace=tmp_path,
        path=tmp_path / "exploration",
        relative_path=Path("exploration"),
        submit_debug_trial=lambda _policy, number, path: _trial(path, number),
        max_submissions=5,
        record_artifact=artifacts.append,
    )
    manager.path.mkdir()
    serialized = cloudpickle.dumps(policy)

    trials = [manager.submit("def policy(app: Dimos) -> None: ...", serialized) for _ in range(5)]

    assert [trial.run_id for trial in trials] == [f"debug-{number}" for number in range(1, 6)]
    assert manager.last_policy is not None
    assert manager.last_policy.serialized_path.parent.name == "submission-0005"
    with pytest.raises(RuntimeError, match="budget exhausted"):
        manager.submit("def policy(app: Dimos) -> None: ...", serialized)


def test_trial_run_exposes_logs_and_read_only_memory(tmp_path: Path) -> None:
    trial = _trial(tmp_path / "trial", 2)

    assert trial.read_logs(module="Planner") == ({"module": "Planner", "number": 2},)
    with trial.open_memory() as memory:
        assert memory.streams.events.last().data == 2
        with pytest.raises(PermissionError):
            memory.streams.events.append(3)


def test_policy_artifact_records_serialized_callable(tmp_path: Path) -> None:
    serialized_path = tmp_path / "policy.pkl"
    serialized_path.write_bytes(cloudpickle.dumps(policy))
    artifact = PolicyArtifact(
        source_path=tmp_path / "policy.py",
        serialized_path=serialized_path,
        sha256="digest",
    )

    with artifact.serialized_path.open("rb") as handle:
        loaded = cloudpickle.load(handle)

    assert loaded.__name__ == "policy"


def test_policy_worker_connects_and_invokes_callable(mocker, tmp_path: Path) -> None:
    serialized_path = tmp_path / "policy.pkl"
    serialized_path.write_bytes(cloudpickle.dumps(policy))
    memory_path = tmp_path / "recording.db"
    with SqliteStore(path=str(memory_path)) as memory:
        memory.stream("events", int).append(1)
    app = mocker.Mock(spec=Dimos)
    connect = mocker.patch.object(Dimos, "connect", return_value=app)
    results, worker_results = multiprocessing.Pipe(duplex=False)
    start_event = threading.Event()

    worker = threading.Thread(
        target=_execute_policy_worker,
        args=(str(serialized_path), str(memory_path), worker_results, start_event),
    )
    worker.start()

    assert results.poll(1)
    assert results.recv() == ("ready", None)
    connect.assert_called_once()
    attached = connect.call_args.kwargs["memory"]
    assert attached.config.read_only is True
    start_event.set()
    assert results.poll(1)
    assert results.recv() == ("completed", None)
    worker.join(timeout=1)
    assert not worker.is_alive()
    app.stop.assert_called_once_with()
    results.close()
    worker_results.close()


def test_prepared_policy_waits_for_explicit_start_and_stops_at_trial_end(mocker) -> None:
    process = mocker.Mock()
    process.is_alive.return_value = True
    messages = mocker.Mock()
    messages.poll.return_value = False
    start_event = threading.Event()
    execution = _PolicyExecutionProcess(process, messages, start_event)

    assert not start_event.is_set()
    execution.start()
    assert start_event.is_set()

    result = execution.finish(grace_s=0.0)

    assert result.status == "stopped"
    process.terminate.assert_called_once_with()


def test_policy_result_larger_than_pipe_buffer_does_not_deadlock() -> None:
    context = multiprocessing.get_context("spawn")
    messages, worker_messages = context.Pipe(duplex=False)
    start_event = context.Event()
    result_sending = context.Event()
    process = context.Process(
        target=_send_large_policy_error,
        args=(worker_messages, start_event, result_sending),
        daemon=True,
    )
    process.start()
    worker_messages.close()
    execution = _PolicyExecutionProcess(process, messages, start_event)
    execution.start()
    assert result_sending.wait(timeout=2)

    result = execution.finish(grace_s=0.01)

    assert result.status == "policy_error"
    assert result.error == "x" * 1_000_000


def test_explore_freezes_last_of_five_debug_submissions(mocker, tmp_path: Path) -> None:
    class FakePiRunner:
        def __init__(self, **_kwargs) -> None:
            pass

        def run(self, **_kwargs) -> PiRunResult:
            assert _FakeCodePolicyServer.current is not None
            serialized = cloudpickle.dumps(policy)
            for _ in range(5):
                _FakeCodePolicyServer.current.submission_handler(
                    "def policy(app: Dimos) -> None: ...",
                    serialized,
                )
            return PiRunResult("done", 5, 2.0, None, "")

    marker = tmp_path / "pi"
    marker.touch()
    mocker.patch.object(runtime_module, "CodePolicyMcpServer", _FakeCodePolicyServer)
    mocker.patch.object(runtime_module, "PiCliRunner", FakePiRunner)
    mocker.patch.object(runtime_module, "_pi_paths", return_value=(marker, marker))
    runtime = CodePolicyRuntimeFactory(
        api_key="secret",
        workspace=tmp_path,
        condition=RuntimeCondition(model="gpt-5.6-luna", thinking_level="medium"),
    )

    outcome = runtime.explore(
        evaluation_protocol="Use normal DimOS information.",
        task_input="Complete the task.",
        submit_debug_trial=lambda _policy, number, path: _trial(path, number),
    )

    assert outcome.status == "valid"
    assert len(outcome.trials) == 5
    assert outcome.policy is not None
    assert outcome.policy.serialized_path.parent.name == "submission-0005"


def test_explore_publishes_jsonl_and_rendered_transcript(mocker, tmp_path: Path) -> None:
    class FakePiRunner:
        def __init__(self, **_kwargs) -> None:
            pass

        def run(self, **kwargs) -> PiRunResult:
            transcript = kwargs["run_dir"] / "pi-session" / "session.jsonl"
            transcript.parent.mkdir()
            transcript.write_text('{"type":"session"}\n')
            return PiRunResult("done", 1, 2.0, transcript, "")

        def export_transcript(self, **kwargs) -> None:
            kwargs["output_path"].write_text("<html>rendered</html>")

    marker = tmp_path / "pi"
    marker.touch()
    mocker.patch.object(runtime_module, "CodePolicyMcpServer", _FakeCodePolicyServer)
    mocker.patch.object(runtime_module, "PiCliRunner", FakePiRunner)
    mocker.patch.object(runtime_module, "_pi_paths", return_value=(marker, marker))
    runtime = CodePolicyRuntimeFactory(
        api_key="secret",
        workspace=tmp_path,
        condition=RuntimeCondition(model="gpt-5.6-luna", thinking_level="medium"),
    )

    outcome = runtime.explore(
        evaluation_protocol="Use normal DimOS information.",
        task_input="Complete the task.",
        submit_debug_trial=lambda _policy, number, path: _trial(path, number),
    )

    exploration = tmp_path / "runtime" / "exploration-0001"
    assert outcome.status == "invalid"
    assert (exploration / "pi-transcript.jsonl").read_text() == '{"type":"session"}\n'
    assert (exploration / "pi-transcript.html").read_text() == "<html>rendered</html>"
    artifacts = {artifact.path: artifact.media_type for artifact in runtime.runtime_artifacts}
    assert artifacts["runtime/exploration-0001/pi-transcript.jsonl"] == ("application/x-ndjson")
    assert artifacts["runtime/exploration-0001/pi-transcript.html"] == "text/html"


def test_explore_retains_transcript_when_pi_fails(mocker, tmp_path: Path) -> None:
    class FakePiRunner:
        def __init__(self, **_kwargs) -> None:
            pass

        def run(self, **kwargs) -> PiRunResult:
            transcript = kwargs["run_dir"] / "pi-session" / "session.jsonl"
            transcript.parent.mkdir()
            transcript.write_text('{"type":"session"}\n')
            raise PiRunError("agent stopped", transcript_path=transcript)

        def export_transcript(self, **kwargs) -> None:
            kwargs["output_path"].write_text("<html>failure</html>")

    marker = tmp_path / "pi"
    marker.touch()
    mocker.patch.object(runtime_module, "CodePolicyMcpServer", _FakeCodePolicyServer)
    mocker.patch.object(runtime_module, "PiCliRunner", FakePiRunner)
    mocker.patch.object(runtime_module, "_pi_paths", return_value=(marker, marker))
    runtime = CodePolicyRuntimeFactory(
        api_key="secret",
        workspace=tmp_path,
        condition=RuntimeCondition(model="gpt-5.6-luna", thinking_level="medium"),
    )

    with pytest.raises(PiRunError, match="agent stopped"):
        runtime.explore(
            evaluation_protocol="Use normal DimOS information.",
            task_input="Complete the task.",
            submit_debug_trial=lambda _policy, number, path: _trial(path, number),
        )

    exploration = tmp_path / "runtime" / "exploration-0001"
    assert (exploration / "pi-transcript.jsonl").is_file()
    assert (exploration / "pi-transcript.html").read_text() == "<html>failure</html>"


def test_transcript_export_failure_is_diagnostic_only(mocker, tmp_path: Path) -> None:
    class FakePiRunner:
        def __init__(self, **_kwargs) -> None:
            pass

        def run(self, **kwargs) -> PiRunResult:
            transcript = kwargs["run_dir"] / "pi-session" / "session.jsonl"
            transcript.parent.mkdir()
            transcript.write_text('{"type":"session"}\n')
            return PiRunResult("done", 1, 2.0, transcript, "")

        def export_transcript(self, **_kwargs) -> None:
            raise RuntimeError("secret could not render")

    marker = tmp_path / "pi"
    marker.touch()
    progress = []
    mocker.patch.object(runtime_module, "CodePolicyMcpServer", _FakeCodePolicyServer)
    mocker.patch.object(runtime_module, "PiCliRunner", FakePiRunner)
    mocker.patch.object(runtime_module, "_pi_paths", return_value=(marker, marker))
    runtime = CodePolicyRuntimeFactory(
        api_key="secret",
        workspace=tmp_path,
        condition=RuntimeCondition(model="gpt-5.6-luna", thinking_level="medium"),
        progress=progress.append,
    )

    outcome = runtime.explore(
        evaluation_protocol="Use normal DimOS information.",
        task_input="Complete the task.",
        submit_debug_trial=lambda _policy, number, path: _trial(path, number),
    )

    exploration = tmp_path / "runtime" / "exploration-0001"
    assert outcome.status == "invalid"
    assert (exploration / "pi-transcript.jsonl").is_file()
    assert not (exploration / "pi-transcript.html").exists()
    assert (exploration / "pi-transcript-export-error.log").read_text() == (
        "RuntimeError: [REDACTED] could not render\n"
    )
    assert progress[-1].message == (
        "transcript export failed: RuntimeError: [REDACTED] could not render"
    )
