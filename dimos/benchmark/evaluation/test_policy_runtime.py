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
from pathlib import Path
import queue

import cloudpickle
import pytest

from dimos.benchmark.evaluation.pi_process import PiRunResult
from dimos.benchmark.evaluation.protocol import PolicyArtifact, TrialOutcome, TrialRun
import dimos.benchmark.evaluation.runtime as runtime_module
from dimos.benchmark.evaluation.runtime import (
    CodePolicyRuntimeFactory,
    _execute_policy_worker,
    _SubmissionManager,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.porcelain.dimos import Dimos


def policy(app: Dimos) -> None:
    del app


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
    app = mocker.Mock(spec=Dimos)
    connect = mocker.patch.object(Dimos, "connect", return_value=app)
    results: queue.Queue[tuple[str, str | None]] = queue.Queue()

    _execute_policy_worker(str(serialized_path), results)

    assert results.get_nowait() == ("completed", None)
    connect.assert_called_once_with()
    app.stop.assert_called_once_with()


def test_explore_freezes_last_of_five_debug_submissions(mocker, tmp_path: Path) -> None:
    class FakeServer:
        current = None
        mcp_url = "http://127.0.0.1:1/mcp"

        def __init__(self, submission_handler) -> None:
            self.submission_handler = submission_handler
            FakeServer.current = self

        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass

    class FakePiRunner:
        def __init__(self, **_kwargs) -> None:
            pass

        def run(self, **_kwargs) -> PiRunResult:
            assert FakeServer.current is not None
            serialized = cloudpickle.dumps(policy)
            for _ in range(5):
                FakeServer.current.submission_handler(
                    "def policy(app: Dimos) -> None: ...",
                    serialized,
                )
            return PiRunResult("done", 5, 2.0, None, "")

    marker = tmp_path / "pi"
    marker.touch()
    mocker.patch.object(runtime_module, "CodePolicyMcpServer", FakeServer)
    mocker.patch.object(runtime_module, "PiCliRunner", FakePiRunner)
    mocker.patch.object(runtime_module, "_pi_paths", return_value=(marker, marker))
    runtime = CodePolicyRuntimeFactory(api_key="secret", workspace=tmp_path)

    outcome = runtime.explore(
        evaluation_protocol="Use normal DimOS information.",
        task_input="Complete the task.",
        submit_debug_trial=lambda _policy, number, path: _trial(path, number),
    )

    assert outcome.status == "valid"
    assert len(outcome.trials) == 5
    assert outcome.policy is not None
    assert outcome.policy.serialized_path.parent.name == "submission-0005"
