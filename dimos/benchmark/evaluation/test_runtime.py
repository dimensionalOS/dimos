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

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from dimos.agents.code_policy_core import (
    CodeExecutionRecord,
    FrozenMemoryEnvironment,
    PolicyExecutionResult,
)
from dimos.benchmark.evaluation.models import CodePolicyAgentConfig
from dimos.benchmark.evaluation.pi_process import PiRunError, PiRunResult
import dimos.benchmark.evaluation.runtime as runtime


class FakeServer:
    mcp_url = "http://127.0.0.1:1234/mcp"
    current = None

    def __init__(self, config) -> None:
        self.config = config
        self.session = SimpleNamespace(config=config, execution_records=[])
        self.started = False
        FakeServer.current = self

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False


class FakeRunner:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs

    def run(self, *, run_dir: Path, prompt: str, system_prompt: str, **_kwargs):
        transcript = run_dir / "native.jsonl"
        transcript.write_text('{"type":"session"}\n')
        return PiRunResult("ANSWER: 2", 3, 1.0, transcript, "")


class FailingRunner(FakeRunner):
    def run(self, **_kwargs):
        raise PiRunError("Pi failed", stderr="diagnostic")


class PolicyRunner(FakeRunner):
    sources: list[str] = []
    prompts: list[str] = []

    def run(self, *, prompt: str, **kwargs):
        self.prompts.append(prompt)
        source = self.sources.pop(0)
        assert FakeServer.current is not None
        FakeServer.current.session.execution_records.append(
            CodeExecutionRecord(source, "completed", "(completed)", 0.01)
        )
        return super().run(prompt=prompt, **kwargs)


def test_runtime_records_separate_prompt_components(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runtime, "CodePolicyMcpServer", FakeServer)
    monkeypatch.setattr(runtime, "PiCliRunner", FakeRunner)
    marker = tmp_path / "exists"
    marker.touch()
    monkeypatch.setattr(runtime, "_pi_paths", lambda: (marker, marker))
    factory = runtime.CodePolicyRuntimeFactory(
        config=CodePolicyAgentConfig(),
        api_key="secret",
        workspace=tmp_path,
    )
    environment = FrozenMemoryEnvironment(
        recording_path="source.db",
        derived_recording_path="derived.db",
        memory_cutoff_timestamp=1.0,
    )

    with factory.open_session(environment) as session:
        outcome = session.answer(
            evaluation_protocol="End with ANSWER: <integer>.",
            task_input="How many rooms?",
        )

    assert outcome.final_text == "ANSWER: 2"
    session_path = tmp_path / "runtime" / "session-0001"
    assert (session_path / "evaluation-protocol.txt").read_text() == ("End with ANSWER: <integer>.")
    assert (session_path / "task-input.txt").read_text() == "How many rooms?"
    assembly = json.loads((session_path / "prompt-assembly.json").read_text())
    task = next(item for item in assembly["components"] if item["path"].endswith("task-input.txt"))
    assert task["owner"] == "evaluation"
    assert task["sha256"] == hashlib.sha256(b"How many rooms?").hexdigest()
    assert not (session_path / "working").exists()
    assert {item.path for item in factory.prompt_evidence} >= {
        "runtime/session-0001/runtime-system.txt",
        "runtime/session-0001/evaluation-protocol.txt",
        "runtime/session-0001/task-input.txt",
    }


def test_runtime_retains_pi_stderr_on_failure(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runtime, "CodePolicyMcpServer", FakeServer)
    monkeypatch.setattr(runtime, "PiCliRunner", FailingRunner)
    marker = tmp_path / "exists"
    marker.touch()
    monkeypatch.setattr(runtime, "_pi_paths", lambda: (marker, marker))
    factory = runtime.CodePolicyRuntimeFactory(
        config=CodePolicyAgentConfig(),
        api_key="secret",
        workspace=tmp_path,
    )
    environment = FrozenMemoryEnvironment(
        recording_path="source.db",
        derived_recording_path="derived.db",
        memory_cutoff_timestamp=1.0,
    )

    with pytest.raises(PiRunError, match="Pi failed"):
        with factory.open_session(environment) as session:
            session.answer(evaluation_protocol="Use memory.", task_input="Question")

    assert (tmp_path / "runtime/session-0001/stderr.log").read_text() == "diagnostic"
    assert factory.runtime_artifacts[-1].path == "runtime/session-0001/stderr.log"


def test_policy_authoring_repairs_mechanical_failure_and_records_artifacts(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runtime, "CodePolicyMcpServer", FakeServer)
    monkeypatch.setattr(runtime, "PiCliRunner", PolicyRunner)
    marker = tmp_path / "exists"
    marker.touch()
    monkeypatch.setattr(runtime, "_pi_paths", lambda: (marker, marker))
    PolicyRunner.sources = [
        "def policy():\n    return missing",
        "def policy():\n    return 2",
    ]
    PolicyRunner.prompts = []
    validations = iter(
        [
            PolicyExecutionResult(valid=False, error="NameError: missing", duration_seconds=0.1),
            PolicyExecutionResult(valid=True, result=2, duration_seconds=0.1),
        ]
    )
    monkeypatch.setattr(runtime, "validate_policy_source", lambda *_args: next(validations))
    factory = runtime.CodePolicyRuntimeFactory(
        config=CodePolicyAgentConfig(),
        api_key="secret",
        workspace=tmp_path,
    )
    environment = FrozenMemoryEnvironment(
        recording_path="source.db",
        derived_recording_path="derived.db",
        memory_cutoff_timestamp=1.0,
    )

    with factory.open_session(environment) as session:
        outcome = session.author_policy(
            evaluation_protocol="Return an integer from policy().",
            task_input="How many rooms?",
            max_rounds=3,
        )

    assert outcome.status == "valid"
    assert outcome.result == 2
    assert len(outcome.validations) == 2
    assert "NameError: missing" in PolicyRunner.prompts[1]
    session_path = tmp_path / "runtime/session-0001"
    assert (session_path / "policy.py").read_text() == "def policy():\n    return 2\n"
    assert json.loads((session_path / "policy-result.json").read_text()) == 2
    assert len(json.loads((session_path / "policy-validation.json").read_text())) == 2


def test_policy_authoring_exhaustion_is_invalid_and_session_cannot_be_reused(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(runtime, "CodePolicyMcpServer", FakeServer)
    monkeypatch.setattr(runtime, "PiCliRunner", PolicyRunner)
    marker = tmp_path / "exists"
    marker.touch()
    monkeypatch.setattr(runtime, "_pi_paths", lambda: (marker, marker))
    PolicyRunner.sources = ["def policy():\n    return missing"]
    PolicyRunner.prompts = []
    monkeypatch.setattr(
        runtime,
        "validate_policy_source",
        lambda *_args: PolicyExecutionResult(valid=False, error="NameError", duration_seconds=0.1),
    )
    factory = runtime.CodePolicyRuntimeFactory(
        config=CodePolicyAgentConfig(),
        api_key="secret",
        workspace=tmp_path,
    )
    environment = FrozenMemoryEnvironment(
        recording_path="source.db",
        derived_recording_path="derived.db",
        memory_cutoff_timestamp=1.0,
    )

    with factory.open_session(environment) as session:
        outcome = session.author_policy(
            evaluation_protocol="Return an integer from policy().",
            task_input="Question",
            max_rounds=1,
        )
        with pytest.raises(RuntimeError, match="one top-level operation"):
            session.answer(evaluation_protocol="Answer.", task_input="Question")

    assert outcome.status == "invalid"
    assert outcome.result is None


def test_policy_authoring_requires_positive_round_limit(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(runtime, "CodePolicyMcpServer", FakeServer)
    factory = runtime.CodePolicyRuntimeFactory(
        config=CodePolicyAgentConfig(), api_key="secret", workspace=tmp_path
    )
    environment = FrozenMemoryEnvironment(
        recording_path="source.db",
        derived_recording_path="derived.db",
        memory_cutoff_timestamp=1.0,
    )

    with factory.open_session(environment) as session:
        with pytest.raises(ValueError, match="max_rounds must be positive"):
            session.author_policy(
                evaluation_protocol="Protocol",
                task_input="Question",
                max_rounds=0,
            )
