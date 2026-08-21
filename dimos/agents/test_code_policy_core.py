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

import asyncio
import json
from pathlib import Path

import cloudpickle
import pytest
from pytest_mock import MockerFixture

import dimos.agents.code_policy_core as code_policy_core
from dimos.agents.code_policy_core import (
    CodePolicyImage,
    CodePolicySession,
    CodePolicySessionConfig,
    LiveDimosEnvironment,
    SubmissionEnvironment,
    _bootstrap_source,
    _kernel_environment,
    validate_policy_callable,
)
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.evaluation.protocol import (
    PolicyArtifact,
    PolicyCandidate,
    PolicyRequestError,
    PolicySnapshot,
    TrialEvidence,
    TrialOutcome,
    TrialRun,
)
from dimos.memory2.store.sqlite import SqliteStore
from dimos.porcelain.dimos import Dimos


def policy(app: Dimos) -> None:
    del app


def test_validate_policy_callable_accepts_canonical_signature() -> None:
    validate_policy_callable(policy)


@pytest.mark.parametrize(
    ("candidate", "message"),
    [
        (lambda: None, "named 'policy'"),
        (lambda app: None, "named 'policy'"),
    ],
)
def test_validate_policy_callable_rejects_noncanonical_functions(candidate, message: str) -> None:
    with pytest.raises(TypeError, match=message):
        validate_policy_callable(candidate)


def test_exploration_repl_submits_candidate_and_freezes_it(tmp_path: Path) -> None:
    artifacts = tmp_path / "trial"
    artifacts.mkdir()
    log_path = artifacts / "main.jsonl"
    log_path.write_text(json.dumps({"module": "Planner", "event": "failed"}) + "\n")
    memory_path = artifacts / "recording.db"
    with SqliteStore(path=str(memory_path)) as memory:
        memory.stream("events", str).append("attempted")
    submitted: list[object] = []

    policy_artifact = PolicyArtifact(
        source="def policy(app: Dimos) -> None: ...\n",
        serialized=cloudpickle.dumps(policy),
        sha256="digest",
    )
    handled_candidates: list[PolicyCandidate] = []

    def handle(source: str, serialized: bytes) -> PolicyCandidate:
        submitted.append(cloudpickle.loads(serialized))
        assert "def policy" in source
        trial = TrialRun(
            run_id="debug-1",
            outcome=TrialOutcome(
                success=False,
                reward=0.0,
                status="completed",
                error=None,
                duration_seconds=1.0,
            ),
            artifacts=artifacts,
            log_path=log_path,
            memory_path=memory_path,
            policy_output="planner diagnostic",
        )
        candidate = PolicyCandidate(
            id="candidate-0001-digest",
            policy=PolicySnapshot(
                source=policy_artifact.source,
                sha256=policy_artifact.sha256,
            ),
            evidence=TrialEvidence("candidate-0001-digest", trial, 4),
        )
        handled_candidates.append(candidate)
        return candidate

    frozen: list[str] = []

    def freeze(candidate_id: str) -> PolicyCandidate:
        frozen.append(candidate_id)
        return handled_candidates[0]

    with CodePolicyMcpServer(handle, freeze_handler=freeze) as server:
        assert server.session is not None
        result = server.session.python_exec(
            "def policy(app: Dimos) -> None:\n"
            "    app.list_modules()\n\n"
            "candidate = submit_policy(policy)\n"
            "freeze_policy(candidate)\n"
            "(candidate.id, candidate.trial.run_id, candidate.evidence.policy_output)"
        )

    assert "('candidate-0001-digest', 'debug-1', 'planner diagnostic')" in result.text
    assert len(submitted) == 1
    assert frozen == ["candidate-0001-digest"]


def test_submission_endpoint_maps_contract_errors_to_bad_request(mocker: MockerFixture) -> None:
    def reject(_source: str, _serialized: bytes) -> PolicyCandidate:
        raise PolicyRequestError("submission budget exhausted")

    server = CodePolicyMcpServer(reject, freeze_handler=lambda _candidate_id: None)  # type: ignore[arg-type]
    request = mocker.Mock()
    request.headers = {"authorization": f"Bearer {server.submission_token}"}
    request.json = mocker.AsyncMock(
        return_value={"source": "def policy(app): ...", "serialized": "eA=="}
    )

    response = asyncio.run(server._submit_policy(request))

    assert response.status_code == 400
    assert json.loads(response.body) == {"error": "submission budget exhausted"}


def test_submission_endpoint_does_not_mask_server_bugs(mocker: MockerFixture) -> None:
    def crash(_source: str, _serialized: bytes) -> PolicyCandidate:
        raise AssertionError("server bug")

    server = CodePolicyMcpServer(crash, freeze_handler=lambda _candidate_id: None)  # type: ignore[arg-type]
    request = mocker.Mock()
    request.headers = {"authorization": f"Bearer {server.submission_token}"}
    request.json = mocker.AsyncMock(
        return_value={"source": "def policy(app): ...", "serialized": "eA=="}
    )

    with pytest.raises(AssertionError, match="server bug"):
        asyncio.run(server._submit_policy(request))


def test_live_repl_bootstraps_public_runtime_without_credentials(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setenv("ORDINARY_SETTING", "retained")
    environment = LiveDimosEnvironment(recording_path="/attempt/recording.db")
    config = CodePolicySessionConfig(environment=environment)
    mocker.patch.object(code_policy_core.global_config, "transport", "zenoh")

    source = _bootstrap_source(environment)
    kernel_environment = _kernel_environment(config)

    assert "app = Dimos.connect(" in source
    assert "memory=SqliteStore" in source
    assert "\nmemory =" not in source
    assert "app.memory.start()" in source
    assert "submit_policy" not in source
    assert "OPENAI_API_KEY" not in kernel_environment
    assert kernel_environment["ORDINARY_SETTING"] == "retained"
    assert kernel_environment["DIMOS_TRANSPORT"] == "zenoh"


def test_bounded_output_retains_displayed_image() -> None:
    output = code_policy_core._BoundedOutput(1_000)

    output(
        {
            "header": {"msg_type": "display_data"},
            "content": {
                "data": {
                    "text/plain": "<PIL.Image.Image>",
                    "image/png": "cG5n",
                }
            },
        }
    )

    assert output.text() == "<PIL.Image.Image>"
    assert output.images == [CodePolicyImage(data="cG5n", mime_type="image/png")]


def test_python_exec_returns_displayed_image() -> None:
    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=SubmissionEnvironment(
                submission_url="http://127.0.0.1:1",
                freeze_url="http://127.0.0.1:1",
                submission_token="unused",
            )
        )
    )
    session.start()
    try:
        result = session.python_exec(
            "from IPython.display import display\n"
            "from PIL import Image\n"
            "display(Image.new('RGB', (2, 2), 'red'))"
        )
    finally:
        session.stop()

    assert result.text.endswith("<PIL.Image.Image image mode=RGB size=2x2>")
    assert len(result.images) == 1
    assert result.images[0].mime_type in {"image/png", "image/jpeg"}
    assert result.images[0].data


def test_python_exec_rejects_timeout_longer_than_transport_request() -> None:
    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=SubmissionEnvironment(
                submission_url="http://127.0.0.1:1",
                freeze_url="http://127.0.0.1:1",
                submission_token="unused",
            )
        )
    )
    session.start()
    try:
        result = session.python_exec("1 + 1", timeout_s=300.0)
    finally:
        session.stop()

    assert result.text == "timeout_s must be in (0, 240]"
