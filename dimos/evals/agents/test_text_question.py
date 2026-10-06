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

from collections.abc import Iterator
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any

from langchain_core.language_models.fake_chat_models import FakeListChatModel
import pytest
from pytest_mock import MockerFixture

from dimos.evals.agents.lib import single_call
from dimos.evals.agents.pi import PiAdapter
from dimos.evals.agents.text_question import TextQuestion
from dimos.evals.environments.prompt import Prompt
from dimos.evals.runner import EvalRunner
from dimos.evals.types import EvalCase, Outcome, RunningEnvironment


@dataclass
class ProviderState:
    requests: list[dict[str, Any]] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)
    release: threading.Event = field(default_factory=threading.Event)
    status: int = 200


@pytest.fixture
def provider(monkeypatch: pytest.MonkeyPatch) -> Iterator[ProviderState]:
    state = ProviderState()
    state.release.set()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            return None

        def do_POST(self) -> None:
            request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            with state.lock:
                state.requests.append(request)
                status = state.status
            state.release.wait(timeout=5)
            response: dict[str, Any]
            if status != 200:
                response = {
                    "error": {"message": "test provider unavailable", "type": "server_error"}
                }
            elif self.path == "/v1/responses":
                response = {
                    "id": "resp_test",
                    "object": "response",
                    "created_at": 1700000000,
                    "status": "completed",
                    "error": None,
                    "service_tier": "default",
                    "model": "gpt-5-test-resolved",
                    "output": [
                        {
                            "id": "msg_test",
                            "type": "message",
                            "status": "completed",
                            "role": "assistant",
                            "content": [{"type": "output_text", "text": "2", "annotations": []}],
                        }
                    ],
                    "usage": {"input_tokens": 11, "output_tokens": 1, "total_tokens": 12},
                }
            else:
                response = {
                    "id": "chatcmpl_test",
                    "object": "chat.completion",
                    "created": 1700000000,
                    "model": "gpt-4o-test-resolved",
                    "choices": [
                        {
                            "index": 0,
                            "message": {"role": "assistant", "content": "2"},
                            "finish_reason": "stop",
                        }
                    ],
                    "usage": {"prompt_tokens": 11, "completion_tokens": 1, "total_tokens": 12},
                }
            payload = json.dumps(response).encode()
            try:
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                # A timed-out SDK request closes its socket before the fixture releases us.
                return None

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = threading.Thread(target=server.serve_forever)
    worker.start()
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    monkeypatch.setenv("OPENAI_BASE_URL", f"http://127.0.0.1:{server.server_port}/v1")
    monkeypatch.setenv("NO_PROXY", "127.0.0.1")
    try:
        yield state
    finally:
        state.release.set()
        server.shutdown()
        server.server_close()
        worker.join(timeout=5)
        assert not worker.is_alive()


def completed_answer(outcome: Outcome) -> float:
    if outcome.trajectory.extra.ended_by != "answer":
        raise TimeoutError("no completed model answer")
    return float(outcome.trajectory.final_answer == "2")


@pytest.mark.parametrize(
    ("model", "payload_key", "token_key", "content_type", "resolved"),
    [
        ("openai:gpt-4o", "messages", "max_completion_tokens", "text", "gpt-4o-test-resolved"),
        ("gpt-5-test", "input", "max_output_tokens", "input_text", "gpt-5-test-resolved"),
    ],
)
def test_text_question_uses_original_prompt_and_provider_limits(
    provider: ProviderState,
    tmp_path: Path,
    mocker: MockerFixture,
    model: str,
    payload_key: str,
    token_key: str,
    content_type: str,
    resolved: str,
) -> None:
    client = mocker.spy(single_call, "tracing_http_client")
    prompt = "Frame 1: red → blue.\nFrame 2: blue → green.\nChoose 1 or 2."
    case = EvalCase(id="text", inputs=prompt, environment=Prompt(), grade=completed_answer)
    runner = EvalRunner(out_dir=tmp_path)
    result = runner.run([case], TextQuestion(model=model, max_output_tokens=37))[0]

    assert result.score == 1.0 and not result.error
    with provider.lock:
        assert len(provider.requests) == 1
        payload = provider.requests[0]
    assert payload[payload_key] == [
        {"role": "user", "content": [{"type": content_type, "text": prompt}]}
    ]
    assert payload[token_key] == 37
    assert "tools" not in payload
    assert client.spy_return.is_closed
    saved = json.loads(Path(result.trajectory).read_text())
    assert saved["agent"]["model_name"] == resolved
    assert saved["agent"]["tool_definitions"] == []
    assert result.prompt_tokens == 11 and result.completion_tokens == 1
    request = json.loads((runner.run_dir / "text/raw/000-request.json").read_text())
    assert request["body"] == payload
    assert "authorization" not in request["headers"]


def test_provider_timeout_retains_trajectory_and_closes_owned_client(
    provider: ProviderState, tmp_path: Path, mocker: MockerFixture
) -> None:
    provider.release.clear()
    client = mocker.spy(single_call, "tracing_http_client")
    runner = EvalRunner(out_dir=tmp_path)
    case = EvalCase(
        id="timeout", inputs="answer", environment=Prompt(), grade=completed_answer, timeout_s=0.05
    )
    result = runner.run([case], TextQuestion(model="openai:gpt-4o"))[0]
    provider.release.set()

    assert result.ended_by == "timeout" and "no completed model answer" in result.error
    assert result.final_answer == "" and not result.passed
    saved = json.loads(Path(result.trajectory).read_text())
    assert "APITimeoutError" in saved["extra"]["error"]
    assert [step["source"] for step in saved["steps"]] == ["user"]
    with provider.lock:
        assert len(provider.requests) == 1
    assert result.request_attempts == 1
    assert client.spy_return.is_closed


def test_provider_failure_does_not_retry_or_discard_trajectory(
    provider: ProviderState, tmp_path: Path
) -> None:
    with provider.lock:
        provider.status = 503
    runner = EvalRunner(out_dir=tmp_path)
    case = EvalCase(id="failure", inputs="answer", environment=Prompt(), grade=completed_answer)
    result = runner.run([case], TextQuestion(model="openai:gpt-4o"))[0]

    assert result.ended_by == "error" and "test provider unavailable" in result.error
    assert not result.passed and result.final_answer == ""
    assert Path(result.trajectory).is_file()
    with provider.lock:
        assert len(provider.requests) == 1
    response = json.loads((runner.run_dir / "failure/raw/000-response.json").read_text())
    assert response["status"] == 503


@pytest.mark.parametrize("system_prompt", [None, ""])
def test_absent_system_prompt_and_injected_model_are_preserved(
    tmp_path: Path, mocker: MockerFixture, system_prompt: str | None
) -> None:
    chat = FakeListChatModel(responses=["2"])
    generate = mocker.spy(FakeListChatModel, "generate")
    agent = TextQuestion(chat_model=chat, system_prompt=system_prompt)
    trajectory = agent.run("original", Prompt().start(()), tmp_path, timeout_s=1)

    assert trajectory.final_answer == "2"
    assert [message.type for message in generate.call_args.args[1][0]] == ["human"]
    assert chat.invoke("still usable").content == "2"


def test_prompt_environment_rejects_modules() -> None:
    environment = Prompt()
    with pytest.raises(ValueError, match="cannot launch"):
        environment.preflight(TextQuestion(modules=("mcp-server",)))
    with pytest.raises(ValueError, match="cannot launch"):
        environment.start(("mcp-server",))


def test_prompt_environment_rejects_native_agent_tools() -> None:
    with pytest.raises(ValueError, match="without tools"):
        Prompt().preflight(PiAdapter())


def test_text_question_rejects_robot_context(tmp_path: Path) -> None:
    agent = TextQuestion(chat_model=FakeListChatModel(responses=["2"]))
    env = RunningEnvironment(mcp_url="http://localhost:9990/mcp", streams=(), artifacts={})
    with pytest.raises(ValueError, match="prompt-only"):
        agent.run("question", env, tmp_path, timeout_s=1)
