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

"""Exercise Pi's configuration, event handling and shutdown without an external agent."""

from collections.abc import Iterator
from contextlib import nullcontext
import json
import os
from pathlib import Path
import subprocess
from typing import IO
from unittest.mock import MagicMock

from pydantic import JsonValue
import pytest
from pytest_mock import MockerFixture

from dimos.agents.llm_trace import request_path, response_path
from dimos.evals.agents import pi
from dimos.evals.agents.blind import Blind
from dimos.evals.agents.lib.pi_config import Provider, RunPaths, RuntimeConfig
from dimos.evals.agents.lib.pi_to_atif import PiToAtif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.mcp_client_adapter import McpClientAdapter
from dimos.evals.agents.pi import PiAdapter, recording_file
from dimos.evals.cli import load_agent
from dimos.evals.types import (
    FinalMetrics,
    Observation,
    ObservationResult,
    RunningEnvironment,
    StepExtra,
    ToolCall,
)
from dimos.memory.store.sqlite import SqliteStore


def test_cli_overrides_reach_pi_adapter() -> None:
    agent = load_agent(
        "dimos.evals.agents.pi",
        ["model=eval-model", "max_steps=3", 'modules=["rangefinder-skill"]'],
    )

    assert isinstance(agent, PiAdapter)
    assert agent.config.model == "eval-model"
    assert agent.config.max_steps == 3
    assert agent.config.modules == ("rangefinder-skill",)


def test_invalid_tool_policy_rejected_before_startup() -> None:
    for names in [("unknown",), ("bash", "bash"), ("",)]:
        with pytest.raises(ValueError):
            PiAdapter(allowed_tools=names)
    with pytest.raises(ValueError, match="does not support allowed_tools"):
        McpClientAdapter(allowed_tools=())
    with pytest.raises(ValueError, match="has no tools"):
        Blind(allowed_tools=("bash",))
    assert Blind(allowed_tools=()).available_tools(()) == ()


def test_zero_budget_returns_without_starting_pi(tmp_path: Path) -> None:
    agent = PiAdapter(max_steps=0, cli=str(tmp_path / "pi-not-installed"))
    environment = RunningEnvironment(mcp_url="", streams=(), artifacts={})

    trajectory = agent.run("Answer the question.", environment, tmp_path, timeout_s=10)

    assert trajectory.extra.ended_by == "max_steps"
    assert [step.source for step in trajectory.steps] == ["user"]
    assert list(tmp_path.iterdir()) == []


def test_recording_export_contains_only_selected_data(dataset: str, tmp_path: Path) -> None:
    """Pi must receive the case's selected observations, not the full recording."""
    exported = tmp_path / "selected.db"
    with SqliteStore(path=dataset, must_exist=True) as source:
        source.stream("excluded", str).append("not selected", ts=1000.0)
        recording_file((source.streams.odom.limit(2),), exported)

    with SqliteStore(path=str(exported), must_exist=True) as copy:
        assert copy.list_streams() == ["odom"]
        assert [(obs.ts, obs.data.position.x) for obs in copy.streams.odom] == [
            (1000.0, 0.0),
            (1001.0, 1.0),
        ]


@pytest.fixture
def pi_process(mocker: MockerFixture) -> Iterator[tuple[MagicMock, IO[bytes], MagicMock]]:
    """Replace external processes; retain real pipe reads and event conversion."""
    mocker.patch.object(pi, "model_trace_proxy", return_value=nullcontext("http://provider.test"))
    # Native-harness tests cover startup; these pipes exercise event and shutdown failures.
    mocker.patch.object(PiAdapter, "_check_tools_ready")
    spawn = mocker.patch("dimos.evals.agents.pi.subprocess.Popen")
    process = spawn.return_value.__enter__.return_value
    process.returncode = 0
    sweep = mocker.patch.object(pi, "kill_run_processes")
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "rb") as incoming, os.fdopen(write_fd, "wb") as outgoing:
        process.stdout = incoming
        yield process, outgoing, sweep


def test_run_preserves_retried_tool_exchange_and_matching_traces(
    pi_process: tuple[MagicMock, IO[bytes], MagicMock], tmp_path: Path
) -> None:
    process, outgoing, _ = pi_process
    raw = tmp_path / "raw"
    raw.mkdir()
    # All responses predate stdout consumption, including an incomplete retry.
    for seq, body in enumerate(
        [
            "{",
            '{"body": {"error": "retry"}}',
            json.dumps({"body": 'event: response.created\ndata: {"response":{"id":"tool"}}\n'}),
            json.dumps({"body": {"id": "answer"}, "latency_s": 0.5}),
        ]
    ):
        request_path(raw, seq).write_text("{}")
        response_path(raw, seq).write_text(body)
    messages = [
        {"role": "assistant", "stopReason": "error", "errorMessage": "retry"},
        {
            "role": "assistant",
            "responseId": "tool",
            "responseModel": "actual-model",
            "content": [
                {"type": "thinking", "thinking": "inspect"},
                {
                    "type": "toolCall",
                    "id": "call-1",
                    "name": "read",
                    "arguments": {"path": "facts.txt"},
                },
            ],
            "usage": {
                "input": 2,
                "cacheWrite": 3,
                "cacheRead": 5,
                "output": 7,
                "reasoning": 4,
                "cost": {"total": 0.25},
            },
        },
        {
            "role": "assistant",
            "responseId": "answer",
            "content": [{"type": "text", "text": "42"}],
            "usage": {"input": 1, "output": 2, "cost": {"total": 0.5}},
        },
    ]
    events = [{"type": "message_end", "message": message} for message in messages]
    events.insert(
        2,
        {
            "type": "tool_execution_end",
            "toolCallId": "call-1",
            "result": {
                "content": [
                    {"type": "text", "text": "first"},
                    {"type": "image"},
                    {"type": "text", "text": "second"},
                ]
            },
        },
    )
    # The final line deliberately lacks a newline: EOF must not discard it.
    outgoing.write("\n".join(json.dumps(event) for event in events).encode())
    outgoing.close()
    agent = PiAdapter(cli=str(tmp_path / "pi"), max_steps=None)

    result = agent.run(
        "Question", RunningEnvironment(mcp_url="", streams=(), artifacts={}), tmp_path, timeout_s=10
    )

    assert result.extra.ended_by == "answer"
    assert result.final_answer == "42"
    assert result.final_metrics == FinalMetrics(
        total_prompt_tokens=11,
        total_completion_tokens=9,
        total_cached_tokens=5,
        total_cost_usd=0.75,
        total_steps=3,
    )
    _, tool, answer = result.steps
    assert tool.extra == StepExtra(
        request=request_path(raw, 2), response=response_path(raw, 2), reasoning_tokens=4
    )
    assert answer.extra == StepExtra(
        request=request_path(raw, 3), response=response_path(raw, 3), latency_s=0.5
    )
    assert tool.model_name == "actual-model"
    assert tool.reasoning_content == "inspect"
    assert tool.tool_calls == (
        ToolCall(tool_call_id="call-1", function_name="read", arguments={"path": "facts.txt"}),
    )
    assert tool.observation == Observation(
        results=(ObservationResult(source_call_id="call-1", content="first\nsecond"),)
    )
    process.terminate.assert_called_once()


@pytest.mark.parametrize("stop", ["timeout", "exit_error", "budget"])
def test_run_reports_stop_reason_and_cleans_up_pi(
    stop: str, pi_process: tuple[MagicMock, IO[bytes], MagicMock], tmp_path: Path
) -> None:
    process, outgoing, sweep = pi_process
    process.returncode = 17
    outgoing.close()
    # The normal wait (after EOF) may time out; shutdown must still escalate.
    expired = subprocess.TimeoutExpired("pi", 0)
    process.wait.side_effect = [expired, expired, 17] if stop == "timeout" else [17, 17]
    if stop == "budget":
        (tmp_path / "pi-request-limit-reached").touch()
    agent = PiAdapter(cli=str(tmp_path / "pi"), shutdown_timeout_s=0)
    environment = RunningEnvironment(mcp_url="", streams=(), artifacts={})
    result = agent.run("Question", environment, tmp_path, timeout_s=10)
    expected = {"budget": "max_steps", "timeout": "timeout", "exit_error": "error"}
    assert result.extra.ended_by == expected[stop]
    if stop == "exit_error":
        assert "exit status 17" in result.extra.error
    process.terminate.assert_called_once()
    assert process.kill.call_count == int(stop == "timeout")
    sweep.assert_called_once_with(
        str(tmp_path),
        env_var="DIMOS_EVAL_RUN_ID",
        exclude_pids=(process.pid,),
        term_timeout=0,
    )


@pytest.mark.parametrize(
    "provider,key", [("openai", "OPENAI_API_KEY"), ("anthropic", "ANTHROPIC_API_KEY")]
)
def test_model_proxy_keeps_native_provider_and_no_secret(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, provider: Provider, key: str
) -> None:
    monkeypatch.setenv(key, "private-value")
    agent = PiAdapter(provider=provider, model="registered-model", max_output_tokens=4096)
    paths = RunPaths.for_run(tmp_path)
    agent._configure(
        RunningEnvironment(mcp_url="", streams=(), artifacts={}), paths, "http://provider.test"
    )
    serialized = (paths.config / "extensions/runtime.json").read_text()
    assert "private-value" not in serialized
    config = RuntimeConfig.model_validate_json(serialized)
    assert config.provider == provider
    assert config.base_url == "http://provider.test"
    assert config.key_env == key
    assert config.max_output_tokens == 4096
    env = agent._build_process_env(paths)
    assert env[key] == "private-value"
    for name in (
        "HOME",
        "XDG_CONFIG_HOME",
        "XDG_STATE_HOME",
        "XDG_CACHE_HOME",
        "PI_CODING_AGENT_DIR",
        "PI_SKIP_VERSION_CHECK",
        "PI_TELEMETRY",
    ):
        assert env.get(name) == os.environ.get(name)


def test_anthropic_sse_usage_and_trace_matching(tmp_path: Path) -> None:
    raw = tmp_path / "raw"
    raw.mkdir()
    request_path(raw, 0).write_text("{}")
    response_path(raw, 0).write_text(
        json.dumps(
            {
                "body": 'event: message_start\ndata: {"type":"message_start","message":{"id":"msg_1"}}\n'
            }
        )
    )
    events = PiToAtif(raw, TrajectoryBuilder("Question", name="Pi"))
    events.append_event(
        {
            "type": "message_end",
            "message": {
                "role": "assistant",
                "responseId": "msg_1",
                "model": "claude-fable-5-1",
                "content": [{"type": "text", "text": "answer"}],
                "usage": {"input": 10, "cacheRead": 20, "cacheWrite": 30, "output": 4},
            },
        }
    )
    result = events.trajectory.build("answer")
    assert result.final_answer == "answer"
    assert result.final_metrics.total_prompt_tokens == 60
    assert result.final_metrics.total_cached_tokens == 20
    assert result.final_metrics.total_cost_usd is None


def test_pi_retains_its_stock_prompt(tmp_path: Path) -> None:
    command = PiAdapter()._build_pi_command(
        "Question", "Shared context", RunPaths.for_run(tmp_path)
    )
    assert "--append-system-prompt" in command
    assert "--system-prompt" not in command


@pytest.mark.parametrize(
    "message",
    [
        {"role": "assistant", "content": [{"type": "toolCall", "name": "bash", "arguments": {}}]},
        {"role": "assistant", "usage": {"input": "not-a-token-count"}},
        {"role": "assistant", "content": [{"type": "text", "text": "untraceable"}]},
    ],
)
def test_malformed_assistant_events_fail_without_inventing_a_step(
    message: dict[str, JsonValue], tmp_path: Path
) -> None:
    events = PiToAtif(tmp_path, TrajectoryBuilder("Question", name="test"))
    with pytest.raises((ValueError, RuntimeError)):
        events.append_event({"type": "message_end", "message": message})
    assert len(events.trajectory.build("error").steps) == 1
