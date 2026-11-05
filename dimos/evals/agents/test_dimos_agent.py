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

from contextlib import nullcontext
import json
import os
from pathlib import Path
import subprocess

import pytest

from dimos.agents.llm_trace import request_path, response_path
from dimos.evals.agents import pi
from dimos.evals.agents.dimos_agent import DimosAgentAdapter
from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.environments.dataset import Dataset
from dimos.evals.types import RunningEnvironment, ToolCall


@pytest.mark.parametrize(
    "option,value",
    [
        ("allowed_tools", ("bash",)),
        ("no_dimos", True),
        ("skills", ("robot.md",)),
        ("excluded_keywords", ("dimos",)),
        ("max_tool_seconds", 1),
    ],
)
def test_unsupported_tool_restrictions_are_rejected(option, value):
    with pytest.raises(ValueError, match=f"does not support {option}"):
        DimosAgentAdapter(**{option: value})


@pytest.mark.parametrize("returncode", [0, 1])
def test_preflight_checks_the_agent_installation(mocker, monkeypatch, returncode):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    mocker.patch("dimos.evals.agents.dimos_agent.shutil.which", return_value="/test/bin/dimos")
    version = mocker.patch(
        "dimos.evals.agents.dimos_agent.subprocess.run",
        return_value=subprocess.CompletedProcess([], returncode, stderr="agent build failed"),
    )
    agent = DimosAgentAdapter()
    expected = (
        pytest.raises(RuntimeError, match="agent build failed") if returncode else nullcontext()
    )

    with expected:
        agent.preflight(Dataset("unused"))

    version.assert_called_once_with(
        ["/test/bin/dimos", "agent", "version"], capture_output=True, text=True
    )


@pytest.mark.parametrize("outcome", ["done", "unanswered"])
def test_run_records_tool_results_and_reports_unanswered_cases(
    mocker, monkeypatch, tmp_path, outcome
):
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    mocker.patch.object(pi, "model_trace_proxy", return_value=nullcontext("http://trace.test"))
    mocker.patch.object(pi, "kill_run_processes")
    listing = mocker.patch.object(pi, "tool_listing")
    mocker.patch("dimos.evals.agents.dimos_agent.shutil.which", return_value="/test/bin/dimos")
    spawn = mocker.patch.object(pi.subprocess, "Popen")
    process = spawn.return_value.__enter__.return_value
    process.returncode = 0
    assistant = {
        "role": "assistant",
        "responseId": "tool-response",
        "content": [
            {"type": "toolCall", "id": "read-1", "name": "read", "arguments": {"path": "facts.txt"}}
        ],
    }
    records = [
        {"item": {"kind": "user"}},
        {"item": {"kind": "assistant", "message": assistant}},
        {
            "item": {
                "kind": "tool_result",
                "result": {"toolCallId": "read-1", "content": [{"type": "text", "text": "42"}]},
            }
        },
        {
            "item": {
                "kind": "assistant",
                "message": {
                    "role": "assistant",
                    "responseId": "answer-response",
                    "content": [{"type": "text", "text": "42"}],
                },
            }
        },
        {"outcome": {"status": outcome, "reason": "model unavailable"}},
    ]
    agent = DimosAgentAdapter(model="test-model", thinking="low", max_output_tokens=128)
    env = RunningEnvironment(mcp_url="http://robot.test:4321/mcp", streams=(), artifacts={})
    raw = tmp_path / "raw"
    raw.mkdir()
    for seq, response_id in enumerate(("tool-response", "answer-response")):
        request_path(raw, seq).write_text("{}")
        response_path(raw, seq).write_text(json.dumps({"body": {"id": response_id}}))
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "rb") as incoming, os.fdopen(write_fd, "wb") as outgoing:
        process.stdout = incoming
        outgoing.write("\n".join(json.dumps(record) for record in records).encode())
        outgoing.close()

        result = agent.run("Read facts.txt", env, tmp_path, timeout_s=5)

    assert result.final_answer == "42", result.extra
    assert result.extra.ended_by == ("answer" if outcome == "done" else "error")
    assert result.extra.error == (
        "" if outcome == "done" else "dimos agent run ended unanswered: model unavailable"
    )
    tool_step = result.steps[1]
    assert tool_step.tool_calls == (
        ToolCall(tool_call_id="read-1", function_name="read", arguments={"path": "facts.txt"}),
    )
    assert tool_step.observation.results[0].content == "42"
    listing.assert_not_called()

    paths = RunPaths.for_run(tmp_path)
    command = spawn.call_args.args[0]
    assert command == [
        "/test/bin/dimos",
        "agent",
        "run",
        "--json",
        "--cwd",
        str(tmp_path),
        (tmp_path / "system-prompt.txt").read_text() + "\n\nRead facts.txt",
    ]
    process_env = spawn.call_args.kwargs["env"]
    assert {
        name: process_env[name]
        for name in ("MCPSERVER__HOST", "MCPSERVER__PORT", "DIMOS_EVAL_API_KEY")
    } == {
        "MCPSERVER__HOST": "robot.test",
        "MCPSERVER__PORT": "4321",
        "DIMOS_EVAL_API_KEY": "test-key",
    }
    assert process_env["XDG_CONFIG_HOME"] == str(paths.config)
    assert process_env["XDG_STATE_HOME"] == str(tmp_path / "agent-state")
    assert process_env["XDG_CACHE_HOME"] == str(paths.cache)
    config = paths.config / "dimos" / "agent"
    assert json.loads((config / "config.json").read_text()) == {
        "model": {"provider": "openai", "modelId": "test-model"},
        "thinkingLevel": "low",
    }
    assert json.loads((config / "models.json").read_text()) == {
        "providers": {
            "openai": {
                "baseUrl": "http://trace.test",
                "apiKey": "$DIMOS_EVAL_API_KEY",
                "modelOverrides": {"test-model": {"maxTokens": 128}},
            }
        },
    }
    process.terminate.assert_called_once()
    assert not Path(process_env["XDG_RUNTIME_DIR"]).exists()
