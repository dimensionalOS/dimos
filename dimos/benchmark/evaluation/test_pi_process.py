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

from io import StringIO
import json
from pathlib import Path
import subprocess
import threading

import pytest

from dimos.benchmark.evaluation.pi_process import PiCliRunner, PiRunError, parse_pi_events


def test_parse_stock_pi_events_uses_final_message_and_counts_tools() -> None:
    events = [
        {"type": "tool_execution_start", "toolName": "python_exec"},
        {
            "type": "message_end",
            "message": {
                "role": "assistant",
                "content": [{"type": "text", "text": "Reasoning\nANSWER: 8"}],
                "stopReason": "stop",
            },
        },
    ]
    text, count, error = parse_pi_events("\n".join(json.dumps(item) for item in events))
    assert text == "Reasoning\nANSWER: 8"
    assert count == 1
    assert error is None


def test_stock_cli_streams_assistant_tools_and_stderr_while_running(mocker, tmp_path: Path) -> None:
    cli = tmp_path / "cli.js"
    extension = tmp_path / "extension.js"
    cli.touch()
    extension.touch()
    process = mocker.Mock(returncode=0)
    process.stdout = StringIO(
        "\n".join(
            json.dumps(event)
            for event in (
                {
                    "type": "message_update",
                    "assistantMessageEvent": {"type": "text_delta", "delta": "Checking"},
                },
                {
                    "type": "tool_execution_start",
                    "toolCallId": "call-1",
                    "toolName": "python_exec",
                    "args": {"code": "memory.list_streams()"},
                },
                {
                    "type": "tool_execution_end",
                    "toolCallId": "call-1",
                    "toolName": "python_exec",
                    "result": {"content": [{"type": "text", "text": "['lidar']"}]},
                    "isError": False,
                },
                {
                    "type": "message_end",
                    "message": {
                        "role": "assistant",
                        "content": [{"type": "text", "text": "ANSWER: 2"}],
                        "stopReason": "stop",
                    },
                },
            )
        )
    )
    process.stderr = StringIO("provider secret connected\n")
    mocker.patch("dimos.benchmark.evaluation.pi_process.subprocess.Popen", return_value=process)
    progress = []
    assistant_seen = threading.Event()

    def observe(event) -> None:
        progress.append(event)
        if event.kind == "assistant_text":
            assistant_seen.set()

    def wait_for_process(*, timeout) -> int:
        assert timeout == 10
        assert assistant_seen.wait(timeout=1)
        return 0

    process.wait.side_effect = wait_for_process
    runner = PiCliRunner(
        cli=cli,
        extension=extension,
        model="gpt-5.6-luna",
        thinking_level="medium",
        timeout_s=10,
        progress=observe,
    )

    result = runner.run(
        prompt="Count",
        system_prompt="Use memory",
        mcp_url="http://127.0.0.1:1234/mcp",
        api_key="secret",
        run_dir=tmp_path,
    )

    assert [(event.kind, getattr(event, "delta", None)) for event in progress[:1]] == [
        ("assistant_text", "Checking")
    ]
    structured = [event for event in progress if event.kind != "status"]
    assert [event.kind for event in structured] == [
        "assistant_text",
        "tool_start",
        "tool_end",
        "final_response",
    ]
    assert structured[1].code == "memory.list_streams()"
    assert structured[2].result == "['lidar']"
    assert [event.message for event in progress if event.kind == "status"] == [
        "provider [REDACTED] connected"
    ]
    assert result.stderr == "provider [REDACTED] connected\n"
    assert result.final_text == "ANSWER: 2"


def test_stock_cli_receives_only_api_key_and_evaluator_binding(mocker, tmp_path: Path) -> None:
    cli = tmp_path / "cli.js"
    extension = tmp_path / "extension.js"
    cli.touch()
    extension.touch()
    process = mocker.Mock(returncode=0)
    process.stdout = StringIO(
        json.dumps(
            {
                "type": "message_end",
                "message": {
                    "role": "assistant",
                    "content": [{"type": "text", "text": "ANSWER: 2"}],
                    "stopReason": "stop",
                },
            }
        )
    )
    process.stderr = StringIO()
    process.wait.return_value = 0
    popen = mocker.patch(
        "dimos.benchmark.evaluation.pi_process.subprocess.Popen", return_value=process
    )
    runner = PiCliRunner(
        cli=cli,
        extension=extension,
        model="gpt-5.6-luna",
        thinking_level="medium",
        timeout_s=10,
    )
    result = runner.run(
        prompt="Count",
        system_prompt="Use memory",
        mcp_url="http://127.0.0.1:1234/mcp",
        api_key="secret",
        run_dir=tmp_path,
    )
    command = popen.call_args.args[0]
    env = popen.call_args.kwargs["env"]
    assert command[command.index("--mode") + 1] == "json"
    assert "--no-builtin-tools" in command
    assert command[command.index("--tools") + 1] == "python_exec"
    assert env["OPENAI_API_KEY"] == "secret"
    assert env["DIMOS_CODE_POLICY_MCP_URL"].endswith("/mcp")
    assert "secret" not in command
    assert result.final_text == "ANSWER: 2"


def test_stock_cli_timeout_terminates_the_child(mocker, tmp_path: Path) -> None:
    cli = tmp_path / "cli.js"
    extension = tmp_path / "extension.js"
    cli.touch()
    extension.touch()
    process = mocker.Mock()
    process.stdout = StringIO()
    process.stderr = StringIO("stopped")
    process.wait.side_effect = [
        subprocess.TimeoutExpired("pi", 0.01),
        0,
    ]
    mocker.patch("dimos.benchmark.evaluation.pi_process.subprocess.Popen", return_value=process)
    runner = PiCliRunner(
        cli=cli,
        extension=extension,
        model="gpt-5.6-luna",
        thinking_level="medium",
        timeout_s=0.01,
    )

    with pytest.raises(PiRunError, match="timed out"):
        runner.run(
            prompt="Count",
            system_prompt="Use memory",
            mcp_url="http://127.0.0.1:1234/mcp",
            api_key="secret",
            run_dir=tmp_path,
        )

    process.terminate.assert_called_once_with()
    process.kill.assert_not_called()
