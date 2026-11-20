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

import subprocess
import sys

from langchain_core.messages import AIMessage, ToolMessage
import pytest

from dimos.cli.agentspy.agentspy import format_message_content


@pytest.mark.parametrize(
    ("message", "expected"),
    [
        (
            AIMessage(
                content=[
                    {"type": "reasoning", "encrypted_content": "encrypted-secret"},
                    {"type": "text", "text": "Hello"},
                ]
            ),
            "Hello",
        ),
        (
            AIMessage(
                content=[{"type": "text", "text": "Hello"}],
                tool_calls=[{"name": "wave", "args": {}, "id": "call"}],
            ),
            "Hello\n[Tool Calls: wave({})]",
        ),
        (
            ToolMessage(
                content=[{"type": "text", "text": "Done"}], name="wave", tool_call_id="call"
            ),
            "wave() -> Done",
        ),
    ],
)
def test_displays_text_and_tool_calls(message, expected):
    assert format_message_content(message) == expected


def test_help_exits_without_tui():
    result = subprocess.run(
        [sys.executable, "-m", "dimos.cli.agentspy.agentspy", "--help"],
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout
    assert "--transport" in result.stdout
