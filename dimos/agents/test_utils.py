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

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import pytest

from dimos.agents import utils


@pytest.mark.parametrize(
    "message",
    [
        HumanMessage(content="Hello"),
        ToolMessage(content=[{"type": "text", "text": "Hello"}], tool_call_id="call"),
        AIMessage(
            content=[
                {"type": "reasoning", "encrypted_content": "encrypted-secret"},
                {"type": "text", "text": "Hello"},
                {"type": "image_url", "image_url": {"url": "data:image/png;base64,SECRET"}},
            ],
            tool_calls=[{"name": "speak", "args": {"text": "Hello"}, "id": "call"}],
        ),
    ],
)
def test_message_output_shows_text_and_tool_calls_without_raw_blocks(message, mocker, capsys):
    log = mocker.patch.object(utils.logger, "info")

    utils.pretty_print_langchain_message(message)

    fields = log.call_args.kwargs
    assert fields["content"] == "Hello"
    assert fields["msg_type"] == message.type
    output = capsys.readouterr().out
    assert "Hello" in output
    assert "encrypted-secret" not in output
    assert "SECRET" not in output
    assert "'type':" not in output


def test_message_logging_keeps_tool_calls_when_text_is_empty(mocker):
    log = mocker.patch.object(utils.logger, "info")
    message = AIMessage(
        content=[{"type": "reasoning", "encrypted_content": "encrypted-secret"}],
        tool_calls=[{"name": "speak", "args": {"text": "Hello"}, "id": "call"}],
    )

    utils.pretty_print_langchain_message(message)

    log.assert_called_once_with(
        "Agent message", msg_type="ai", tool_calls=[{"name": "speak", "args": {"text": "Hello"}}]
    )
