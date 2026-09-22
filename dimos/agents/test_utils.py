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

import json

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from dimos.agents.utils import chat_entries, message_text, summarize_tool_result


def test_message_text_joins_text_blocks() -> None:
    content = [
        {"type": "reasoning", "summary": []},
        {"type": "text", "text": "first"},
        {"type": "text", "text": "second"},
    ]
    assert message_text(content) == "first\nsecond"
    assert message_text("plain") == "plain"


def test_summarize_tool_result_reads_a_skill_result() -> None:
    ok = json.dumps({"success": True, "message": "Found it", "error_code": None})
    failed = json.dumps(
        {"success": False, "message": "No result", "error_code": "EXECUTION_FAILED"}
    )
    assert summarize_tool_result(ok) == "Found it"
    assert summarize_tool_result(failed) == "EXECUTION_FAILED: No result"
    assert summarize_tool_result("Started navigating.") == "Started navigating."


def test_chat_entries_split_a_reply_from_its_tool_calls() -> None:
    reply = AIMessage(
        content=[{"type": "text", "text": "Looking now."}],
        tool_calls=[{"name": "find_in_memory", "args": {"query": "tree"}, "id": "call_1"}],
    )
    assert chat_entries(reply) == [
        {"role": "agent", "text": "Looking now."},
        {
            "role": "tool_call",
            "name": "find_in_memory",
            "args": '{"query":"tree"}',
            "call_id": "call_1",
        },
    ]
    assert chat_entries(HumanMessage(content="Where is the tree?")) == [
        {"role": "human", "text": "Where is the tree?"}
    ]
    failed = json.dumps(
        {"success": False, "message": "timed out", "error_code": "EXECUTION_TIMEOUT"}
    )
    assert chat_entries(ToolMessage(content=failed, tool_call_id="call_1")) == [
        {
            "role": "tool_result",
            "text": "EXECUTION_TIMEOUT: timed out",
            "ok": False,
            "call_id": "call_1",
        }
    ]
