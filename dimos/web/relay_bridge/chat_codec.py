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

"""chat.json.v1: LangChain agent messages -> chat lines (the Chat panel).

Kept out of builtin_codecs on purpose: langchain-core belongs to the [agents]
extra, so the generic bridge and authoring modules must import without it.
The Chat panel imports this module (registering the encoder) only when used.

Wire contract: one frame per message, the payload a JSON array of lines
`{"role": str, "text": str, "ts": float, "tool"?: str}` (ts = seconds since
the epoch at encode time; tool = the tool name on the condensed tool
call/result/progress lines).
"""

import json
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, SystemMessage, ToolMessage

from dimos.web.codecs import web_encoder

# Tool progress rides /agent as HumanMessage("[tool:NAME] text")
# (mcp_client._on_tool_stream_message); same parse as humancli.
_TOOL_MSG_PREFIX = "[tool:"
# A tool result can be megabytes and the cockpit's JSON decoder drops payloads
# above 256 KiB; a chat line is for reading, not for carrying data.
_CHAT_TEXT_MAX_CHARS = 4000


def chat_lines(msg: BaseMessage, ts: float) -> list[dict[str, Any]]:
    """The chat lines of one LangChain message: humancli's rendering as
    data. Known kinds are human, ai, tool and system; any other message type
    is passed through as its role so new kinds degrade visibly, not
    silently. Content blocks flatten to their text (images never cross the
    wire); empty lines are dropped."""
    text = msg.text
    lines: list[dict[str, Any]]
    if isinstance(msg, HumanMessage):
        end = text.find("]")
        if text.startswith(_TOOL_MSG_PREFIX) and end != -1:
            tool = text[len(_TOOL_MSG_PREFIX) : end]
            lines = [{"role": "tool", "tool": tool, "text": text[end + 1 :].lstrip()}]
        else:
            lines = [{"role": "human", "text": text}]
    elif isinstance(msg, AIMessage):
        lines = [{"role": "ai", "text": text}]
        for call in msg.tool_calls:
            args = json.dumps(call["args"], separators=(",", ":"))
            lines.append({"role": "ai", "tool": call["name"], "text": f"{call['name']}({args})"})
    elif isinstance(msg, ToolMessage):
        lines = [{"role": "tool", "tool": msg.name or "tool", "text": text}]
    elif isinstance(msg, SystemMessage):
        lines = [{"role": "system", "text": text}]
    else:
        lines = [{"role": msg.type, "text": text}]
    for line in lines:
        if len(line["text"]) > _CHAT_TEXT_MAX_CHARS:
            line["text"] = line["text"][:_CHAT_TEXT_MAX_CHARS] + "..."
        line["ts"] = ts
    return [line for line in lines if line["text"]]


@web_encoder("chat.json.v1")
def encode_chat(msg: BaseMessage) -> bytes | None:
    """None skips a message with nothing to show (an empty AI turn)."""
    lines = chat_lines(msg, time.time())
    if not lines:
        return None
    return json.dumps(lines, separators=(",", ":")).encode()
