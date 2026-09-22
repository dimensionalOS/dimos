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

from datetime import datetime
import json
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
MAGENTA = "\033[35m"
BLUE = "\033[34m"
GRAY = "\033[90m"
RESET = "\033[0m"
BOLD = "\033[1m"

TYPE_WIDTH = 12


def message_text(content: object) -> str:
    """The readable text of a message whose content may be a list of typed blocks.

    A reply can arrive as CONTENT BLOCKS rather than a string -- gpt-5.6-luna sends
    `[{'type': 'text', ...}]` beside its reasoning and function-call blocks -- and reading
    only the string case silently discards every answer it sends.
    """
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        texts = [
            str(block["text"])
            for block in content
            if isinstance(block, dict) and block.get("type") == "text" and block.get("text")
        ]
        return "\n".join(texts).strip()
    return str(content) if content else ""


def summarize_tool_result(content: object) -> str:
    """A skill's result as its message line; anything else as it came."""
    if not isinstance(content, str):
        return str(content)
    try:
        result = json.loads(content)
    except ValueError:
        return content
    if not isinstance(result, dict) or "message" not in result:
        return content
    message = str(result["message"])
    if result.get("success") is False:
        return f"{result.get('error_code') or 'failed'}: {message}"
    return message


def _tool_result_ok(content: object) -> bool:
    if not isinstance(content, str):
        return True
    try:
        result = json.loads(content)
    except ValueError:
        return True
    return not (isinstance(result, dict) and result.get("success") is False)


ChatEntry = dict[str, Any]


def chat_entries(msg: BaseMessage) -> list[ChatEntry]:
    """A message as the rows a chat view shows: text by role, then one row per tool call."""
    entries: list[ChatEntry] = []
    if isinstance(msg, HumanMessage):
        text = message_text(msg.content)
        if text:
            entries.append({"role": "human", "text": text})
    elif isinstance(msg, SystemMessage):
        text = message_text(msg.content)
        if text:
            entries.append({"role": "system", "text": text})
    elif isinstance(msg, ToolMessage):
        entries.append(
            {
                "role": "tool_result",
                "text": summarize_tool_result(msg.content),
                "ok": _tool_result_ok(msg.content),
                "call_id": msg.tool_call_id,
            }
        )
    elif isinstance(msg, AIMessage):
        text = message_text(msg.content)
        if text:
            entries.append({"role": "agent", "text": text})
        for call in msg.tool_calls:
            entries.append(
                {
                    "role": "tool_call",
                    "name": call.get("name") or "unknown",
                    "args": json.dumps(call.get("args") or {}, separators=(",", ":")),
                    "call_id": call.get("id"),
                }
            )
    return entries


def pretty_print_langchain_message(msg: BaseMessage) -> None:
    d = msg.__dict__
    msg_type = d.get("type", "unknown")

    type_colors = {
        "human": CYAN,
        "ai": GREEN,
        "tool": YELLOW,
        "system": MAGENTA,
    }
    type_color = type_colors.get(msg_type, RESET)

    print(f"{GRAY}{'-' * 60}{RESET}")

    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    time_str = f"{GRAY}{timestamp}{RESET}  "
    type_str = f"{type_color}{msg_type:<{TYPE_WIDTH}}{RESET}"

    content = _try_to_remove_url_data(d.get("content", ""))
    tool_calls = d.get("tool_calls", [])

    # 12 chars for timestamp + 1 space + TYPE_WIDTH + 1 space
    indent = " " * (12 + 1 + TYPE_WIDTH + 1)
    first_line = True

    def print_line(text: str) -> None:
        nonlocal first_line
        if first_line:
            print(f"{time_str} {type_str} {text}")
            first_line = False
        else:
            print(f"{indent}{text}")

    if content:
        content_str = repr(content)
        if len(content_str) > 2000:
            content_str = content_str[:5000] + "... [truncated]"
        print_line(f"{BOLD}{type_color}{content_str}{RESET}")

    if tool_calls:
        print_line(f"{MAGENTA}tool_calls:{RESET}")
        for tc in tool_calls:
            name = tc.get("name")
            args = tc.get("args")
            print_line(f"  - {BLUE}{name}{RESET}({CYAN}{args}{RESET})")

    if first_line:
        print(f"{time_str} {type_str}")

    # Also log to structlog so agent messages appear in per-run JSONL logs.
    _log_message(msg_type, content, tool_calls)


def _log_message(msg_type: str, content: object, tool_calls: list[dict[str, Any]]) -> None:
    """Write agent message to structlog (per-run JSONL)."""
    kw: dict[str, Any] = {"msg_type": msg_type}
    if content:
        kw["content"] = str(content)[:500]
    if tool_calls:
        kw["tool_calls"] = [{"name": tc.get("name"), "args": tc.get("args")} for tc in tool_calls]
    logger.info("Agent message", **kw)


def _try_to_remove_url_data(content: Any) -> Any:
    if not isinstance(content, list):
        return content

    ret = []

    for item in content:
        if isinstance(item, dict) and item.get("type") == "image_url":
            ret.append({**item, "image_url": "<removed>"})
        else:
            ret.append(item)

    return ret
