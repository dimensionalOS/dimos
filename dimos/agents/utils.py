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
from typing import Any

from langchain_core.messages.base import BaseMessage

CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
MAGENTA = "\033[35m"
BLUE = "\033[34m"
GRAY = "\033[90m"
RESET = "\033[0m"
BOLD = "\033[1m"

TYPE_WIDTH = 12


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

    content = d.get("content", "")
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
        content_str = repr(_try_to_remove_url_data(content))
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
