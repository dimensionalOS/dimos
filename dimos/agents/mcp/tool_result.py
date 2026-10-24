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

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

JSON: TypeAlias = "str | int | float | bool | None | list[JSON] | dict[str, JSON]"


@dataclass(frozen=True)
class ToolResult:
    """A skill result with a structured half, served by McpServer as ``structuredContent``."""

    text: str
    """What the model reads: the tool call's text content."""

    structured: dict[str, JSON]
    """The same result as JSON, for clients that parse it instead of reading it."""

    is_error: bool = False
    """The call ran and failed, such as a module that did not start; McpServer sends it as ``isError``."""

    def agent_encode(self) -> list[dict[str, str]]:
        return [{"type": "text", "text": self.text}]
