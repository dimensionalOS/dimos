# Copyright 2025-2026 Dimensional Inc.
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

"""Return type for ``@skill`` methods.

A skill returns a plain statement of what it did and measured, as a
``SkillResult``. The agent reading it decides whether that is what it wanted.
When the skill's own code or a driver fails, the skill raises instead; the MCP
server turns the exception into a failed tool call.

The MCP server (``dimos/agents/mcp/mcp_server.py``) calls ``agent_encode`` on
this class and forwards its output as the tool call's ``content``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any


@dataclass
class SkillResult:
    """What a ``@skill`` call did, written for the agent to read.

    Args:
        message: Plain statement of what happened, e.g. "Opened the gripper; it
            reads 0.08 m".
        duration_ms: Wall time the skill took, in milliseconds. The ``@skill``
            decorator fills this in.
        metadata: Extra values the agent may use, e.g. a list of object ids.
    """

    message: str = ""
    duration_ms: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def agent_encode(self) -> list[dict[str, Any]]:
        """Encode as MCP tool-call content: one text item holding a JSON object."""
        payload: dict[str, Any] = {
            "message": self.message,
            "duration_ms": round(self.duration_ms, 1),
        }
        if self.metadata:
            payload["metadata"] = self.metadata
        return [{"type": "text", "text": json.dumps(payload)}]

    def __str__(self) -> str:
        return self.message
