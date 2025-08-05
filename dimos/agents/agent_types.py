# Copyright 2025 Dimensional Inc.
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

"""Agent-specific types for message passing."""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
import time


@dataclass
class AgentImage:
    """Image data encoded for agent consumption.

    Images are stored as base64-encoded JPEG strings ready for
    direct use by LLM/vision models.
    """

    base64_jpeg: str
    width: Optional[int] = None
    height: Optional[int] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __repr__(self) -> str:
        return f"AgentImage(size={self.width}x{self.height}, metadata={list(self.metadata.keys())})"


@dataclass
class ToolCall:
    """Represents a tool/function call request from the LLM."""

    id: str
    name: str
    arguments: Dict[str, Any]
    status: str = "pending"  # pending, executing, completed, failed

    def __repr__(self) -> str:
        return f"ToolCall(id='{self.id}', name='{self.name}', status='{self.status}')"


@dataclass
class AgentResponse:
    """Enhanced response from an agent query with tool support.

    Based on common LLM response patterns, includes content and metadata.
    """

    content: str
    role: str = "assistant"
    tool_calls: Optional[List[ToolCall]] = None
    requires_follow_up: bool = False  # Indicates if tool execution is needed
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def __repr__(self) -> str:
        content_preview = self.content[:50] + "..." if len(self.content) > 50 else self.content
        tool_info = f", tools={len(self.tool_calls)}" if self.tool_calls else ""
        return f"AgentResponse(role='{self.role}', content='{content_preview}'{tool_info})"
