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
import json
import threading
import time
from typing import Any


@dataclass
class AgentImage:
    """Image data encoded for agent consumption.

    Images are stored as base64-encoded JPEG strings ready for
    direct use by LLM/vision models.
    """

    base64_jpeg: str
    width: int | None = None
    height: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __repr__(self) -> str:
        return f"AgentImage(size={self.width}x{self.height}, metadata={list(self.metadata.keys())})"


@dataclass
class ToolCall:
    """Represents a tool/function call request from the LLM."""

    id: str
    name: str
    arguments: dict[str, Any]
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
    tool_calls: list[ToolCall] | None = None
    requires_follow_up: bool = False  # Indicates if tool execution is needed
    metadata: dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def __repr__(self) -> str:
        content_preview = self.content[:50] + "..." if len(self.content) > 50 else self.content
        tool_info = f", tools={len(self.tool_calls)}" if self.tool_calls else ""
        return f"AgentResponse(role='{self.role}', content='{content_preview}'{tool_info})"


@dataclass
class ConversationMessage:
    """Single message in conversation history.

    Represents a message in the conversation that can be converted to
    different formats (OpenAI, TensorZero, etc).
    """

    role: str  # "system", "user", "assistant", "tool"
    content: str | list[dict[str, Any]]  # Text or content blocks
    tool_calls: list[ToolCall] | None = None
    tool_call_id: str | None = None  # For tool responses
    name: str | None = None  # For tool messages (function name)
    timestamp: float = field(default_factory=time.time)

    def to_openai_format(self) -> dict[str, Any]:
        """Convert to OpenAI API format."""
        msg = {"role": self.role}

        # Handle content
        if isinstance(self.content, str):
            msg["content"] = self.content
        else:
            # Content is already a list of content blocks
            msg["content"] = self.content  # type: ignore[assignment]

        # Add tool calls if present
        if self.tool_calls:
            # Handle both ToolCall objects and dicts
            if isinstance(self.tool_calls[0], dict):
                msg["tool_calls"] = self.tool_calls  # type: ignore[assignment]
            else:
                msg["tool_calls"] = [  # type: ignore[assignment]
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {"name": tc.name, "arguments": json.dumps(tc.arguments)},
                    }
                    for tc in self.tool_calls
                ]

        # Add tool_call_id for tool responses
        if self.tool_call_id:
            msg["tool_call_id"] = self.tool_call_id

        # Add name field if present (for tool messages)
        if self.name:
            msg["name"] = self.name

        return msg

    def __repr__(self) -> str:
        content_preview = (
            str(self.content)[:50] + "..." if len(str(self.content)) > 50 else str(self.content)
        )
        return f"ConversationMessage(role='{self.role}', content='{content_preview}')"


class ConversationHistory:
    """Thread-safe conversation history manager.

    Manages conversation history with proper formatting for different
    LLM providers and automatic trimming.
    """

    def __init__(self, max_size: int = 20) -> None:
        """Initialize conversation history.

        Args:
            max_size: Maximum number of messages to keep
        """
        self._messages: list[ConversationMessage] = []
        self._lock = threading.Lock()
        self.max_size = max_size

    def add_user_message(self, content: str | list[dict[str, Any]]) -> None:
        """Add user message to history.

        Args:
            content: Text string or list of content blocks (for multimodal)
        """
        with self._lock:
            self._messages.append(ConversationMessage(role="user", content=content))
            self._trim()

    def add_assistant_message(self, content: str, tool_calls: list[ToolCall] | None = None) -> None:
        """Add assistant response to history.

        Args:
            content: Response text
            tool_calls: Optional list of tool calls made
        """
        with self._lock:
            self._messages.append(
                ConversationMessage(role="assistant", content=content, tool_calls=tool_calls)
            )
            self._trim()

    def add_tool_result(self, tool_call_id: str, content: str, name: str | None = None) -> None:
        """Add tool execution result to history.

        Args:
            tool_call_id: ID of the tool call this is responding to
            content: Result of the tool execution
            name: Optional name of the tool/function
        """
        with self._lock:
            self._messages.append(
                ConversationMessage(
                    role="tool", content=content, tool_call_id=tool_call_id, name=name
                )
            )
            self._trim()

    def add_raw_message(self, message: dict[str, Any]) -> None:
        """Add a raw message dict to history.

        Args:
            message: Message dict with role and content
        """
        with self._lock:
            # Extract fields from raw message
            role = message.get("role", "user")
            content = message.get("content", "")

            # Handle tool calls if present
            tool_calls = None
            if "tool_calls" in message:
                tool_calls = [
                    ToolCall(
                        id=tc["id"],
                        name=tc["function"]["name"],
                        arguments=json.loads(tc["function"]["arguments"])
                        if isinstance(tc["function"]["arguments"], str)
                        else tc["function"]["arguments"],
                        status="completed",
                    )
                    for tc in message["tool_calls"]
                ]

            # Handle tool_call_id for tool responses
            tool_call_id = message.get("tool_call_id")

            self._messages.append(
                ConversationMessage(
                    role=role, content=content, tool_calls=tool_calls, tool_call_id=tool_call_id
                )
            )
            self._trim()

    def to_openai_format(self) -> list[dict[str, Any]]:
        """Export history in OpenAI format.

        Returns:
            List of message dicts in OpenAI format
        """
        with self._lock:
            return [msg.to_openai_format() for msg in self._messages]

    def clear(self) -> None:
        """Clear all conversation history."""
        with self._lock:
            self._messages.clear()

    def size(self) -> int:
        """Get number of messages in history.

        Returns:
            Number of messages
        """
        with self._lock:
            return len(self._messages)

    def _trim(self) -> None:
        """Trim history to max_size (must be called within lock)."""
        if len(self._messages) > self.max_size:
            # Keep the most recent messages
            self._messages = self._messages[-self.max_size :]

    def __repr__(self) -> str:
        with self._lock:
            return f"ConversationHistory(messages={len(self._messages)}, max_size={self.max_size})"
