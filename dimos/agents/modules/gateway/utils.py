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

"""Utility functions for gateway operations."""

import logging
from typing import Any

logger = logging.getLogger(__name__)


def convert_tools_to_standard_format(tools: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Convert DimOS tool format to standard format accepted by gateways.

    DimOS tools come from pydantic_function_tool and have this format:
    {
        "type": "function",
        "function": {
            "name": "tool_name",
            "description": "tool description",
            "parameters": {
                "type": "object",
                "properties": {...},
                "required": [...]
            }
        }
    }

    We keep this format as it's already standard JSON Schema format.
    """
    if not tools:
        return []

    # Tools are already in the correct format from pydantic_function_tool
    return tools


def parse_streaming_response(chunk: dict[str, Any]) -> dict[str, Any]:
    """Parse a streaming response chunk into a standard format.

    Args:
        chunk: Raw chunk from the gateway

    Returns:
        Parsed chunk with standard fields:
        - type: "content" | "tool_call" | "error" | "done"
        - content: The actual content (text for content type, tool info for tool_call)
        - metadata: Additional information
    """
    # Handle TensorZero streaming format
    if "choices" in chunk:
        # OpenAI-style format from TensorZero
        choice = chunk["choices"][0] if chunk["choices"] else {}
        delta = choice.get("delta", {})

        if "content" in delta:
            return {
                "type": "content",
                "content": delta["content"],
                "metadata": {"index": choice.get("index", 0)},
            }
        elif "tool_calls" in delta:
            tool_calls = delta["tool_calls"]
            if tool_calls:
                tool_call = tool_calls[0]
                return {
                    "type": "tool_call",
                    "content": {
                        "id": tool_call.get("id"),
                        "name": tool_call.get("function", {}).get("name"),
                        "arguments": tool_call.get("function", {}).get("arguments", ""),
                    },
                    "metadata": {"index": tool_call.get("index", 0)},
                }
        elif choice.get("finish_reason"):
            return {
                "type": "done",
                "content": None,
                "metadata": {"finish_reason": choice["finish_reason"]},
            }

    # Handle direct content chunks
    if isinstance(chunk, str):
        return {"type": "content", "content": chunk, "metadata": {}}

    # Handle error responses
    if "error" in chunk:
        return {"type": "error", "content": chunk["error"], "metadata": chunk}

    # Default fallback
    return {"type": "unknown", "content": chunk, "metadata": {}}
