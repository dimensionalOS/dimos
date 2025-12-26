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


def create_tool_response(tool_id: str, result: Any, is_error: bool = False) -> dict[str, Any]:
    """Create a properly formatted tool response.

    Args:
        tool_id: The ID of the tool call
        result: The result from executing the tool
        is_error: Whether this is an error response

    Returns:
        Formatted tool response message
    """
    content = str(result) if not isinstance(result, str) else result

    return {
        "role": "tool",
        "tool_call_id": tool_id,
        "content": content,
        "name": None,  # Will be filled by the calling code
    }


def extract_image_from_message(message: dict[str, Any]) -> dict[str, Any] | None:
    """Extract image data from a message if present.

    Args:
        message: Message dict that may contain image data

    Returns:
        Dict with image data and metadata, or None if no image
    """
    content = message.get("content", [])

    # Handle list content (multimodal)
    if isinstance(content, list):
        for item in content:
            if isinstance(item, dict):
                # OpenAI format
                if item.get("type") == "image_url":
                    return {
                        "format": "openai",
                        "data": item["image_url"]["url"],
                        "detail": item["image_url"].get("detail", "auto"),
                    }
                # Anthropic format
                elif item.get("type") == "image":
                    return {
                        "format": "anthropic",
                        "data": item["source"]["data"],
                        "media_type": item["source"].get("media_type", "image/jpeg"),
                    }

    return None
