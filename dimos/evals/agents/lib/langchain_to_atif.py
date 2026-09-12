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

"""Translate LangChain AI messages into ATIF trajectory steps."""

from __future__ import annotations

from pathlib import Path

from langchain_core.messages import AIMessage
from langchain_core.messages.ai import UsageMetadata

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import Metrics, ToolCall


def append_ai_message_to_atif(
    trajectory: TrajectoryBuilder,
    message: AIMessage,
    *,
    request: Path,
    response: Path,
    latency_s: float = 0.0,
    at: float | None = None,
) -> None:
    """An ``AIMessage`` as one agent step paired with its raw payloads."""
    usage: UsageMetadata = message.usage_metadata or {
        "input_tokens": 0,
        "output_tokens": 0,
        "total_tokens": 0,
    }
    meta = message.response_metadata
    trajectory.step(
        message=str(message.text),
        reasoning="\n\n".join(
            block["reasoning"]
            for block in message.content_blocks
            if block["type"] == "reasoning" and block.get("reasoning")
        ),
        tool_calls=tuple(
            ToolCall(
                tool_call_id=str(tc["id"]),
                function_name=tc["name"],
                arguments=tc["args"],
            )
            for tc in message.tool_calls
        ),
        metrics=Metrics(
            prompt_tokens=usage["input_tokens"],
            completion_tokens=usage["output_tokens"],
            cached_tokens=(usage.get("input_token_details") or {}).get("cache_read", 0),
        ),
        model_name=str(meta.get("model_name") or ""),
        latency_s=latency_s,
        reasoning_tokens=(usage.get("output_token_details") or {}).get("reasoning", 0),
        request=request,
        response=response,
        at=at,
    )
