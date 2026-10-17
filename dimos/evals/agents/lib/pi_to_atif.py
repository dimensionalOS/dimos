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

"""Translate Pi events into typed ATIF steps paired with model requests."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, TypeAdapter, ValidationError
from pydantic.alias_generators import to_camel
from typing_extensions import TypedDict

from dimos.agents.llm_trace import list_llm_trace_pairs
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import Metrics, ToolCall


class TextContent(TypedDict):
    type: Literal["text"]
    text: str


class ThinkingContent(TypedDict):
    type: Literal["thinking"]
    thinking: str


class ToolContent(TypedDict):
    type: Literal["toolCall"]
    id: str
    name: str
    arguments: dict[str, JsonValue]


Content = Annotated[TextContent | ThinkingContent | ToolContent, Field(discriminator="type")]


class PiCost(BaseModel):
    total: float | None = None


class PiUsage(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, strict=True)

    input: int = 0
    output: int = 0
    cache_read: int = 0
    cache_write: int = 0
    reasoning: int = 0
    cost: PiCost = Field(default_factory=PiCost)


class AssistantMessage(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel)

    response_id: str | None = None
    response_model: str = ""
    model: str = ""
    stop_reason: str = ""
    error_message: str = ""
    content: tuple[Content, ...] = ()
    usage: PiUsage = Field(default_factory=PiUsage)


class TraceResponse(BaseModel):
    body: JsonValue
    latency_s: float = 0.0


_json_object = TypeAdapter(dict[str, JsonValue])


def _result_text(result: JsonValue) -> str:
    content = result.get("content") if isinstance(result, dict) else None
    if isinstance(content, list):
        return "\n".join(
            text
            for c in content
            if isinstance(c, dict)
            and c.get("type") == "text"
            and isinstance(text := c.get("text"), str)
        )
    return str(result)


def _response_id(body: JsonValue) -> str | None:
    """Read the provider response ID from an OpenAI or Anthropic JSON/SSE body."""
    if isinstance(body, dict):
        response_id = body.get("id")
        return response_id if isinstance(response_id, str) else None
    if isinstance(body, str):
        for line in body.splitlines():
            if not line.startswith("data:"):
                continue
            data = line.removeprefix("data:").strip()
            if data == "[DONE]":
                continue
            event = _json_object.validate_json(data)
            response = event.get("response") or event.get("message")
            if isinstance(response, dict) and isinstance(response.get("id"), str):
                return str(response["id"])
    return None


class PiToAtif:
    """Append assistant steps and tool results from Pi's JSON event stream."""

    def __init__(self, raw_dir: Path, trajectory: TrajectoryBuilder) -> None:
        self.raw_dir = raw_dir
        self.trajectory = trajectory
        self.calls = 0
        self.wants_tool = False
        self.error = ""
        self._next_seq = 0

    def append_event(self, event: dict[str, JsonValue]) -> None:
        message = event.get("message")
        if event.get("type") == "tool_execution_end" and self.calls:
            call_id = event.get("toolCallId")
            if not isinstance(call_id, str):
                raise ValueError("Pi tool result has no tool call ID")
            self.trajectory.observe(call_id, _result_text(event.get("result")))
        elif (
            event.get("type") == "message_end"
            and isinstance(message, dict)
            and message.get("role") == "assistant"
        ):
            self._step(AssistantMessage.model_validate(message))

    def _trace_for_response(self, response_id: str) -> tuple[Path, Path, float]:
        """Match by provider ID: newer traces may already exist when stdout is buffered."""
        for seq, request, response in list_llm_trace_pairs(self.raw_dir):
            if seq < self._next_seq:
                continue
            try:
                record = TraceResponse.model_validate_json(response.read_text())
                recorded_id = _response_id(record.body)
            except ValidationError:
                # Abandoned attempts can leave partial traces; the matched response is complete.
                continue
            if recorded_id == response_id:
                self._next_seq = seq + 1
                return request, response, record.latency_s
        raise RuntimeError(f"No recorded HTTP response for Pi response {response_id!r}")

    def _step(self, message: AssistantMessage) -> None:
        self.calls += 1
        self.wants_tool = False
        self.error = (
            message.error_message or message.stop_reason
            if message.stop_reason in ("error", "aborted")
            else ""
        )
        if not message.response_id:
            if self.error:
                # Retries may recover; keep the failed attempt's raw logs.
                return
            raise RuntimeError("Pi assistant message has no provider response ID")
        request, response, latency_s = self._trace_for_response(message.response_id)
        usage = message.usage
        tool_calls = tuple(
            ToolCall(tool_call_id=c["id"], function_name=c["name"], arguments=c["arguments"])
            for c in message.content
            if c["type"] == "toolCall"
        )
        # Pi's input excludes cache traffic: what was sent is the three together.
        self.trajectory.step(
            message="".join(c["text"] for c in message.content if c["type"] == "text"),
            reasoning="\n\n".join(
                c["thinking"] for c in message.content if c["type"] == "thinking"
            ),
            tool_calls=tool_calls,
            metrics=Metrics(
                prompt_tokens=usage.input + usage.cache_write + usage.cache_read,
                completion_tokens=usage.output,
                cached_tokens=usage.cache_read,
                cost_usd=usage.cost.total,
            ),
            model_name=message.response_model or message.model,
            latency_s=latency_s,
            reasoning_tokens=usage.reasoning,
            request=request,
            response=response,
        )
        self.wants_tool = bool(tool_calls)
