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

"""Translate Pi events into ATIF steps paired with recorded model requests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.agents.llm_trace import list_llm_trace_pairs
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import Metrics, ToolCall


def _result_text(result: Any) -> str:
    content = result.get("content") if isinstance(result, dict) else None
    if isinstance(content, list):
        return "\n".join(str(c.get("text", "")) for c in content if c.get("type") == "text")
    return str(result)


def _response_id(body: Any) -> str | None:
    """Read the provider response ID from an OpenAI or Anthropic JSON/SSE body."""
    if isinstance(body, dict):
        response_id = body.get("id")
        return str(response_id) if response_id else None
    if isinstance(body, str):
        for line in body.splitlines():
            if not line.startswith("data:"):
                continue
            data = line.removeprefix("data:").strip()
            if data == "[DONE]":
                continue
            event = json.loads(data)
            if response_id := (event.get("response") or event.get("message") or {}).get("id"):
                return str(response_id)
    return None


class PiToAtif:
    """Append assistant steps and tool results from Pi's JSON event stream."""

    def __init__(self, raw_dir: Path, trajectory: TrajectoryBuilder) -> None:
        self.raw_dir = raw_dir
        self.trajectory = trajectory
        self.calls = 0  # model calls seen
        self.wants_tool = False  # the latest one asked for a tool
        self.error = ""
        self._next_seq = 0

    def append_event(self, event: dict[str, Any]) -> None:
        if event.get("type") == "tool_execution_end" and self.calls:
            self.trajectory.observe(str(event["toolCallId"]), _result_text(event.get("result")))
        elif event.get("type") == "message_end" and event["message"].get("role") == "assistant":
            self._step(event["message"])

    def _trace_for_response(self, response_id: str) -> tuple[Path, Path, float]:
        """Match by provider ID: newer traces may already exist when stdout is buffered."""
        for seq, request, response in list_llm_trace_pairs(self.raw_dir):
            if seq < self._next_seq:
                continue
            try:
                record = json.loads(response.read_text())
                recorded_id = _response_id(record["body"])
            except json.JSONDecodeError:
                # An abandoned attempt may still be writing. The matching
                # response is complete before the proxy delivers it to Pi.
                continue
            if recorded_id == response_id:
                self._next_seq = seq + 1
                return request, response, float(record.get("latency_s") or 0.0)
        raise RuntimeError(f"No recorded HTTP response for Pi response {response_id!r}")

    def _step(self, message: dict[str, Any]) -> None:
        self.calls += 1
        self.wants_tool = False
        self.error = (
            str(message.get("errorMessage") or message["stopReason"])
            if message.get("stopReason") in ("error", "aborted")
            else ""
        )
        response_id = message.get("responseId")
        if not response_id:
            if self.error:
                # Pi reported no response ID. Keep the raw failure logs;
                # a successful retry will clear this error and record its own step.
                return
            raise RuntimeError("Pi assistant message has no provider response ID")
        request, response, latency_s = self._trace_for_response(response_id)
        usage = message.get("usage") or {}
        content = message.get("content") or []
        tool_calls = tuple(
            ToolCall(
                tool_call_id=str(c["id"]),
                function_name=str(c["name"]),
                arguments=dict(c.get("arguments") or {}),
            )
            for c in content
            if c.get("type") == "toolCall"
        )
        # Pi's ``input`` excludes cache traffic: what was sent is the three together.
        cached = int(usage.get("cacheRead", 0))
        self.trajectory.step(
            message="".join(str(c.get("text", "")) for c in content if c.get("type") == "text"),
            reasoning="\n\n".join(
                str(c.get("thinking", "")) for c in content if c.get("type") == "thinking"
            ),
            tool_calls=tool_calls,
            metrics=Metrics(
                prompt_tokens=int(usage.get("input", 0)) + int(usage.get("cacheWrite", 0)) + cached,
                completion_tokens=int(usage.get("output", 0)),
                cached_tokens=cached,
                cost_usd=(
                    float(usage["cost"]["total"])
                    if usage.get("cost", {}).get("total") is not None
                    else None
                ),
            ),
            model_name=str(message.get("responseModel") or message.get("model") or ""),
            latency_s=latency_s,
            reasoning_tokens=int(usage.get("reasoning", 0)),
            request=request,
            response=response,
        )
        self.wants_tool = bool(tool_calls)
