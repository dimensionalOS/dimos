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

from dimos.agents.llm_trace import latest_pair
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import Metrics, ToolCall


def _result_text(result: Any) -> str:
    content = result.get("content") if isinstance(result, dict) else None
    if isinstance(content, list):
        return "\n".join(str(c.get("text", "")) for c in content if c.get("type") == "text")
    return str(result)


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

    def _step(self, message: dict[str, Any]) -> None:
        pair = latest_pair(self.raw_dir, self._next_seq)
        if pair is None:
            raise RuntimeError(
                f"Pi made a model call that left no trace under {self.raw_dir}; "
                "every call must go through the recording proxy"
            )
        self._next_seq = pair[0] + 1
        usage = message.get("usage") or {}
        content = message.get("content") or []
        if message.get("stopReason") in ("error", "aborted"):
            self.error = str(message.get("errorMessage") or message["stopReason"])
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
                cost_usd=float((usage.get("cost") or {}).get("total") or 0.0),
            ),
            model_name=str(message.get("responseModel") or message.get("model") or ""),
            latency_s=float(json.loads(pair[2].read_text()).get("latency_s") or 0.0),
            reasoning_tokens=int(usage.get("reasoning", 0)),
            request=pair[1],
            response=pair[2],
        )
        self.calls += 1
        self.wants_tool = bool(tool_calls)
