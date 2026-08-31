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

"""LangChain callbacks -> one :class:`~dimos.evals.types.Step` per model call."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Any
from uuid import UUID

from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage
from langchain_core.outputs import LLMResult

from dimos.agents.llm_trace import latest_pair, write_normalized
from dimos.evals.types import EndedBy, Step, ToolCall, Trajectory


def reasoning_text(content: Any) -> str:
    """Readable reasoning from provider content blocks — Anthropic ``thinking``
    text or OpenAI ``reasoning`` summaries — "" when there is none."""
    parts: list[str] = []
    for block in content if isinstance(content, list) else []:
        if not isinstance(block, dict):
            continue
        if block.get("type") == "thinking":
            parts.append(str(block.get("thinking", "")))
        elif block.get("type") == "reasoning":
            summary = block.get("summary")
            if isinstance(summary, list):
                parts.append("".join(str(s.get("text", "")) for s in summary if isinstance(s, dict)))
    return "\n\n".join(p for p in parts if p)


@dataclass
class StepDraft:
    """A :class:`Step` still collecting tool results."""

    index: int
    t: float
    message: str
    tool_calls: list[ToolCall]
    input_tokens: int
    output_tokens: int
    latency_s: float
    request: Path
    response: Path
    reasoning: str = ""
    reasoning_tokens: int = 0
    observations: list[str] = field(default_factory=list)

    def freeze(self) -> Step:
        return Step(
            index=self.index,
            t=self.t,
            message=self.message,
            reasoning=self.reasoning,
            tool_calls=tuple(self.tool_calls),
            observations=tuple(self.observations),
            input_tokens=self.input_tokens,
            output_tokens=self.output_tokens,
            reasoning_tokens=self.reasoning_tokens,
            latency_s=self.latency_s,
            request=self.request,
            response=self.response,
        )


class StepRecorder(BaseCallbackHandler):
    """LangChain callbacks -> one :class:`Step` per model call.

    Pass as ``config={"callbacks": [recorder]}`` to ``invoke``/``stream``;
    the graph inherits it into the model and tool runs. Each step is paired
    with the request/response files the HTTP hook wrote for that call; when a
    provider has no hook, the normalized messages and result are written
    instead. Fresh per run: it holds the run's clock and step count.
    """

    raise_error = True  # a recorder bug must not silently drop steps

    def __init__(self, raw_dir: Path) -> None:
        self.raw_dir = raw_dir
        self.t0 = time.monotonic()
        self.model_name = ""
        self.cached_tokens = 0
        self._drafts: list[StepDraft] = []
        self._open: dict[UUID, tuple[list[BaseMessage], float]] = {}
        self._next_seq = 0

    def on_chat_model_start(
        self,
        serialized: dict[str, Any],
        messages: list[list[BaseMessage]],
        *,
        run_id: UUID,
        **kwargs: Any,
    ) -> None:
        self._open[run_id] = (list(messages[0]), time.monotonic())

    def on_llm_end(self, response: LLMResult, *, run_id: UUID, **kwargs: Any) -> None:
        sent, started = self._open.pop(run_id)
        message: Any = getattr(response.generations[0][0], "message", None)
        if message is None:
            raise TypeError("chat model returned a bare Generation, not a message")
        usage = getattr(message, "usage_metadata", None) or {}
        meta = getattr(message, "response_metadata", None) or {}
        self.model_name = str(meta.get("model_name") or self.model_name)
        cache_read = int((usage.get("input_token_details") or {}).get("cache_read", 0))
        self.cached_tokens += cache_read
        pair = latest_pair(self.raw_dir, self._next_seq)
        if pair is None:
            pair = write_normalized(self.raw_dir, sent, response)
        self._next_seq = pair[0] + 1
        tool_calls = [
            ToolCall(name=str(tc["name"]), args=dict(tc.get("args") or {}))
            for tc in getattr(message, "tool_calls", None) or []
        ]
        self._drafts.append(
            StepDraft(
                index=len(self._drafts),
                t=started - self.t0,
                message=str(message.text),
                reasoning=reasoning_text(message.content),
                tool_calls=tool_calls,
                input_tokens=int(usage.get("input_tokens", 0)) - cache_read,
                output_tokens=int(usage.get("output_tokens", 0)),
                reasoning_tokens=int((usage.get("output_token_details") or {}).get("reasoning", 0)),
                latency_s=time.monotonic() - started,
                request=pair[1],
                response=pair[2],
            )
        )

    def trajectory(self, *, final_answer: str, model: str, ended_by: EndedBy) -> Trajectory:
        steps = tuple(d.freeze() for d in self._drafts)
        return Trajectory(
            final_answer=final_answer,
            steps=steps,
            model=self.model_name or model,
            input_tokens=sum(s.input_tokens for s in steps),
            output_tokens=sum(s.output_tokens for s in steps),
            cached_tokens=self.cached_tokens,
            reasoning_tokens=sum(s.reasoning_tokens for s in steps),
            duration_s=time.monotonic() - self.t0,
            ended_by=ended_by,
            raw_dir=self.raw_dir,
        )
