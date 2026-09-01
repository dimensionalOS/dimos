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

"""LangChain objects in, a trajectory out."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any
from uuid import UUID

from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage
from langchain_core.outputs import LLMResult

from dimos.agents.llm_trace import latest_pair, write_normalized
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import EndedBy, Metrics, ToolCall, Trajectory


def _reasoning_text(content: Any) -> str:
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
                parts.append(
                    "".join(str(s.get("text", "")) for s in summary if isinstance(s, dict))
                )
    return "\n\n".join(p for p in parts if p)


class LangChainRecorder(BaseCallbackHandler):
    """One run's trajectory, fed LangChain messages.

    Two ways in. As a callback handler — ``config={"callbacks": [recorder]}``
    on ``invoke``/``stream``, inherited by the graph's model and tool runs —
    every model call becomes a step paired with the request/response files
    the HTTP hook wrote for it, or the normalized messages when a provider
    has no hook. Or by hand: :meth:`record` for an ``AIMessage`` read off the
    wire, :meth:`observe` for a ``ToolMessage``. Then :meth:`build`.
    """

    raise_error = True  # a recorder bug must not silently drop steps

    def __init__(self, inputs: str, *, name: str, model: str, raw_dir: Path) -> None:
        self.raw_dir = raw_dir
        self._trajectory = TrajectoryBuilder(inputs, name=name, model=model)
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
        pair = latest_pair(self.raw_dir, self._next_seq)
        if pair is None:
            pair = write_normalized(self.raw_dir, sent, response)
        self._next_seq = pair[0] + 1
        self.record(
            message, request=pair[1], response=pair[2], latency_s=time.monotonic() - started
        )

    def record(
        self,
        message: Any,
        *,
        request: Path,
        response: Path,
        latency_s: float = 0.0,
        at: float | None = None,
    ) -> None:
        """An ``AIMessage`` as one agent step paired with its raw payloads."""
        usage = getattr(message, "usage_metadata", None) or {}
        meta = getattr(message, "response_metadata", None) or {}
        self._trajectory.step(
            message=str(message.text),
            reasoning=_reasoning_text(message.content),
            tool_calls=tuple(
                ToolCall(
                    tool_call_id=str(tc["id"]),
                    function_name=str(tc["name"]),
                    arguments=dict(tc.get("args") or {}),
                )
                for tc in getattr(message, "tool_calls", None) or []
            ),
            metrics=Metrics(
                prompt_tokens=int(usage.get("input_tokens", 0)),
                completion_tokens=int(usage.get("output_tokens", 0)),
                cached_tokens=int((usage.get("input_token_details") or {}).get("cache_read", 0)),
            ),
            model_name=str(meta.get("model_name") or ""),
            latency_s=latency_s,
            reasoning_tokens=int((usage.get("output_token_details") or {}).get("reasoning", 0)),
            request=request,
            response=response,
            at=at,
        )

    def observe(self, call_id: str, content: str) -> None:
        """A tool result, for the call in the latest step that made it."""
        self._trajectory.observe(call_id, content)

    def build(self, ended_by: EndedBy) -> Trajectory:
        return self._trajectory.build(ended_by)
