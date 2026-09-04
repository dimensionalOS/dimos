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

"""Building a :class:`~dimos.evals.types.Trajectory` incrementally.

Agents learn what happened one event at a time (a model call finished, a
tool returned) while the trajectory they must return is a frozen document
with sequential step ids and totals. :class:`TrajectoryBuilder` takes the
events in order and produces the document once at the end.
"""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
import time

from dimos.evals.types import (
    AgentInfo,
    EndedBy,
    FinalMetrics,
    Metrics,
    Observation,
    ObservationResult,
    RunExtra,
    Step,
    StepExtra,
    ToolCall,
    Trajectory,
)


def _iso(epoch: float) -> str:
    return datetime.fromtimestamp(epoch, tz=timezone.utc).isoformat()


class TrajectoryBuilder:
    """The instruction first, then one :meth:`step` per model call and one
    :meth:`observe` per tool result, then :meth:`build`. ``model`` is what the
    agent asked for; a step reporting the provider's own name overrides it."""

    def __init__(self, inputs: str, *, name: str, model: str = "") -> None:
        self.model_name = model
        self._name = name
        self._steps: list[Step] = [
            Step(step_id=1, timestamp=_iso(time.time()), source="user", message=inputs)
        ]

    def step(
        self,
        *,
        message: str,
        request: Path,
        response: Path,
        reasoning: str = "",
        tool_calls: tuple[ToolCall, ...] = (),
        metrics: Metrics = Metrics(prompt_tokens=0, completion_tokens=0),
        model_name: str = "",
        latency_s: float = 0.0,
        reasoning_tokens: int = 0,
        at: float | None = None,  # epoch seconds the call started; now - latency if None
    ) -> None:
        self.model_name = model_name or self.model_name
        self._steps.append(
            Step(
                step_id=len(self._steps) + 1,
                timestamp=_iso(time.time() - latency_s if at is None else at),
                source="agent",
                message=message,
                model_name=self.model_name,
                reasoning_content=reasoning or None,
                tool_calls=tool_calls,
                metrics=metrics,
                extra=StepExtra(
                    request=request,
                    response=response,
                    latency_s=latency_s,
                    reasoning_tokens=reasoning_tokens,
                ),
            )
        )

    def observe(self, call_id: str, content: str) -> None:
        """A tool result, attached to the call in the latest step that made it."""
        last = self._steps[-1]
        if not any(c.tool_call_id == call_id for c in last.tool_calls or ()):
            raise RuntimeError(
                f"tool result for {call_id!r}, which step {last.step_id} never called"
            )
        results = (
            *(last.observation.results if last.observation else ()),
            ObservationResult(source_call_id=call_id, content=content),
        )
        self._steps[-1] = replace(last, observation=Observation(results=results))

    def build(self, ended_by: EndedBy) -> Trajectory:
        metrics = [s.metrics for s in self._steps if s.metrics]
        return Trajectory(
            agent=AgentInfo(name=self._name, version=version("dimos"), model_name=self.model_name),
            steps=tuple(self._steps),
            final_metrics=FinalMetrics(
                total_prompt_tokens=sum(m.prompt_tokens for m in metrics),
                total_completion_tokens=sum(m.completion_tokens for m in metrics),
                total_cached_tokens=sum(m.cached_tokens for m in metrics),
                total_cost_usd=sum(m.cost_usd or 0.0 for m in metrics),
                total_steps=len(self._steps),
            ),
            extra=RunExtra(ended_by=ended_by),
        )
