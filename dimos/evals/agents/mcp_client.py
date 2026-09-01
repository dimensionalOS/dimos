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

"""The shipped agent: the blueprint's own ``McpClient`` over ``/human_input``."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage, ToolMessage

from dimos.evals.agents.lib.recorder import reasoning_text
from dimos.evals.types import Environment, RunningEnvironment, Step, ToolCall, Trajectory


class _Turn:
    """One McpClient turn as seen on the wire: every message on ``/agent``
    from the moment ``/agent_idle`` goes False until it comes back True."""

    def __init__(self) -> None:
        self.received: list[BaseMessage] = []
        self.idle = threading.Event()
        self._started = threading.Event()

    def on_agent(self, msg: Any) -> None:
        if isinstance(msg, BaseMessage):
            self.received.append(msg)

    def on_idle(self, flag: Any) -> None:
        if flag is False:
            self._started.set()
        elif flag is True and self._started.is_set():
            self.idle.set()


def _pairs(raw_dir: Path) -> list[tuple[Path, Path]]:
    """The McpClient's request/response pairs under *raw_dir*, in order."""
    pairs: list[tuple[Path, Path]] = []
    for req in sorted(raw_dir.glob("*-request.json")):
        resp = req.with_name(req.name.replace("-request.json", "-response.json"))
        if resp.exists():
            pairs.append((req, resp))
    return pairs


@dataclass
class McpClientAgent:
    """The production agent: ``inputs`` on ``/human_input``, the turn read
    back from ``/agent`` until ``/agent_idle``. Its model and prompt are the
    ``McpClient`` module's own — configure the module, not this agent. Raw
    capture is the McpClient's trace dir, which the runner points at
    ``run_dir/raw``.

    ``modules`` is the agentic composite appended to the case's stack, e.g.
    ``"unitree-go2-agentic"`` (the whole shipped stack; ``autoconnect``
    dedups the base it shares with the case). ``""`` attaches to a dimos that
    is already running.
    """

    modules: str = ""

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """The shipped agent can call every tool its MCP server exposes."""
        return environment_tools

    def preflight(self, environment: Environment) -> None:
        from dimos.core.run_registry import list_runs

        if not environment.has_robot:
            raise RuntimeError(
                f"McpClientAgent needs a running McpClient; {type(environment).__name__} has no robot"
            )
        if not self.modules and not list_runs(alive_only=True):
            raise RuntimeError(
                "McpClientAgent adds no McpClient and no dimos is running to attach to"
            )

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        from dimos.core.transport_factory import make_transport

        turn = _Turn()
        agent_t, idle_t, human_t = (
            make_transport("/agent"),
            make_transport("/agent_idle"),
            make_transport("/human_input"),
        )
        for t in (agent_t, idle_t, human_t):
            t.start()
        t0 = time.monotonic()
        try:
            agent_t.subscribe(turn.on_agent)
            idle_t.subscribe(turn.on_idle)
            human_t.publish(inputs)
            turn.idle.wait()  # the runner's timeout is the only limit
        finally:
            for t in (agent_t, idle_t, human_t):
                t.stop()
        return self._trajectory(turn.received, run_dir, t0)

    def _trajectory(
        self,
        received: list[BaseMessage],
        run_dir: Path,
        t0: float,
    ) -> Trajectory:
        from dimos.agents.mcp.mcp_client import McpClientConfig

        raw_dir = run_dir / "raw"
        pairs = _pairs(raw_dir)
        steps: list[Step] = []
        final = ""
        cached = 0
        for msg in received:
            if isinstance(msg, AIMessage):
                if len(steps) >= len(pairs):
                    raise RuntimeError(
                        f"McpClient wrote no LLM trace for call {len(steps)} under {raw_dir}; "
                        "every call must be captured whole"
                    )
                usage: dict[str, Any] = dict(msg.usage_metadata or {})
                cache_read = int((usage.get("input_token_details") or {}).get("cache_read", 0))
                cached += cache_read
                steps.append(
                    Step(
                        index=len(steps),
                        t=time.monotonic() - t0,
                        message=str(msg.text),
                        reasoning=reasoning_text(msg.content),
                        tool_calls=tuple(
                            ToolCall(
                                id=str(tc["id"]),
                                name=str(tc["name"]),
                                args=dict(tc.get("args") or {}),
                            )
                            for tc in msg.tool_calls
                        ),
                        input_tokens=int(usage.get("input_tokens", 0)) - cache_read,
                        output_tokens=int(usage.get("output_tokens", 0)),
                        reasoning_tokens=int(
                            (usage.get("output_token_details") or {}).get("reasoning", 0)
                        ),
                        request=pairs[len(steps)][0],
                        response=pairs[len(steps)][1],
                    )
                )
                if not msg.tool_calls:
                    final = str(msg.text)
            elif isinstance(msg, ToolMessage) and steps:
                call = next(call for call in steps[-1].tool_calls if call.id == msg.tool_call_id)
                call.result = str(msg.content)
        model = next(
            (
                str(m.response_metadata.get("model_name"))
                for m in received
                if isinstance(m, AIMessage) and m.response_metadata.get("model_name")
            ),
            McpClientConfig().model,
        )
        return Trajectory(
            final_answer=final,
            steps=tuple(steps),
            model=model,
            input_tokens=sum(s.input_tokens for s in steps),
            output_tokens=sum(s.output_tokens for s in steps),
            cached_tokens=cached,
            reasoning_tokens=sum(s.reasoning_tokens for s in steps),
            duration_s=time.monotonic() - t0,
            ended_by="answer",
            raw_dir=run_dir / "raw",
        )
