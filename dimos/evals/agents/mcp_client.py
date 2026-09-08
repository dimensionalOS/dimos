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

"""Evaluate the production McpClient through its input and output topics."""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Any

from langchain_core.messages import AIMessage, BaseMessage, ToolMessage

from dimos.agents.llm_trace import list_llm_trace_pairs
from dimos.evals.agents.base import Agent
from dimos.evals.agents.lib.langchain_to_atif import append_ai_message_to_atif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment, Trajectory


class _Turn:
    """One McpClient turn as seen on the wire: every message on ``/agent``
    from the moment ``/agent_idle`` goes False until it comes back True.
    ``/agent_idle`` is its own topic and can overtake the last ``/agent``
    message, so the turn is done only once the received ``AIMessage``s match
    the raw trace, which is complete before the idle flip is published."""

    def __init__(self, raw_dir: Path) -> None:
        self.received: list[BaseMessage] = []
        self.done = threading.Event()
        self._started = threading.Event()
        self._lock = threading.Lock()
        self._raw_dir = raw_dir
        self._expected: int | None = None

    def on_agent(self, msg: Any) -> None:
        if isinstance(msg, BaseMessage):
            with self._lock:
                self.received.append(msg)
                self._maybe_done()

    def on_idle(self, flag: Any) -> None:
        if flag is False:
            self._started.set()
        elif flag is True and self._started.is_set():
            with self._lock:
                self._expected = len(list_llm_trace_pairs(self._raw_dir))
                self._maybe_done()

    def _maybe_done(self) -> None:
        if self._expected is not None and (
            sum(isinstance(m, AIMessage) for m in self.received) >= self._expected
        ):
            self.done.set()


class McpClientAgent(Agent):
    """An eval adapter for the production ``McpClient``.

    Send the instruction on ``/human_input`` and capture ``/agent`` until
    ``/agent_idle``. Configure the model and prompt on the production module;
    this adapter only redirects its trace to ``run_dir/raw`` for each case.
    ``modules`` adds an agentic stack to the environment; an empty sequence
    uses the agent already supplied by the environment.
    """

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """The shipped agent can call every tool its MCP server exposes."""
        return environment_tools

    def preflight(self, environment: Environment) -> None:
        if not environment.has_robot and not self.config.modules:
            raise RuntimeError(
                f"McpClientAgent needs a running McpClient; {type(environment).__name__} "
                "has no robot and this agent adds no modules"
            )

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        from dimos.agents.mcp.mcp_client import McpClientConfig
        from dimos.core.transport_factory import make_transport
        from dimos.porcelain.dimos import Dimos

        # Set the directory for logging raw request/response payloads
        # in dimos.agents.mcp.mcp_client.McpClient
        app = Dimos.connect()
        try:
            mcp_client: Any = app.McpClient  # handle type depends on what's importable
            mcp_client.set_trace_dir(str(run_dir / "raw"))
        finally:
            app.stop()

        # init the stateful trajectory builder and subscribe to McpClient events
        trajectory = TrajectoryBuilder(
            inputs, name=type(self).__name__, model=McpClientConfig().model
        )
        turn = _Turn(run_dir / "raw")
        agent_t, idle_t, human_t = (
            make_transport("/agent"),
            make_transport("/agent_idle"),
            make_transport("/human_input"),
        )
        for t in (agent_t, idle_t, human_t):
            t.start()
        try:
            agent_t.subscribe(turn.on_agent)
            idle_t.subscribe(turn.on_idle)
            human_t.publish(inputs)
            finished = turn.done.wait(timeout_s)
        finally:
            for t in (agent_t, idle_t, human_t):
                t.stop()

        # build and return the ATIF trajectory
        pairs = list_llm_trace_pairs(run_dir / "raw")
        calls = 0
        for msg in turn.received:
            if isinstance(msg, AIMessage):
                if calls >= len(pairs):
                    raise RuntimeError(
                        f"McpClient wrote no LLM trace for call {calls} under {run_dir / 'raw'}; "
                        "every call must be captured whole"
                    )
                _, request_path, response_path = pairs[calls]
                request = json.loads(request_path.read_text())
                response = json.loads(response_path.read_text())
                append_ai_message_to_atif(
                    trajectory,
                    msg,
                    request=request_path,
                    response=response_path,
                    at=request["started_at"],
                    latency_s=response["latency_s"],
                )
                calls += 1
            elif isinstance(msg, ToolMessage):
                trajectory.observe(str(msg.tool_call_id), str(msg.content))
        return trajectory.build("answer" if finished else "timeout")
