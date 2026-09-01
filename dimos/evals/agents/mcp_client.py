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

from dimos.evals.agents.lib.langchain_recorder import LangChainRecorder
from dimos.evals.types import Environment, RunningEnvironment, Trajectory


class _Turn:
    """One McpClient turn as seen on the wire: every message on ``/agent``
    from the moment ``/agent_idle`` goes False until it comes back True.
    ``/agent_idle`` is its own topic and can overtake the last ``/agent``
    message, so the turn is done only once the received ``AIMessage``s match
    the raw trace, which is complete before the idle flip is published."""

    def __init__(self, raw_dir: Path) -> None:
        self.received: list[tuple[BaseMessage, float]] = []  # message, epoch it arrived
        self.done = threading.Event()
        self._started = threading.Event()
        self._lock = threading.Lock()
        self._raw_dir = raw_dir
        self._expected: int | None = None

    def on_agent(self, msg: Any) -> None:
        if isinstance(msg, BaseMessage):
            with self._lock:
                self.received.append((msg, time.time()))
                self._maybe_done()

    def on_idle(self, flag: Any) -> None:
        if flag is False:
            self._started.set()
        elif flag is True and self._started.is_set():
            with self._lock:
                self._expected = len(_pairs(self._raw_dir))
                self._maybe_done()

    def _maybe_done(self) -> None:
        if self._expected is not None and (
            sum(isinstance(m, AIMessage) for m, _ in self.received) >= self._expected
        ):
            self.done.set()


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
    capture is the McpClient's trace dir, which this agent points at
    ``run_dir/raw`` (the ``set_trace_dir`` RPC) as the turn starts —
    launched and attached McpClients alike.

    ``modules`` is the agentic composite appended to the case's stack, e.g.
    ``"unitree-go2-agentic"`` (the whole shipped stack; ``autoconnect``
    dedups the base it shares with the case); on a frozen ``Dataset`` it is
    the whole launched stack. ``""`` attaches to a dimos that is already
    running.
    """

    modules: str = ""

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """The shipped agent can call every tool its MCP server exposes."""
        return environment_tools

    def preflight(self, environment: Environment) -> None:
        from dimos.core.run_registry import list_runs

        if not environment.has_robot and not self.modules:
            raise RuntimeError(
                f"McpClientAgent needs a running McpClient; {type(environment).__name__} "
                "has no robot and this agent adds no modules"
            )
        if not self.modules and not list_runs(alive_only=True):
            raise RuntimeError(
                "McpClientAgent adds no McpClient and no dimos is running to attach to"
            )

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        from dimos.agents.mcp.mcp_client import McpClientConfig
        from dimos.core.transport_factory import make_transport
        from dimos.porcelain.dimos import Dimos

        app = Dimos.connect()
        try:
            mcp_client: Any = app.McpClient  # handle type depends on what's importable
            mcp_client.set_trace_dir(str(run_dir / "raw"))
        finally:
            app.stop()

        recorder = LangChainRecorder(
            inputs, name=type(self).__name__, model=McpClientConfig().model, raw_dir=run_dir / "raw"
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
            turn.done.wait()  # the runner's timeout is the only limit
        finally:
            for t in (agent_t, idle_t, human_t):
                t.stop()
        pairs = _pairs(run_dir / "raw")
        calls = 0
        for msg, at in turn.received:
            if isinstance(msg, AIMessage):
                if calls >= len(pairs):
                    raise RuntimeError(
                        f"McpClient wrote no LLM trace for call {calls} under {run_dir / 'raw'}; "
                        "every call must be captured whole"
                    )
                recorder.record(msg, request=pairs[calls][0], response=pairs[calls][1], at=at)
                calls += 1
            elif isinstance(msg, ToolMessage):
                recorder.observe(str(msg.tool_call_id), str(msg.content))
        return recorder.build("answer")
