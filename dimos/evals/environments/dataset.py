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

"""A frozen recording, restricted to what the task is about."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, ClassVar

from dimos.evals.environments.lib.launch import blueprint_modules, default_mcp_url
from dimos.evals.types import Agent, RunningEnvironment, Select

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store


@dataclass
class Dataset:
    """A frozen recording, restricted to what the task is about.

    ``select`` decides what the recording *contains* for this case ("the point
    cloud shown" is one lidar frame; "how far between t=20 and t=30" is that
    odom window); ``()`` keeps all of it. How the agent gets to see it is the
    agent's business. A tool-using agent brings its tool surface itself: its
    ``modules`` are launched as their own stack (``dimos run <modules>`` —
    there is no world for them to stand on) and torn down with the case.
    ``mcp_url`` attaches a running MCP server instead; with neither there is
    no robot.
    """

    name: str  # a memory dataset name ("go2_short") or a path
    select: tuple[Select, ...] = ()
    mcp_url: str = ""
    launch_timeout_s: float = 300.0  # module stack + MCP readiness; no simulator to boot

    artifacts: ClassVar[tuple[str, ...]] = ("recording",)
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)
    _proc: DimosCliCall | None = field(default=None, init=False, repr=False, compare=False)

    @property
    def has_robot(self) -> bool:
        return bool(self.mcp_url)

    def preflight(self, agent: Agent) -> None:
        if agent.modules and self.mcp_url:
            raise RuntimeError(
                f"Dataset({self.name!r}) already attaches to {self.mcp_url}; "
                f"{type(agent).__name__} also adds modules {agent.modules!r}"
            )
        if agent.modules:
            blueprint_modules(agent.modules)  # raises: unknown name
        from dimos.memory.cli.dataset import open_dataset

        store = open_dataset(self.name)  # raises: dataset unresolvable
        try:
            for sel in self.select:
                sel(store)  # raises "No stream 'x'. Available: [...]" — no data read
        finally:
            store.stop()

    def start(self, modules: str) -> RunningEnvironment:
        from dimos.memory.cli.dataset import open_dataset, resolve_dataset
        from dimos.memory.store.base import copy_streams
        from dimos.memory.store.memory import MemoryStore

        mcp_url = self.mcp_url
        if modules:
            from dimos.agents.mcp.mcp_adapter import McpAdapter
            from dimos.e2e_tests.dimos_cli_call import DimosCliCall

            proc = DimosCliCall()
            proc.simulator = None
            proc.demo_args = ["run", *modules.split()]
            proc.start()
            self._proc = proc
            mcp_url = default_mcp_url()
            if not McpAdapter(mcp_url).wait_for_ready(timeout=self.launch_timeout_s, interval=2.0):
                raise RuntimeError(f"MCP at {mcp_url} not ready — is dimos up?")
        source = open_dataset(self.name)
        try:
            streams = (
                [sel(source) for sel in self.select]
                if self.select
                else [source.stream(name) for name in source.list_streams()]
            )
            self._recording = copy_streams(streams, MemoryStore())
        finally:
            source.stop()
        return RunningEnvironment(
            mcp_url=mcp_url,
            recording=self._recording,
            artifacts={"recording": resolve_dataset(self.name)},
        )

    def settle(self, budget_s: float) -> None:
        return None

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
        if self._proc is not None:
            self._proc.stop()
            self._proc = None
