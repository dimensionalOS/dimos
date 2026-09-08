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

from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING, Any

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environment import Environment
from dimos.evals.environments.lib.launch import default_mcp_url, validate_blueprints
from dimos.evals.types import RunningEnvironment
from dimos.memory.cli.dataset import open_dataset, resolve_dataset
from dimos.memory.store.base import Store
from dimos.memory.stream import Stream
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.evals.agent import Agent


class DatasetConfig(BaseConfig):
    name: str
    select: tuple[Callable[[Store], Stream[Any, Any]], ...] = ()
    mcp_url: str = ""
    launch_timeout_s: float = 300.0


class Dataset(Environment):
    """A frozen recording; ``select`` restricts the streams available to the agent.

    Agent modules launch as their own stack. ``mcp_url`` instead attaches to
    an existing MCP server; with neither, the recording stands alone.
    """

    config: DatasetConfig

    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(name=name, **kwargs)

    @property
    def has_robot(self) -> bool:
        return bool(self.config.mcp_url)

    def preflight(self, agent: Agent) -> None:
        if agent.config.modules and self.config.mcp_url:
            raise RuntimeError(
                f"Dataset({self.config.name!r}) already attaches to {self.config.mcp_url}; "
                f"{type(agent).__name__} also adds modules {agent.config.modules!r}"
            )
        if agent.config.modules:
            validate_blueprints(agent.config.modules)
        with open_dataset(self.config.name) as store:
            for select in self.config.select:
                select(store)

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        mcp_url = self.config.mcp_url
        if modules:
            proc = DimosCliCall()
            proc.simulator = None
            proc.demo_args = ["run", *modules]
            self._resources.callback(proc.stop)
            proc.start()
            mcp_url = default_mcp_url()
            if not McpAdapter(mcp_url).wait_for_ready(
                timeout=self.config.launch_timeout_s, interval=2.0
            ):
                raise RuntimeError(f"MCP at {mcp_url} not ready — is dimos up?")
        store = open_dataset(self.config.name)
        self._resources.callback(store.stop)
        streams = (
            [select(store) for select in self.config.select]
            if self.config.select
            else [store.stream(name) for name in store.list_streams()]
        )
        return RunningEnvironment(
            mcp_url=mcp_url,
            streams=streams,
            artifacts={"recording": resolve_dataset(self.config.name)},
        )
