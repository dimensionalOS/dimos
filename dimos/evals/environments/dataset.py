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

from dimos.evals.types import Agent, RunningEnvironment, Select

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


@dataclass
class Dataset:
    """A frozen recording, restricted to what the task is about.

    ``select`` decides what the recording *contains* for this case ("the point
    cloud shown" is one lidar frame; "how far between t=20 and t=30" is that
    odom window); ``()`` keeps all of it. How the agent gets to see it is the
    agent's business. ``mcp_url`` attaches a running MCP server so a
    tool-using agent can act over the frozen recording; without it there is
    no robot.
    """

    name: str  # a memory dataset name ("go2_short") or a path
    select: tuple[Select, ...] = ()
    mcp_url: str = ""

    artifacts: ClassVar[tuple[str, ...]] = ("recording",)
    _recording: Store | None = field(default=None, init=False, repr=False, compare=False)

    @property
    def has_robot(self) -> bool:
        return bool(self.mcp_url)

    def preflight(self, agent: Agent) -> None:
        if agent.modules:
            raise RuntimeError(
                f"Dataset({self.name!r}) is a frozen recording and launches nothing; "
                f"{type(agent).__name__} adds modules {agent.modules!r}"
            )
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
            mcp_url=self.mcp_url,
            recording=self._recording,
            artifacts={"recording": resolve_dataset(self.name)},
        )

    def stop(self) -> None:
        if self._recording is not None:
            self._recording.stop()
            self._recording = None
